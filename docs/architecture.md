# Architecture

How the emulator is put together: components, the run loop, the memory and I/O map as modelled, interrupts, the
sub-CPU model, snapshots and the time/cost model. For what the modelled hardware *is* (axes, keys, NVRAM), see
[hardware-model.md](hardware-model.md); for the time scale, [timing.md](timing.md).

## Components and data flow

```
            host side                                 emulated controller
  +------------------------------+      +---------------------------------------------------+
  | cli.py   run / boot / info   |      |  Unicorn (x86, 16-bit real mode) = main V33       |
  | jobs.py  run_job, recorder   |      |    + cpu.py: BRKXA/RETXA, IVT dispatch            |
  | bridge.py  pty <-> UART      | ---> |                                                   |
  | ui/server.py  HTTP + 3D view |      |  fastcore.c (C): slice loop, 8259, timer ticks,   |
  | session.py  Session          |      |    UART pacing, interrupt delivery, class costs,  |
  +--------------+---------------+      |    motion window, position counter, axis model    |
                 |                      |                                                   |
                 v                      |  machine.py (Python): memory map, port decoding,  |
  +------------------------------+      |    devices.py: panel/LCD, UART, PIC view, NVRAM,  |
  | snapshot_state.py  booted()  |      |    page registers, paged store, end switches      |
  | ~/.cache/me500emu/*.snap     |      |  peer.py / fastdev.py: sub-CPU and axes           |
  +------------------------------+      +---------------------------------------------------+
```

- **Unicorn** executes the firmware. The V33 is run as an x86 in 16-bit real mode; its two non-x86 instructions
  that the firmware uses (`BRKXA` at `8000:0457` and `RETXA` at `8000:0470`, V33 expanded addressing) are
  trapped by address-ranged code hooks in `cpu.py`.
- **The C core** (`fastcore.c`, loaded through ctypes by `fastcore.py`) holds everything that runs per
  instruction, per interrupt or per motion strobe. It is compiled with `gcc` on first import against the Unicorn
  headers shipped in the `unicorn` wheel; it does not link against the library but receives the addresses of the
  entry points the Python binding already loaded. If the package directory is not writable (installed read-only),
  the library is built into the cache directory. `CC` selects the compiler. There is no pure-Python fallback any
  more (`FASTCORE=0` aborts).
- **Python devices** (`devices.py`, `fastdev.py`, `peer.py`) are reached through callbacks from the C core for
  every port access and every access to a memory-mapped device window. Most device state that the C core needs on
  every slice (PIC registers, UART pacing, motion window, axes) lives in a C struct; the Python objects are views
  onto it, so scripts can read and set attributes such as `m.pic.imr` or `m.subcpu.axes["X"].pos`.
- **`Machine`** (`machine.py`) wires it all together and exposes `run()`.
- **Host-side tools** - `Session`, `run_job`, the bridge and the UI - only talk to the machine the way a host or an
  operator would: bytes into the UART, key presses into the matrix, and reads of RAM cells for status.

**The C core is a process-wide singleton.** It keeps one global state; creating a second `Machine` in the same
process re-targets the core. Use one machine per process; run several jobs in parallel with separate processes.

## Memory map

The V33 has a 1 MiB physical address space. Modelled regions (`bus.py`):

| Physical | Size | What |
|---|---|---|
| `0x00000` | 64 KiB | RAM. Only segment 0 is ever used by the firmware (DS=SS=ES=0, stack at the top). |
| `0x20000` | 256 B | **Sub-CPU window** (`2000:0000`): shared RAM with the motor-control CPU; the motion request registers and the strobe live here. |
| `0x24000` | 1 KiB | NVRAM: 512 byte-wide cells at **even** offsets. Writes are gated by port `0x0a`. |
| `0x30000` | 64 KiB | paged window "producer" (page registers `0xff18..0xff1e`) |
| `0x40000` | 64 KiB | paged window "consumer" (`0xff20..0xff26`) |
| `0x50000` | 64 KiB | paged window "probe" (`0xff28`) |
| `0x60000` | 64 KiB | paged window "font" (`0xff30/0xff32`) |
| `0x80000` | 512 KiB | the EPROM image, read/execute only. Reset vector at `0xffff0` jumps to `8000:0000`. |

The four windows share one 1 MiB backing store (`PagedStore`), translated through the 64 V33 page registers at
ports `0xff00..0xff7e`: `physical = page[L >> 14] * 0x4000 + (L & 0x3fff)`. The firmware's 1 MB HP-GL receive
ring ("1 [MB]" on the LCD) lives there. Port `0xff80` bit 0 reports whether expanded addressing is active.

**Fail loudly.** An access outside a known region, a read or write past the real size of a device, an `IN`/`OUT`
to a port not in the known-port list: each stops the run and appends a message to `m.faults`, naming the address,
the PC and the nearest Ghidra symbol (`data/symbols.txt`). Nothing answers with a silent zero.

## I/O ports

| Port | Device | Model |
|---|---|---|
| `0x04` | LCD data (write), LCD status (read, busy flag always clear) | `devices.Panel` |
| `0x07` | LCD control word | logged |
| `0x08` | key matrix input (active low) | `Panel.keys` |
| `0x09` | panel latch: E strobe (bit 7), R/W (6), RS (5), key column select (bits 2-4) | `Panel` |
| `0x06` | sensor register: end switches (bits 4-6, set = actuated), attachment switches (bits 0-1) | `devices.Endstops` |
| `0x0a` | board control; bits 6-7 clear = NVRAM write enabled | `Machine.board_0a` |
| `0x10`, `0x11` | 8259 PIC | C core + `devices.Pic` view |
| `0x14`, `0x15`, `0x17` | 8253 counters 0 and 1 (read back / latch) | `Machine._ctr0_live/_ctr1_live` |
| `0x18`, `0x19` | 8251 UART data / status+command | `devices.Uart` (+ pacing in C) |
| `0x30`, `0x34` | 16-bit position counter, latched (the service menu shows it as "FLAT") | C core |
| `0xff00..0xff7e` | V33 page registers | `devices.PageRegisters` |
| `0xff80` | expanded-addressing status | `PageRegisters` |

A few more ports are written once during boot and only logged (`0x0b`, `0x16`, `0x1c`, `0x1d`, ...).

## Interrupts

The firmware programs the 8259 for edge-triggered, single, **no auto-EOI**, vector base `0x20`, with IRQ5..7
masked. The core raises:

| IRQ | Vector | Source | Emulated period |
|---|---|---|---|
| 0 | INT 20h | motion tick (8253 counter 0) - the step engine | 2000 units = 1.0 ms |
| 1 | INT 21h | UART receive (RxRDY) | one byte per byte-time at the chosen baud rate |
| 2 | INT 22h | UART transmit (TxRDY) | when the holding register is empty and TxEN is set |
| 3 | INT 23h | kernel timer (8253 counter 1, 50000 counts) | 20340 units = 10.17 ms |
| 4 | INT 24h | service tick of the serial frontend (counter 2, 5000 counts) | 2035 units = 1.017 ms |
| - | INT 27h | 8259 default vector: IRQ1 requested but RxRDY gone again before the acknowledge | |

The PIC is modelled in fully nested mode: while an IRQ is in service, only a higher-priority request is delivered,
until the firmware's non-specific (or specific) EOI clears the in-service bit. Rotation commands, OCW3 IRR/ISR
reads and the mask register are modelled. A handler that forgets its EOI therefore blocks lower-priority interrupts
here as it would on the machine.

Software interrupts (`INT n` in the firmware) are dispatched through the IVT as the CPU would. INT 40h..47h are
the system calls of the cooperative multitasking kernel in the ROM (INT 43h is its yield and runs constantly);
`service.py` is only a logging fallback for a kernel vector the firmware has not installed.

## The run loop

`Machine.run(max_instr, slice_size=20000)` runs until the time counter `m.instr` reaches the **absolute** value
`max_instr` (so the usual call is `m.run(m.instr + n)`). The first call starts at the reset vector; later calls
continue from the current `CS:IP`.

Unicorn does not raise interrupts itself, so the core runs the CPU in **slices** and plays the interrupt controller
between them (`fc_run` in `fastcore.c`):

1. The slice budget is `slice_size`, capped at the next due tick, UART byte or timer deadline (the cap is taken a
   bit short because instruction costs can reach the deadline earlier). While an interrupt is pending and IF is
   set, the budget is 1 instruction, so delivery happens at the next instruction boundary; while IF is clear, a
   watch in the code hook ends the slice as soon as IF comes back.
2. After the slice, time is advanced (see "Time and instruction costs"), the axis model is advanced by the elapsed
   time, the UART delivers the next byte if its byte time has come (and DTR allows it), and the tick deadlines raise
   their IRQs.
3. If IF is set, the highest-priority acceptable pending IRQ is dispatched through the IVT (push flags/CS/IP, clear
   IF and TF, jump to the vector).

**Phase jitter** (optional): `m.jitter = seed` (non-zero) randomises the delivery delay of pending interrupts and
scatters every tick period by +-1/8. Without it, interrupts land at the same program points on every run, because
both the firmware and the ticks are deterministic; a race that would show on the machine can then stay invisible.
`m.jitter = 0` switches it off (default).

`m.abort = True` (from another thread) makes `run()` return at the next slice boundary; the UI uses this for a clean
Ctrl-C.

## The sub-CPU and the axis model

The ME-500 has a second V33, the motor-control CPU, which drives the brushed DC servo motors through a gate array
(PWM out, encoder in). Its firmware is not part of the main EPROM (several independent searches came up empty) and
has not been dumped so far, so this CPU is **not emulated but simulated**: `peer.py` (with its C counterpart in
`fastcore.c`) is a behavioural model of what the main CPU sees of it:

- **Handshakes** in the shared window, as decoded from the firmware: the "MIMAKI" signature at offsets
  `0x1a..0x1f`, the liveness byte at `0x10`, the `0xAA`/`0x55` byte-by-byte sync (answered only while the main CPU
  is inside its sync loop) and the block-complement handshake at `0x00`/`0xff`. They are answered instantly.
- **Motion requests.** Three axis groups of three words each at `0x80` (X), `0x88` (Y), `0x90` (Z), and a strobe at
  `0x98`. A write of 1 to the strobe is one motion command: the sum of each group's three words is the axis delta in
  encoder pulses. The core accumulates the commanded position (`m.motion.pos`), counts strobes (`m.motion.strobes`),
  appends the position to a path buffer, advances the position counter at ports `0x30/0x34` and hands the delta to
  the axes.
- **Axes** (`m.subcpu.axes["X"|"Y"|"Z"]`). Each axis takes the delta as owed motion (`pending`) and pays it out at
  a fixed rate (`INSTR_PER_STEP = 20` time units per pulse, i.e. up to 100 pulses per 1 ms tick, more than the
  firmware ever commands), stopping at the hard ends of its travel. The end switches on port `0x06` follow the axis
  positions. There is no control loop, no following error and no current.

After the boot's homing run, `rebase_to_origin()` puts all three axes on the firmware's LOW LEFT origin, so the
483 x 305 mm table lies physically in front of them. Geometry and scale: [hardware-model.md](hardware-model.md).

## Snapshots

A cold boot runs about 95 million time units (the power-on self-test and the homing run) and takes 1-2 minutes of
wall clock. It is deterministic, so `snapshot_state.booted()` runs it once and saves the result; later calls restore
it in about a second.

**What is saved:** all mapped RAM and ROM contents, the CPU registers, the XA mode flag, page registers and the paged
store, NVRAM cells, the sensor idle value, UART command byte, PIC state, the motion window registers, commanded
position and strobe count, the LCD DDRAM and address counter, the key matrix, the axis positions/pending/clip
counters, the axis geometry, the "rebased" flag and the three tick intervals. **Not saved:** anything the host owns -
Unicorn hooks, callbacks, the Unicorn object. A restore builds a fresh `Machine` and pours the state into it.

**Cache keys.** Snapshots live in the cache directory (`ME500EMU_CACHE`, default `~/.cache/me500emu`):

- `booted-<md5 of the ROM, first 8 hex>-<crc32>.snap`, where the CRC covers the NVRAM contents, the sensor idle
  value and the axis geometry. (Only the historic combination - factory NVRAM, sensor idle `0x8F`, old geometry -
  uses the plain `booted-<md5>.snap`.) A different NVRAM is a different machine, because the boot copies NVRAM
  blocks into RAM.
- `session-<key>.snap`: the state after `Session`'s setup (REMOTE, approach move, Z0, mode), keyed on the ROM,
  NVRAM, mode, baud rate, handshake, Z0, origin and debug flag, so repeated sessions skip that setup too.
- `booted-ui-<rom file name>-<crc of the UI NVRAM file>.snap`: the UI's own boot snapshot.

**Safety checks on load:** a snapshot of a different format version, a different axis geometry, or one that carries
a different ROM image than the machine being restored raises `StaleSnapshot`; `booted()` then cold-boots again
instead of silently restoring the wrong machine. Pass `rebuild=True` (CLI: `me500emu boot --rebuild`) after changing
anything that affects the boot. Deleting the cache directory is always safe.

## Time and instruction costs

`m.instr` is the emulator's clock. It is called "instructions" throughout the code for historical reasons, but it is
a **time unit of 0.5 us** (2000 units = 1 ms, the measured motion tick):

- every executed instruction counts 1 unit, a `rep` string instruction 1 unit per repetition;
- after the boot, a **class cost** is added per instruction: a signed delta per instruction start in the ROM,
  precomputed by `costs.py` from a Capstone disassembly of the image (register, memory read, memory write,
  read-modify-write, multiply/divide, port I/O, `rep` word moves, jumps), plus a surcharge for each access to the
  sub-CPU window and a surcharge for every instruction inside the executor/step-engine range `0x84300..0x86300`.
  The deltas are summed in the code hook in sixteenths and folded into `m.instr` at the end of each slice.

The cost table is cached per image and parameter set as `costs_<hash>.tab` in the cache directory. Costs are
**armed only after the boot** (`Machine.arm_costs()`, called by `booted()`): the boot's homing run was tuned on flat
time, and arming the costs earlier makes it fail with `ERR52` (Z sensor). The numbers and how they were fitted are in
[timing.md](timing.md).

## Trace modes

`Machine(..., trace=...)` (and `booted(..., trace=...)`):

| Mode | Counting | Extras | Use |
|---|---|---|---|
| `"fast"` | time follows the granted slice budget; no per-instruction counting hook for time | - | Session, runner, bridge, UI |
| `"count"` | every hook firing counts (exact per instruction and per `rep` iteration) | `m.watch = {addr: 0, ...}` hit counters | measurements that compare counts |
| `"full"` | as `count` | also `m.last_pc` and `m.recent` (the last 24 executed addresses) | boot tracing, debugging |

`count` and `full` were verified bit-identical against the retired pure-Python core. Class costs apply in all three
modes. In `fast` mode, time can be off by a few units wherever a hook stops the engine early; that does not change
which paths the firmware takes.
