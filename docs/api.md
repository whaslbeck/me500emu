# Python API

The package is `me500emu` (source in `src/me500emu`). Three levels:

1. **`Session` + `run_job`** - a machine ready for jobs, and a runner that reports what it did. Enough for CAM testing.
2. **`Machine` from `snapshot_state.booted()`** - direct access to RAM, devices, axes and the run loop.
3. **`harness`** - helpers for measurements on a `Machine`.

All examples below run as they are from the project root (with the venv's Python, or `PYTHONPATH=src`), given a stock
ROM image in `roms/`.

**One machine per process.** The C core keeps one global state. Creating a new `Machine` or `Session` re-targets it,
so an older machine in the same process must not be used afterwards. Sequential use is fine; for parallel runs use
separate processes (`multiprocessing` with fork, or subprocesses of the CLI).

## ROM lookup and identification

```python
from me500emu import paths, rominfo

rom = paths.rom_path("stock")        # or "max", or a file path; raises paths.RomNotFound
print(rominfo.version(rom))          # '1.50', '1.50MAX BUILD 000019 ...', or None for an unknown image
print(rominfo.is_max(rom))           # True for a 1.50MAX image
print(paths.cache_dir())             # ME500EMU_CACHE or ~/.cache/me500emu (created if missing)
print(paths.data_path("nvram", "device_example.bin"))
```

`rominfo.version()` accepts a path or the image bytes. Lookup order and environment variables:
[rom-and-nvram.md](rom-and-nvram.md).

## Session

```python
from me500emu.session import Session, SessionError

s = Session(rom="stock", mode="std")
print(s.version, s.lcd())            # 1.50  [REMOTE]     0KB | SPIN-OFF COM-IIc | ZD:7.00 Z-ES:10 | ZC:10.00XYES:50
s.send("PA2000,1000;")               # stream with flow control, then wait for standstill
print(s.position_mm())               # (19.998, 9.998, 0.0) table mm (MECA CORRECT of the example machine)
print(s.time_s(), s.status())
```

`Session(rom="stock", mode="std", baud=9600, handshake="hard", nvram=None, z0_mm=None, origin_mm=(5.0, 5.0),
debug=False, rebuild=False, cache=True)`

| Argument | |
|---|---|
| `rom` | `"stock"`, `"max"` or a file path |
| `mode` | `"std"`; `"opt"` and `"gcode"` need a 1.50MAX image |
| `baud` | 9600, 19200 or 38400 - the emulated byte rate |
| `handshake` | `"hard"` (DTR flow control) or `"code"` (XON/XOFF; the session then ignores DTR) |
| `nvram` | NVRAM seed file (512 bytes); default `data/nvram/device_example.bin` |
| `z0_mm` | Z0 in mm below the top of the Z stroke, set like the panel's Z zero key. Default: 10 in G-code mode; in the HP-GL modes the firmware's value is left alone |
| `origin_mm` | XY origin offset in mm, applied in G-code mode only |
| `debug` | 1.50MAX: DEBUG = ON (enables the `~` measurement commands) |
| `rebuild` | cold-boot and set up again instead of using the caches |
| `cache` | save the state after setup as `session-<key>.snap` and reuse it |

Construction restores the boot snapshot (cold boot on first use), switches to REMOTE, sends `IN;SP1;PA0,0;` and waits
until the machine is idle at the origin, sets Z0, and in G-code mode switches the mode and sets the origin offset.
Invalid arguments raise `SessionError`; a missing ROM raises `paths.RomNotFound`.

Attributes: `s.m` (the `Machine`), `s.u` (its Unicorn object), `s.rom`, `s.version`, `s.is_max`, `s.mode`,
`s.baud`, `s.handshake`, `s.halted` (count of halt-loop hits).

| Method | |
|---|---|
| `send(data, settle=True, limit=None, on_step=None)` | stream `data` (str or bytes) through the UART with flow control; then, with `settle`, run until the axes stand still. Returns False if the firmware halted, the `limit` (time units) passed, or the axes did not settle. `on_step(session)` is called every 20000 units while sending. Settling waits at least 15 s of machine time. |
| `feed(data, window=16)` | move as many bytes into the UART's input queue as keep it at `window` bytes; returns the count taken. For writing your own streaming loop. |
| `run(units)` | run the machine for `units` time units (2000 = 1 ms) |
| `settle(limit=400000000)` | run until the axes have stood still for a while; False if not within `limit` |
| `read_tx()` | bytes the machine has sent since the last call |
| `press(col, bit, hold=None, after=None)` | press and release a panel key (see the key map in [hardware-model.md](hardware-model.md#key-matrix)) |
| `hold_until(col, bit, condition, max_steps=900)` | hold a key (jog) until `condition()` is true |
| `position_mm()` | `(x, y, z)` table mm from the axis model: X/Y from LOW LEFT, Z depth below the top |
| `z0_mm()` | the firmware's Z0 (table mm below the top) |
| `xy_origin_mm()` | the firmware's XY origin offset (table mm) |
| `lcd()` | the four LCD lines joined with ` \| ` |
| `time_s()` | emulated seconds since the session became ready |
| `status()` | `{strobes, halted, uart_overruns}` plus, on 1.50MAX, `{stop, error, overflow, line_overflow, dropped}` |
| `ring_empty()` | True when no motion record is queued in the executor |
| `byte(addr)`, `word(addr)` | read RAM |

Module constants: `INSTR_PER_S = 2000000`, `BAUD_INSTR` (time units per byte for each baud rate), `MODES`,
`MAX_CELLS` (the 1.50MAX status cells, valid for build 000019), `DEFAULT_NVRAM`.

## run_job

```python
from me500emu.session import Session
from me500emu.jobs import run_job

s = Session(rom="stock", mode="std")
r = run_job(s, open("examples/square.hpgl").read(), timeout_s=120, keep_points=True)
print(r["machine_time_s"], r["bounds_mm"], r["final_mm"], r["status"])
for t, x, y, z in r["points"][:5]:
    print(t, x, y, z)
```

`run_job(sess, text, timeout_s=3600.0, keep_points=False, progress=None)` streams `text` (str or bytes) and runs until
the machine is idle again (all bytes delivered, executor ring empty, axes still for 1.2 s), the firmware halts, or
`timeout_s` seconds of **emulated** time have passed. It returns the report described in
[cam-testing.md](cam-testing.md#the-report); with `keep_points` it also contains `points`, a list of
`(t_s, x, y, z)` in work coordinates for every change of position. `progress(sent, total, units)` is called after
every run step.

`jobs.PathRecorder(sess, offset)` is the recorder `run_job` uses; call its `harvest()` after each `m.run()` if you
build your own loop.

## Machine

Use `snapshot_state.booted()` to get a machine: it creates the `Machine`, sets the calibrated tick rates, and restores
or performs the boot.

```python
from me500emu import paths, harness as H
from me500emu import snapshot_state as snap

m = snap.booted(paths.rom_path("stock"), trace="count")     # factory NVRAM seed: ORIGIN CENTER
print(m.instr, m.panel.text())            # 95000000 ['[LCL]SPIN-OFF1/4', 'CONDITION     -~', ...]

H.remote(m)                               # tap REMOTE/LOCAL until the REMOTE screen shows
m.uart.rx.extend(b"IN;SP1;PA0,0;PU1000,0;")
H.drain(m)                                # run until the axes stand still
print(m.motion.strobes, {k: a.table_mm() for k, a in m.subcpu.axes.items()})
# ... {'X': 251.5, 'Y': 152.5, 'Z': 0.0}: CENTER origin + 10 mm
```

`snapshot_state.booted(rom, cache=None, physics=True, trace="fast", boot_instr=95000000, rebuild=False,
on_create=None, **kw)` - `kw` goes to `Machine`, most usefully `nvram_seed=` (a seed file; default
`data/nvram/iic10_default.bin`, the factory contents - note that `Session` defaults to `device_example.bin`
instead) and `nvram_path=` (a persistent NVRAM file, read at construction; saving it is up to you:
`m.nvram.save()`). `cache=` overrides the snapshot file; `on_create(m)` is called before the cold boot (the UI uses it
to be able to abort). Restoring also arms the instruction costs.

`Machine(rom_path, strict=True, trace_ports=True, sensor_value=None, physics=False, trace="count", nvram_path=None,
nvram_seed=<iic10_default.bin>)` can be constructed directly, but then the tick intervals are 0 (no interrupts), the
sub-CPU signature is missing and costs are not armed - use `booted()` unless you are studying the boot itself.

### Running

| | |
|---|---|
| `m.run(max_instr, slice_size=20000)` | run until `m.instr` reaches the absolute value `max_instr`; returns `m.instr`. Usual form: `m.run(m.instr + n)`. |
| `m.instr` | the clock, in 0.5 us units (see [timing.md](timing.md)); writable |
| `m.abort = True` | make a running `run()` return at the next slice boundary (from another thread) |
| `m.jitter = seed` | phase jitter of interrupt delivery; 0 = off |
| `m.faults` | list of fault messages (bus faults, unknown ports, Unicorn errors with PC and symbol) |
| `m.tick_interval`, `m.motion_tick_interval`, `m.service_tick_interval`, `m.uart_byte_interval` | INT 23h, INT 20h, INT 24h periods and the UART byte time, in units |
| `m.pending_irqs`, `m.irq_dispatches`, `m.dropped_irqs` | interrupt diagnostics |
| `m.uart_overruns` | bytes lost in the 8251 receive register |
| `m.cpu.uc` | the Unicorn object: `mem_read`, `mem_write`, `reg_read`, `hook_add` ... |

Trace modes (`trace=`): `"fast"` (no exact instruction counting), `"count"` (exact, and `m.watch = {addr: 0}` counts
executions of addresses), `"full"` (also `m.last_pc` and `m.recent`, the last 24 addresses). See
[architecture.md](architecture.md#trace-modes).

### Devices

| | |
|---|---|
| `m.uart.rx` | bytes waiting to be received (a `bytearray`; `extend()` to send). Delivery is paced by `m.uart_byte_interval` and DTR. |
| `m.uart.tx` | bytes the machine has sent (a `bytearray`; clear it yourself) |
| `m.uart.honour_dtr` | True: no byte is delivered while the firmware holds DTR low |
| `m.uart.dtr`, `m.uart.ctrl` | current DTR state / last 8251 command byte |
| `m.panel.press(col, bit)`, `m.panel.release_all()` | key matrix; hold a key for `harness.KEY_HOLD` units |
| `m.panel.text()` | the four LCD lines (list of str) |
| `m.panel.ddram`, `m.panel.keys` | raw display RAM, raw matrix columns |
| `m.subcpu.axes["X"\|"Y"\|"Z"]` | axis model: `.pos` (pulses from the lower end), `.pending`, `.table_mm()`, `.origin_steps`, `.travel_steps`, `.STEPS_PER_MM`, `.at_home()`, `.clipped`, `.max_command` |
| `m.subcpu.over_cap()`, `m.subcpu.follow_report()` | axes whose per-tick command exceeded the firmware's own maximum; backlog statistics |
| `m.motion.strobes` | motion commands issued so far |
| `m.motion.pos` | commanded position per axis (pulses, the accumulator of all strobes; not clipped at end stops) |
| `m.motion.path` | commanded position after each strobe since the path buffer was last emptied (`m.fc.motion.path_n = 0`) |
| `m.nvram.cells` | the 512 NVRAM cells (`bytearray`); `m.nvram.parameters()`, `m.nvram.checksum_ok()`, `m.nvram.save(path)`, `m.nvram.dirty` |
| `m.pic` | 8259 state view: `isr`, `imr`, ... |
| `m.pages.regs`, `m.store.mem` | page registers and the 1 MiB paged store |

Replacing `m.motion.write` or `m.subcpu.consume_step` with your own function is supported (the core then routes those
accesses through Python), at a large speed cost.

## Snapshots

```python
from me500emu import paths, harness as H
from me500emu import snapshot_state as snap
from me500emu.machine import Machine

rom = paths.rom_path("stock")
m = snap.booted(rom)
H.remote(m)
snap.save(m, "/tmp/remote.snap")          # the machine's full state

m2 = Machine(rom, physics=True, trace="fast")   # same ROM and NVRAM seed as m; m must not be used any more
m2.subcpu.sign_window()
snap.load(m2, "/tmp/remote.snap")         # raises snap.StaleSnapshot on a mismatch
m2.arm_costs()                            # load() does not arm the class costs
print(m2.panel.text()[0])                 # [REMOTE] ...
```

What is saved and the cache keys: [architecture.md](architecture.md#snapshots).

## Harness helpers

`me500emu.harness` collects what measurement scripts need again and again:

| | |
|---|---|
| `drain(m, settle=20000000, chunk=4000000, quiet=3, limit=300000000)` | run at least `settle` units, then until the axes have stood still for `quiet` chunks; False if that does not happen within `limit` more units. Always check the result. |
| `remote(m, tries=3)` | tap REMOTE/LOCAL until the LCD shows `[REMOTE]`; returns success |
| `ready(approach=APPROACH, **kw)` | stock machine from the snapshot, REMOTE, approach `IN;SP1;PA0,0;` done and settled (uses `booted()`'s default NVRAM, i.e. factory contents with ORIGIN CENTER) |
| `set_z0_surface(m, z_steps=2000)` | set Z0 like the Z zero key (`[0x04b6]`, 5 um steps below the top) |
| `counting(m, {addr: name})` | context manager; returns a `Counter` of executions of physical addresses inside the block |
| `watching(m, lo, hi, out)` | context manager; appends `(address, size, value, pc)` for every write into `[lo, hi]` |
| `moving_strobes(m, job, settle=30000000)` | send a job, settle, return `(strobes that moved an axis, axis deltas, settled)` |
| `units(mm)`, `steps(mm)`, `line(n, mm, axis, sign)` | MGL-IIc-10 units, X/Y pulses, a job of `n` equal chords |
| `KEY_HOLD`, `KEY_TAP`, `KEY_GAP` | key timing in units (183 ms, 122 ms, 0.9 s) |

Example - count how often a firmware routine runs during a move (the address is physical: `8000:5be6` = `0x85be6`):

```python
from me500emu import paths, harness as H
from me500emu import snapshot_state as snap

m = snap.booted(paths.rom_path("stock"), trace="count")
H.remote(m)
m.uart.rx.extend(b"IN;SP1;PA0,0;PU1000,0;")
with H.counting(m, {0x85BE6: "chain_tick"}) as c:
    assert H.drain(m)
print(dict(c))
```

Physical addresses in the ROM are `0x80000 + file offset`. `symbols.name(addr)` gives the nearest symbol name from
`data/symbols.txt`; `regions.region(addr)` a coarse functional region (boot, panel, parser, planner, executor, step,
kernel, math).
