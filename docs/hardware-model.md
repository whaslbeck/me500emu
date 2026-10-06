# The machine as modelled

What the emulator believes the ME-500 controller and machine to be, and where each belief comes from. Three
sources are used throughout: the firmware itself (how it drives a port or a cell), the Mimaki service and operation
manuals, and measurements on a real machine. Anything that is only a working assumption is marked as such.

## Overview

| Part | Real machine | Emulator |
|---|---|---|
| Main CPU | NEC V33 | Unicorn x86-16 + trapped `BRKXA`/`RETXA`; runs the EPROM image |
| Sub CPU (motor control) | NEC V33 + gate array (PWM, encoder inputs) | simulated (behavioural model); its firmware has not been dumped |
| Firmware | one 27C4096 EPROM, 512 KiB | your image, loaded at `0x80000` |
| Axes | brushed DC servo motors with incremental encoders | ideal encoder-count model, no control loop |
| Operator panel | 16x4 character LCD, 17 keys on a 3x8 matrix | HD44780-style DDRAM model, key matrix |
| Serial | 8251 USART, RS-232C | 8251 model with byte timing, DTR, XON/XOFF |
| Interrupts, timers | 8259, 8253 | modelled in the C core |
| Configuration | NVRAM (EEPROM), 512 cells | cell array, seeded from a file |

## CPUs

The controller has two V33s. The **main CPU** runs the whole EPROM: boot and self-tests, the cooperative
multitasking kernel with its three tasks, the HP-GL/MGL frontend, the planner and the step engine that emits one
motion command per 1 ms tick. The **sub CPU** is the motor controller; it closes the servo loops. Its firmware is
not part of the main EPROM (several independent searches came up empty) and has not been dumped so far, so the
emulator does not emulate this CPU but simulates its behaviour - a model of what the main CPU sees of it: the handshakes in the shared window and axes that follow the commanded motion. See
[architecture.md](architecture.md#the-sub-cpu-and-the-axis-model).

Unicorn executes x86. The V33 is an 80186-class CPU with a few NEC extensions; the firmware uses only `BRKXA` and
`RETXA` of those, and they are trapped. The other direction is **not** checked: an instruction the V33 does not
have (for example a 386 conditional jump with a 16-bit displacement) runs fine in the emulator and fails on the
machine. This matters for anyone building patched images.

## Axes

| | X | Y | Z |
|---|---|---|---|
| Resolution | 2000 pulses/mm (0.5 um) | 2000 pulses/mm | 4000 pulses/mm (0.25 um) |
| Table / stroke | 483 mm | 305 mm | 62 mm |
| Modelled travel | 483 + 2 x 23 mm | 305 + 2 x 8 mm | 62 mm |
| Origin (LOW LEFT) | 23 mm above the lower end | 8 mm above the lower end | top of the stroke |
| Reported range | -23 ... 506 mm | -8 ... 313 mm | 0 ... 62 mm (down) |

- **Resolution** is confirmed three ways: measured from the commanded motion, computed by the firmware itself, and
  stated by the service manual's error thresholds (Err 40/41: 40 pulses = 0.02 mm for X/Y; Err 42: 50 pulses =
  0.0125 mm for Z). What the firmware calls a step is an encoder pulse.
- **Table and stroke** come from the system parameters in NVRAM (no. 21-23: 483, 305, 62 mm).
- **Origin margins** (23 mm in X, 8 mm in Y beyond the table) come from the service manual's reference-point
  procedure (pen at the origin, 23 +- 1 mm / 8 +- 1 mm from the table edge). The overtravel beyond the far edge is
  undocumented; the model assumes the same margins (*assumption*).
- **Frame after the boot.** The rest position after the homing run *is* the LOW LEFT origin: with ORIGIN LOW LEFT,
  `PA0,0` after boot commands no motion at all; with ORIGIN CENTER it commands exactly +241.5 / +152.5 mm. After the
  homing run the model therefore puts all axes on the origin (`SubCpu.rebase_to_origin()`); from then on "table
  coordinates" are mm from LOW LEFT, and Z is the depth below the top of the stroke (positive = down).
- **MECA CORRECT.** System parameters no. 24/25 (NVRAM cells `0x030`/`0x032`) are the factory scale correction of an
  individual machine (ROM default 6000). The firmware scales position to pulses by `6000/corr`; the axis model uses
  `2000 * 6000 / corr` pulses per mm to match. With the example NVRAM (5998/5999) reported positions therefore differ
  from the nominal ones by a few micrometres over tens of millimetres - as they would on that machine. The factory
  seed (6000/6000) gives exactly 2000.
- **Axis speed in the model** is capped at 100 pulses per 1 ms tick (20 time units per pulse); the firmware never
  commands more than 40 per tick for X/Y (80 mm/s). So the model axis always keeps up.

### Homing and end switches

The homing run is performed by the firmware against the model, not skipped. What the model needs for it:

- X's switch at the high end, Y's at the low end (measured from the directions of the firmware's seek moves).
- Z: the sensor (port `0x06` bit 6) follows the profile measured on a real machine (below), and Z starts at its rest
  position, the top of the model's travel. (Until 2026-10-07 a fitted 3 mm "dog" band 32880 +- 6000 pulses from the
  bottom, with Z starting 87120 pulses above it, stood in for the unknown sensor.)
- At power-on X and Y start 10 mm from their switches. The firmware's seek moves are bounded (15-20 mm), so the real
  machine too cannot reference from an arbitrary position.

Port `0x06`, sensor register: bits 4-6 are the end switches, **set = actuated** (established from the escape move at
`8000:1021`, which runs when the bit is set). The idle value `0x8D` presents all switches free and the chip-removal
attachment as not fitted (bit 1 set would make the firmware refuse every Z depth command with `ERR65`). **Which bit
belongs to which axis is not established**; the model's choice (0x20 X, 0x10 Y, 0x40 Z) is a placeholder that boots.

**Measured on a real machine (2026-10-07, firmware 1.50MAX build 000021, two runs):** read in REMOTE with Z on its top
position after homing, port `0x06` is `0x61` right after power-up and `0x69` after the first LOCAL / `<MOVE>` visit. Bit 6
is set there in every reading, consistent with the Z origin sensor being actuated at the reference position (table Z 0).
The model reads `0x8D` as its idle value (bits 7, 5 and 2 differ from the machine, bit 3 is not modelled). On the same
machine the panel jog stops by itself at 62.000 mm below the top (the firmware's Z limit), and the top of the jog is
table Z 0.

**Z sensor profile (measured; modelled after homing, see CHANGELOG).** A second measurement (`zprofil`: Z driven by G-code from the top
position down 8 mm and back in 0.25 mm steps, port `0x06` read at every step; two runs with different Z0, identical
edges, no hysteresis at that resolution) gives bit 6 against the depth below the top position:

| mm below the top | bit 6 |
|---|---|
| 0 ... 2.00 | 1 |
| 2.25 ... 3.75 | 0 |
| 4.00 ... 6.00 | 1 |
| from 6.25 | 0 |

The model returns exactly this profile (`peer.Z_PROFILE_SET_MM`, edges halfway between the measured points), during
the homing run as well. The firmware's Z reference centres on the clear gap:

1. `8000:148d` (coarse): bit 6 set? then nothing to do - the case at the rest position.
2. `8000:14f2` (fine): down until clear (upper gap edge, 2.125 mm); 3 mm up; the bit is set there, so down until clear
   again and remember that position (`[0x08a8]`, written only at `8000:1643`); down until set (lower gap edge,
   3.875 mm); to the centre between both (3.0 mm); declare that table `[0x06cc]` = model record field `[0x06a8]` (3) x
   200 = 3.0 mm (`8000:16fb`); then drive to table 0.

With the measured profile the rest position comes out at 3.0 - 3.0 = 0.0 mm, the measured rest position. The model's
cold boot runs exactly this path and ends at the top with bit 6 set.

Two consequences, both *probable* rather than measured: the reference's 3 mm hop goes 0.875 mm above the rest position,
and the machine boots without `ERR42`, so there is at least that much room above table 0; `G0 Z5` with Z0 3 mm below the
top (2 mm above table 0) did give `ERR42`, so there is less than 2 mm. The model has no room above table 0 - its top end
of travel is the rest position - and simply clips the hop, which leaves the firmware's centring unaffected (it re-assigns
the table at the centre). From a start position below the lower set region the 3 mm hop would land in the gap and the
firmware takes the other branch; the predicted rest position is then about 0.5 mm lower (not measured; the model always
starts at the rest position).

Earlier versions of this model started Z far below a fitted dog: the opening blind move (two large strobes, -55 120
pulses) was then still running when the coarse seek began, and the model axis ran on by the remaining steps after the
firmware stopped the seek (14 584 pulses). Starting at the rest position removes that path; how the sub CPU executes a
blind move on the real machine is not known.

The Z axis is not a reliable witness near its end stops: the model stops dead at the end of travel and drops the rest
of a command, while the firmware keeps believing in its target.

## Operator panel

### LCD

16 columns x 4 lines, HD44780-style: the firmware writes a byte to port `0x04` and latches it with the falling edge of
the E bit on port `0x09` (RS = bit 5 selects data vs. command). The model keeps the 128-byte DDRAM and the address
counter; the four lines are at DDRAM `0x00`, `0x40`, `0x10`, `0x50`. `m.panel.text()` returns them. The busy flag
always reads clear.

### Key matrix

Three columns, selected by port `0x09` bits 2/3/4, eight bits each, read on port `0x08`, active low.
`m.panel.press(col, bit)` presses a key, `m.panel.release_all()` releases everything. The firmware scans in its
10.17 ms timer tick and debounces; a press must be held for several ticks. The harness uses 18 ticks
(`harness.KEY_HOLD`, 183 ms) and a gap of 0.9 s (`KEY_GAP`) before the next key.

The service manual has no matrix table. The assignment below comes from key sweeps in the emulator (pressing every
position in every relevant screen and watching the LCD **and** the axes) and from the panel paths the bridge drives:

| Matrix `col/bit` | Key | Evidence |
|---|---|---|
| `0/0`, `0/1` | PAGE forward / back | page 1/4 -> 4/4 -> 2/4 |
| `0/7`, `0/4` | **F1 +** / **F1 -** (LCD line 2) | opens `<CONDITION>`; XY-ES counts up with `0/7`, down with `0/4` (twelve values, 0.5 ... 50 mm/s) |
| `0/6`, `0/3` | **F2 +** / **F2 -** (line 3) | opens `<TEST CUT>` |
| `0/5`, `0/2` | **F3 +** / **F3 -** (line 4) | opens `<DATA CLEAR>`; `0/5` steps COMMAND forward, `0/2` backward |
| `1/0` | CANCEL / CE | closes dialogs |
| `1/1` | SPINDLE ON/OFF | `SPIN-OFF` -> `SPIN-ON` on the LCD |
| `1/2` | END | executes in `<DATA CLEAR>`, starts the test cut in `<TEST CUT>`, confirms in `<CONDITION>` and `<Z AXIS>` |
| `1/3` | REMOTE/LOCAL | toggles the REMOTE screen |
| `1/4` | X + | in `<MOVE>`: X moves positive |
| `1/7` | X - | |
| `1/5` | Y + | |
| `1/6` | Y - | |
| `2/2` | PAUSE | `[PAUSE]` screen |
| `2/3` | Z AXIS / depth | opens `<Z AXIS> DEPTH` |
| `2/5` | Z jog | in `<MOVE>`: Z moves down (depth increases); the bridge's `zero` command uses it this way |
| `2/6` | `<MOVE>` / Z jog other direction | opens `<MOVE>` from the LOCAL menu (used by the bridge) |
| `2/7` | XY origin key | in `<MOVE>`: sets the XY origin at the current position (used by the bridge) |
| `2/4` | Z zero key | in `<MOVE>`: sets Z0 at the current Z position (used by the bridge) |
| `2/0`, `2/1` | - | no effect found on display or motion |

The operation manual lists 17 keys; the four F keys are pairs of + and - next to the LCD lines, which is why the matrix
has more used positions than the panel has key caps. The mapping is functional (what each position does), not a
reading of the key caps.

## NVRAM

512 byte-wide cells, memory-mapped at `0x24000` on even addresses. Writes are ignored (and logged) unless port `0x0a`
bits 6-7 are clear. At boot the firmware checks an 8-bit sum over cells `0x000..0x03f` against cell `0x1ff` and
copies these blocks into RAM:

| Cells | RAM | Contents |
|---|---|---|
| `0x000` x 0x40 | `0x0680` | the 32 **system parameters** (2 bytes each; names in the service manual): encoder resolution, valid areas (483 / 305 / 62 mm), MECA CORRECT X/Y (no. 24/25), model selection, LCD language, ... |
| `0x100` x 0x14 | `0x0800` | settings |
| `0x140` x 0x18 | `0x09ae` | speed block: XY-ES, Z-ES, XY-MS, Z-MS in 0.1 mm/s, then accelerations (partly identified) and two more words |
| `0x180` x 0x0e | `0x0a7c` | settings, including COMMAND (cell `0x184`, 1 = MGL-IIc 10 um) and ORIGIN (cell `0x18c`, 1 = LOW LEFT, 0 = CENTER) |
| `0x1c0` x 0x08 | `0x04ae` | settings |
| `0x1e0` x 0x02 | `0x08b0` | settings |
| `0x1ff` | `0x0737` | checksum over `0x000..0x03f` |

`m.nvram.cells` is the 512-byte array; `m.nvram.parameters()` lists the 32 system parameters with their manual
names. The ROM contains factory values only for the system parameters; the speed block is programmed at the factory or
from the panel. The seed files the emulator ships and how they are used: [rom-and-nvram.md](rom-and-nvram.md).

## Serial line

An 8251 USART on ports `0x18` (data) and `0x19` (status/command), receive on IRQ1, transmit on IRQ2.

- **Byte timing.** A byte is 10 bits (start, 8 data, stop). The emulator delivers one byte per byte time:
  2083 time units at 9600 baud (1.04 ms), 1042 at 19200, 521 at 38400. Bytes arrive at that rate whether the
  firmware has read the previous one or not. A byte that arrives while the previous is still unread overwrites it,
  sets the overrun flag (status bit 4) and is counted in `m.uart_overruns` - as on the real 8251.
- **Host side.** The host (Session, runner, bridge) never keeps more than 16 bytes queued "on the wire"
  (`m.uart.rx`), like a host UART with a small FIFO that honours flow control.
- **DTR flow control** (`handshake="hard"`, the default). DTR is bit 1 of the 8251 command byte. While the firmware
  holds DTR low, no byte is delivered. On the machine this is the DTR/DSR wiring of the RS-232C cable with the
  handshake set to `HARD`.
- **XON/XOFF** (`handshake="code"`). The session sets the firmware's handshake setting to `CODE` and stops honouring
  DTR; the firmware then sends XOFF (0x13) / XON (0x11) on the line, which a host has to obey itself. `run_job` does
  not interpret XON/XOFF - there, the only protection is the 16-byte window. Through the bridge the XON/XOFF bytes
  reach the host program, which must have software flow control enabled on the port.
- **Transmit.** A byte written to port `0x18` sits in the holding register until the shifter is free and TxEN
  (command bit 0) is set; then it appears in `m.uart.tx` one byte time later. Replies to query commands, and the
  1.50MAX G-code status answers, arrive there.
- **19200 baud** loses bytes on the machine and in the model alike: the receive ISR cannot always fetch a byte before
  the next one arrives while the step engine's tick handler runs. Use 9600.

## What is not modelled

- **The servo loop.** No following error, no motor current. The machine's `Err 40/41/42` (following error >= 40/50
  pulses) and `Err 43/44/45` (average current above 1.9 A over 4 s) can never occur. A feed profile the machine would
  abort runs through. The axis model records the largest per-tick command (`Axis.max_command`) and flags commands
  above the firmware's own maximum of 40 pulses per tick (`SubCpu.over_cap()`), which is the only warning it can
  give.
- **How a motion record ends on the machine.** The emulator ends a record when the commanded pulses have been paid
  out. For the factory ramp fault (below) the real machine evidently ends the record differently; the model does not
  know how.
- **The factory ramp fault** (`8000:366a`): axis-parallel moves of 16.4 to 25.5 mm above 40 mm/s after a predecessor
  of about 2 mm or more get a ramp-up of 4266 ticks and a huge plateau. Harmless on the machine (measured), but in the
  emulator the record never ends.
- **Spindle**, its on/off relay and load; **AUTO VIEW** hold (the example NVRAM has AUTO VIEW off);
  **Z surface probe / flatness sensor** - the encoder on ports `0x30/0x34` reads 0 by default, as measured on a machine
  without the sensor (`m.counter.source` takes `"strobes"` for the old model or a callable returning raw encoder counts
  for a surface model); the firmware's FLATNESS ON/AUTO paths (`8000:4a06` -> `84b55`, `4f29`, `4f7f`) have not been
  run against a surface model; a path waiting for probe contact (port `0x06` bit 7) will not complete; **mechanics**:
  inertia, backlash, cutting forces, lost motion.
- **Gate-array behaviour** below the sub-CPU window: readiness, error signalling, timing.
- **Absolute Z timing**: see [timing.md](timing.md).
