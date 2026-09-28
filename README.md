# me500emu - Mimaki ME-500 controller emulator

An instruction-level emulator of the controller of the **Mimaki ME-500** CNC engraving machine: two NEC V33 CPUs,
one 512 KiB EPROM (27C4096), HP-GL/MGL commands over RS-232. It runs the **unmodified machine firmware** on
[Unicorn](https://www.unicorn-engine.org/) and models the hardware around it well enough that the firmware boots,
references its axes, accepts jobs over the serial line and moves the (modelled) axes the way it moves the real ones.

It came out of a reverse-engineering project on this machine and is now continued as a project of its own. Its
main practical use is as a **headless test bench for CAM output**: stream a real HP-GL or G-code job into the
emulated machine and get back the machine time, the travelled bounds, the final position and the firmware's own
status - without the machine, without a workpiece, and many times faster than real time.

> **No firmware is included.** The ROM image is copyrighted by Mimaki. You need your own image, read from your own
> machine's EPROM - see [roms/README.md](roms/README.md).

## What works

- **Boots the stock firmware 1.50** from reset: self-tests, NVRAM checksum, sub-CPU handshake, the full homing
  (reference) run, and the normal operating screen on the LCD.
- **Devices**, modelled from the firmware's own use of them:
  - 16x4 HD44780-style LCD and the 3x8 key matrix of the operator panel (ports `0x04/0x07/0x08/0x09`)
  - 8251 UART (ports `0x18/0x19`) with a fixed byte rate, DTR flow control or XON/XOFF, receive overrun and a
    transmit holding register
  - 8259 interrupt controller (ports `0x10/0x11`), fully nested, no auto-EOI, as the firmware programs it
  - 8253 timer: motion tick, kernel timer and service tick at the rates measured on the real machine
  - NVRAM (512 cells) with write protection, seeded from files that hold real machine settings
  - the V33 page registers and the 1 MiB paged store behind the four memory windows
  - the **sub-CPU window and an axis model**: X/Y/Z positions in encoder pulses, travel limits, end switches,
    the Z origin dog
- **Headless job runner** (`me500emu run`): JSON report with machine time, bounds, final position, firmware status
  and LCD; optional CSV trace of the travelled path.
- **pty bridge** (`me500emu bridge`): the machine appears as a serial device (`/dev/pts/N`), so an unmodified
  sender, terminal or streaming tool can talk to it; optional real-time pacing and scripted panel actions.
- **Browser UI** (`me500emu ui`): LCD, clickable panel, 3D view of the travelled path, a job loader that draws the
  job's own geometry as a reference and measures the deviation from it.
- **Boot snapshots**: the ~95 million-instruction cold boot is done once and cached; later starts restore it in about
  a second.
- **Optional 1.50MAX images** (an extended firmware built on 1.50 with look-ahead path planning and a G-code mode)
  are recognised and supported by the runner, the bridge and the UI.

## Status and accuracy

The emulated time base is calibrated against measurements on a real ME-500: 2000 emulator time units = 1 ms,
plus per-instruction-class costs fitted to seven instruction-mix measurements on the machine. Compared with a
workshop run on the real machine, XY motion times agree within the measurement accuracy (block jobs at 9600 baud
within about 1 %); Z moves come out somewhat shorter in the emulator. The most robust use is **comparing two
toolpaths** on the same emulator. Details and numbers: [docs/timing.md](docs/timing.md).

What the emulator does **not** model is listed in [Limitations](#limitations) below - most importantly the servo
loop: a feed profile the real machine would abort with a following-error or overcurrent error runs through here.

## Requirements

- Linux x86-64. Tested on Ubuntu 24.04 with Python 3.12. Other POSIX systems may work; they are untested.
- Python >= 3.8, `gcc` and `make` (the C part of the core is compiled on first use).
- Python packages, pinned because the timing calibration was done with exactly these versions:
  `unicorn==2.1.4`, `capstone==5.0.9` (plus `pytest` for the tests). `make` installs them into a local venv.
- Disk: boot snapshots take a few hundred KB to a few MB each; they live in `~/.cache/me500emu` (see
  [docs/rom-and-nvram.md](docs/rom-and-nvram.md)).
- **A ROM image** of the ME-500 firmware 1.50, read from your own machine. Expected checksums of an unmodified
  image:

      MD5     bc21fcc4f50d8d79456f19ddaf7b7680
      SHA256  5baaac69fbc4feedaba5e24ee12b52e1246d3ad6f3834b79f21c093ddb3ecd18

## Quick start

```bash
git clone <this repository> mimaki_me500_emulator
cd mimaki_me500_emulator
make                                   # venv, C core, ROM check (prints what it found)

cp /path/to/your/dump.bin roms/mimaki-me500-c-0mv1.50.bin
.venv/bin/me500emu info                # should report: stock ROM ... (1.50)

make boot                              # cold boot once (~1-2 min), cached afterwards
.venv/bin/me500emu run examples/square.hpgl
```

`run` prints a JSON report like this (abridged, stock firmware, 9600 baud):

```json
{
 "rom": "1.50", "mode": "std", "baud": 9600, "handshake": "hard",
 "bytes_sent": 77, "bytes_total": 77,
 "machine_time_s": 3.88, "wall_time_s": 2.5, "timed_out": false,
 "bounds_mm": {"x": [0.0, 40.0017], "y": [0.0, 39.9983], "z": [-7.0, 0.0]},
 "cut_bounds_mm": {"x": [19.8274, 40.0017], "y": [19.8307, 39.9983], "z": [-7.0, -0.0123]},
 "final_mm": {"x": 0.0, "y": 0.0, "z": 0.0},
 "work_origin_table_mm": {"x": 0.0, "y": 0.0, "z": 0.0},
 "status": {"strobes": 3760, "halted": 0, "uart_overruns": 0},
 "lcd": ["[REMOTE]     0KB", "SPIN-ON  COM-IIc", "ZD:7.00 Z-ES:10", "ZC:10.00XYES:50"]
}
```

(The 40.0017 instead of 40.0 is the axis-scale correction MECA CORRECT of the machine whose NVRAM the default
configuration is; see [docs/hardware-model.md](docs/hardware-model.md).)

Instead of the venv's `me500emu` you can use `python -m me500emu` with `PYTHONPATH=src`, or `pip install -e .`
into an environment of your choice.

## Command line

`me500emu <command> [options]` (or `python -m me500emu <command>`). Without a command it prints a short help and
exits with 2.

| Command | What it does |
|---|---|
| `info` | version, cache directory, which ROM images were found and what they are |
| `boot` | cold-boot a ROM once and cache the snapshot |
| `run JOB` | run a job file headless and report the result |
| `bridge` | the machine as a serial device on a pty |
| `ui` | browser UI on `http://127.0.0.1:8000` |

**`me500emu boot`**

| Option | Default | |
|---|---|---|
| `--rom R` | `stock` | `stock`, `max` or a file path |
| `--nvram FILE` | `data/nvram/device_example.bin` | NVRAM seed (512 bytes) |
| `--rebuild` | off | boot again even if a snapshot exists |

**`me500emu run JOB`**

| Option | Default | |
|---|---|---|
| `--rom R` | `stock` | `stock`, `max` or a file path |
| `--mode M` | see below | `std`, `opt` or `gcode`; `opt`/`gcode` need a 1.50MAX image |
| `--baud B` | `9600` | `9600`, `19200` or `38400` |
| `--handshake H` | `hard` | `hard` (DTR) or `code` (XON/XOFF) |
| `--nvram FILE` | `device_example.bin` | NVRAM seed |
| `--z0 MM` | G-code: 10, else unchanged | Z0 (work surface) in mm below the top of the Z travel |
| `--timeout S` | `3600` | emulated seconds before giving up |
| `--report FILE` | stdout | write the JSON report here |
| `--trace FILE` | none | write the travelled path as CSV (`t_s,x_mm,y_mm,z_mm`) |

Default mode: `std` on the stock image; on a 1.50MAX image `gcode` for `.nc`, `.gcode`, `.ngc`, `.tap` and `.g`
files and `opt` for everything else. Exit code 0 = ran to completion, 1 = timeout, firmware halt or (1.50MAX) a
G-code stop or parser error, 2 = usage error or ROM not found. Full description of the report:
[docs/cam-testing.md](docs/cam-testing.md).

**`me500emu bridge`** - `--rom`, `--mode` (default `gcode` on a 1.50MAX image, else `std`), `--baud`,
`--handshake`, `--debug`, `--nvram`, `--duration S` (wall-clock seconds, default 900), `--path-file FILE`,
`--realtime`, `--panel FILE`. See [docs/bridge.md](docs/bridge.md).

**`me500emu ui`** - `--port` (8000), `--rom` (default: a 1.50MAX image if one is found, else stock), `--nvram FILE`
(default `~/.cache/me500emu/ui-nvram.bin`, `''` disables persistence), `--no-snapshot`, `--no-physics`. See
[docs/ui.md](docs/ui.md). (`make ui` passes `--rom stock` unless you set `ROM=max`.)

`make` targets: `make` (venv + core + info), `make boot`, `make test`, `make test-all`, `make ui`, `make bridge`,
`make clean`, `make distclean`; `ROM=max` and `PORT=...` select the image and the UI port.

## Python API

```python
from me500emu.session import Session
from me500emu.jobs import run_job

s = Session(rom="stock", mode="std")          # restored from the snapshot cache, REMOTE, at the origin
report = run_job(s, open("examples/square.hpgl").read(), timeout_s=600)
print(report["machine_time_s"], report["bounds_mm"], report["final_mm"])
print(s.lcd())
```

`Session` gives you a machine that is booted, in REMOTE, at the origin and in the requested command mode;
`run_job` streams a job through the emulated UART with flow control and runs until the machine is idle. Below that
there is the low-level `Machine` with direct access to memory, devices and axes. Reference:
[docs/api.md](docs/api.md).

Only **one machine per process**: the C core is a process-wide singleton. Run jobs in parallel with separate
processes.

## Using it for CAM testing

Typical checks for a post-processor or a CAM setting:

- does the job run to completion (exit code, `status`), and does the firmware accept every command?
- does it stay inside the table and the Z range (`bounds_mm`)?
- does it end where it should (`final_mm`)?
- how long does it take on the machine (`machine_time_s`), and which of two variants is faster?

```bash
me500emu run a.plt --report a.json
me500emu run b.plt --report b.json
python -c "import json;a,b=(json.load(open(f)) for f in ('a.json','b.json'));print(a['machine_time_s'],b['machine_time_s'])"
```

With a 1.50MAX image the same works for G-code (`--rom max`); arcs sent as `G2/G3` run much faster than the same
arcs as chords. Workflow, coordinate conventions, pitfalls and a pytest example: [docs/cam-testing.md](docs/cam-testing.md).

## Project layout

```
Makefile, pyproject.toml, requirements*.txt
roms/                   your ROM images go here (not in git) - README.md explains how
examples/               small jobs: square.hpgl (stock), circle.nc (1.50MAX G-code)
docs/                   documentation (index below)
tests/                  pytest suite; parts that need a ROM are skipped without one
src/me500emu/
  cli.py                command line
  session.py            Session: booted, REMOTE, homed, in a command mode
  jobs.py               run_job and the path recorder (the CAM test bench)
  bridge.py             pty bridge
  ui/server.py          browser UI (standard library HTTP server)
  ui/static/            three.js r160 + OrbitControls (MIT, with licence)
  machine.py            Machine: memory map, ports, device wiring, run loop
  fastcore.c / .py      the C core: slice loop, 8259, timers, interrupt dispatch, motion window, axes
  fastdev.py            Python views onto the C device state
  cpu.py                Unicorn x86-16 plus the V33 BRKXA/RETXA instructions
  bus.py                physical memory map
  devices.py            NVRAM, panel/LCD, UART, PIC, page registers, paged store, end switches
  peer.py               the sub-CPU and the axis model
  costs.py              per-instruction-class cost table (timing calibration)
  snapshot_state.py     boot snapshots and their cache keys
  harness.py            helpers for measurements (drain, REMOTE, instruction counters, write watches)
  program.py            reference geometry of HP-GL / G-code jobs (used by the UI)
  paths.py, rominfo.py  ROM lookup, cache directory, image identification
  service.py            fallback for kernel service interrupts the firmware has not installed
  regions.py, symbols.py  address -> functional region / Ghidra symbol name (for diagnostics)
  data/nvram/           NVRAM seed files
  data/symbols.txt      symbol names for bus-fault messages
```

## Documentation

| | |
|---|---|
| [docs/architecture.md](docs/architecture.md) | components, run loop, memory and I/O map, interrupts, snapshots, cost model |
| [docs/hardware-model.md](docs/hardware-model.md) | the machine as modelled: CPUs, axes, switches, panel key map, LCD, NVRAM, serial line |
| [docs/timing.md](docs/timing.md) | time scale, calibration, validation against the real machine, what to trust |
| [docs/cam-testing.md](docs/cam-testing.md) | the job runner for CAM work: report fields, conventions, pitfalls, CI |
| [docs/bridge.md](docs/bridge.md) | the pty bridge and its panel control file |
| [docs/ui.md](docs/ui.md) | the browser UI |
| [docs/api.md](docs/api.md) | Python API: Session, run_job, Machine, snapshots, harness helpers |
| [docs/rom-and-nvram.md](docs/rom-and-nvram.md) | ROM lookup, NVRAM seed files, cache directory |
| [roms/README.md](roms/README.md) | how to obtain and place the ROM image |
| [CONTRIBUTING.md](CONTRIBUTING.md), [CHANGELOG.md](CHANGELOG.md) | |

## Limitations

The short list; details in [docs/hardware-model.md](docs/hardware-model.md) and [docs/timing.md](docs/timing.md).

- **No servo loop.** The machine's axes are DC servo motors with encoders, closed-loop controlled by the second
  V33. That CPU's firmware is not part of the main EPROM and has not been dumped (so far), so the sub-CPU is not
  emulated: its behaviour, as seen by the main CPU, is simulated, and the axes are an ideal encoder-count model.
  Following-error and overcurrent aborts (`Err 40..45`) never happen here; a profile the real machine would
  abort runs through.
- **Factory "ramp fault".** In the stock firmware, axis-parallel moves of 16.4 to 25.5 mm above 40 mm/s following a
  predecessor of about 2 mm or more get a huge plateau (`8000:366a`). Harmless on the machine; in the emulator such a
  record never ends (a limit of the executor model). The UI detects and flags it; the headless runner then ends
  with a timeout.
- **Z timing.** Z records come out roughly 0.3-0.5 s shorter per Z record than measured on the machine (open). The
  Z axis is not a reliable witness near its end stops.
- **Not modelled:** spindle (and its load), AUTO VIEW hold (the example NVRAM has AUTO VIEW off), the Z surface
  probe / flatness sensor, mechanics (inertia, backlash, cutting forces).
- **19200 baud** loses bytes behind the motion tick - on the machine and in the model alike. Use 9600.
- **One machine per process.**
- **CPU instruction set.** Unicorn executes x86; the V33 is an 80186-class CPU. Code that uses instructions the V33
  does not have (for example 386 opcodes emitted by a modern assembler in a patched image) runs here and fails on
  the machine. The emulator cannot catch that.

## Legal

This is an independent hobby project. It is not affiliated with, authorised or endorsed by Mimaki Engineering Co.,
Ltd. "Mimaki" and "ME-500" are trademarks of their respective owners and are used here only to identify the
hardware the emulator models.

No firmware or other Mimaki code is included in this repository. You must supply a ROM image you are entitled to
use. `.gitignore` keeps images out of the repository; please do not publish them.

## License

MIT (see [LICENSE](LICENSE)).

Third-party components:

- [Unicorn Engine](https://www.unicorn-engine.org/) - GPLv2. Used as a library, installed from PyPI; not
  redistributed with this project. The C core does not link against it; it calls the entry points of the
  library the Python binding has already loaded.
- [Capstone](https://www.capstone-engine.org/) - BSD. Installed from PyPI, used to classify instructions for the
  cost table.
- [three.js](https://threejs.org/) r160 and its OrbitControls - MIT. Bundled in `src/me500emu/ui/static/` with
  its licence (`LICENSE.three`).
