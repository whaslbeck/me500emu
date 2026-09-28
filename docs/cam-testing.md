# Testing CAM output with the emulator

The emulator runs a job the way the machine would: the bytes go through the serial line with flow control, the
firmware parses them, plans the motion and drives the axes. So it answers the questions a CAM user or post-processor
author has before a job goes on the machine:

- does the firmware accept the whole job, or does it stop or halt somewhere?
- does the path stay on the table and within the Z range, and where does the tool actually cut?
- where does the job end?
- how long does it take, and which of two variants is faster?

It does **not** tell you whether the machine can physically follow the profile (no servo loop, no cutting forces),
and absolute times of Z-heavy jobs are optimistic. See [timing.md](timing.md) and
[hardware-model.md](hardware-model.md#what-is-not-modelled).

## The workflow

```bash
me500emu run part.plt --report part.json --trace part.csv                 # stock firmware, HP-GL/MGL
me500emu run part.nc --rom max --report part.json --trace part.csv        # 1.50MAX image, G-code
```

What happens (`Session` + `run_job`):

1. The machine is restored from the boot snapshot (first time per ROM/NVRAM: a 1-2 minute cold boot).
2. The panel is switched to REMOTE, the approach `IN;SP1;PA0,0;` is sent, the machine settles at the origin. In
   G-code mode Z0 and the XY origin offset are set and the mode is switched. This setup is cached as well
   (`session-*.snap`), so repeated runs with the same settings start in about a second.
3. The job is streamed through the emulated UART at the chosen baud rate, never more than 16 bytes ahead of the
   firmware's flow control.
4. The runner stops when all bytes are delivered, the motion queue is empty and the axes have not moved for 1.2 s
   of machine time - or on a firmware halt, or at the timeout (`--timeout`, emulated seconds, default 3600).

Options: see the [README](../README.md#command-line). The command mode defaults to `std` on the stock image; on a
1.50MAX image to `gcode` for `.nc/.gcode/.ngc/.tap/.g` files and `opt` otherwise.

## The report

`me500emu run` writes a JSON object (to stdout, or to `--report FILE`):

| Field | Meaning |
|---|---|
| `rom` | firmware version: `1.50` or `1.50MAX BUILD nnnnnn <time>` |
| `mode`, `baud`, `handshake` | the settings the job ran with |
| `bytes_sent`, `bytes_total` | bytes the firmware accepted / job size. Less than the total means the run ended before the job was consumed (halt, stop, timeout). |
| `machine_time_s` | emulated time from the first byte to the **last change of position**, in seconds |
| `wall_time_s` | host time the run took |
| `timed_out` | the timeout was reached before the machine became idle |
| `bounds_mm` | `{x: [min, max], y: [...], z: [...]}` over **all** positions the axes passed through, in work coordinates (below). Includes the travel from where the machine stood before the job. `null` if an axis never moved. |
| `cut_bounds_mm` | the same, but only positions with the tool **below Z0** (`z < 0`): where the tool actually cuts. `null` if it never went below Z0. The firmware overlaps XY motion with the end of a Z plunge or retract, so this can extend a fraction of a millimetre beyond the programmed cut (the 20 mm square in `examples/` gives x `[19.83, 40.0]`). |
| `final_mm` | `{x, y, z}`: axis position when the machine was idle again, work coordinates |
| `work_origin_table_mm` | `{x, y, z}`: where the work origin lies in table coordinates (XY origin offset from LOW LEFT; Z0 as depth below the top of the Z stroke) |
| `status.strobes` | motion commands issued to the sub CPU during the job |
| `status.halted` | how often the firmware reached its halt loop (`8000:450f`): a hard stop of the machine |
| `status.uart_overruns` | bytes lost in the 8251 because the firmware did not fetch them in time (should be 0) |
| `status.stop` | 1.50MAX: the G-code job was stopped (unknown word, limit, overflow, pause) |
| `status.error` | 1.50MAX: G-code parser error count |
| `status.overflow` | 1.50MAX: G-code receive buffer overflow |
| `status.line_overflow` | 1.50MAX: line longer than the line buffer |
| `status.dropped` | 1.50MAX: bytes dropped |
| `lcd` | the four LCD lines at the end, e.g. `["[GCODE] STOP", ...]` - the firmware's own error messages show here |
| `machine_output` | everything the machine sent on the serial line during the job (answers to query commands, XON/XOFF), latin-1 |
| `job` | absolute path of the job file (CLI only) |

Positions are taken at every motion command the firmware issues (about one per millisecond of machine time), from the
same pulse counts the axes use, so bounds are exact to the encoder pulse rather than to a sampling interval.

`--trace FILE` writes the path as CSV with the header `t_s,x_mm,y_mm,z_mm`: time since the job started and the
position in work coordinates, one row whenever the position changed.

## Coordinates

The report uses **work coordinates**, the frame a CAM program is written in:

- **X/Y**: mm relative to the firmware's XY origin, i.e. the plotter origin (`ORIGIN` setting LOW LEFT or CENTER)
  plus any origin offset. With ORIGIN LOW LEFT and no offset, X/Y 0 is the lower-left table corner the machine
  homes to. In G-code mode the session sets an origin offset of 5/5 mm by default (room for small negative moves such
  as a tool-radius lead-in); `work_origin_table_mm` says where the origin lies.
- **HP-GL/MGL units** in the usual `MGL-IIc 10` setting are 0.01 mm, so an HP-GL coordinate of `2000` is 20 mm.
- **Z**: mm relative to Z0 with the G-code sign convention - negative is below the surface. In G-code mode Z0 is set to
  10 mm below the top of the Z stroke unless `--z0` says otherwise; in the HP-GL modes the firmware's own Z0 is left
  as it is (the top of the stroke after the boot), and `PD` plunges to the machine's depth setting ZD.
- **Table coordinates** (used by `Session.position_mm()` and in `work_origin_table_mm`) are mm from LOW LEFT for
  X/Y, and depth below the top of the stroke for Z (positive = down). The table is 483 x 305 mm; the axes can travel
  23 mm (X) and 8 mm (Y) beyond the origin, and Z 62 mm.

Positions carry the axis-scale correction of the configured machine (MECA CORRECT, see
[hardware-model.md](hardware-model.md#axes)): with the default NVRAM a 40 mm move reports as 40.0017 mm. Use
tolerances of about 0.01 mm when you compare positions.

## Exit codes

| Code | Meaning |
|---|---|
| 0 | the job ran to completion |
| 1 | timeout, firmware halt, or (1.50MAX) a G-code stop or parser error |
| 2 | usage error, unknown option, ROM not found, or a mode the image does not support |

Exit code 0 does not check `overflow`, `line_overflow`, `dropped` or `uart_overruns` - check those in the report if
they matter to you. The report is written in both cases 0 and 1.

## Firmware: stock 1.50 vs 1.50MAX

**Stock firmware 1.50** (`--rom stock`, mode `std`) speaks HP-GL / MGL-IIc: `IN`, `SP`, `PU`, `PD`, `PA`, `PR` and
the MGL extensions such as `!PZ` (plunge depth, 0.01 mm, negative = below zero). The feed is the machine's XY-ES
setting from the panel (50 mm/s in the example NVRAM, see [rom-and-nvram.md](rom-and-nvram.md)); the job carries
none. The stock firmware plans each segment on its own, so dense runs of short segments are much slower than their
path length suggests - which is precisely what the emulator lets you measure.

**1.50MAX** (an extended firmware built on 1.50; optional, `--rom max`) adds:

- `opt` mode: HP-GL with look-ahead path planning across chained segments;
- `ZQ x,y,z;`: a three-axis linear move in HP-GL;
- `gcode` mode: the serial port reads G-code instead of HP-GL. `G0/G1`, `G2/G3` with `I J` (centre relative to the
  start point, XY plane only, a full circle when start = end, a helix with `Z`), `G90/G91`, `G21`, `F` in **mm/min**
  (0.5 ... 50 mm/s). Common header codes (`G17`, `G40`, `G49`, `G54`..`G59`, `G61`, `G64`, `G80`, `G94`, ...) are
  accepted without effect; `M` words are ignored (`M3/M5` do not switch the spindle). `G20`, `G28`, `G53`, `G92`,
  arcs in `R` form and arcs outside the XY plane **stop the job**.

**Arcs as `G2/G3` are much faster than the same arcs as chords.** The firmware cuts an arc into chords of at most
0.01 mm deviation itself and feeds them to the planner without going through the serial line. On the machine, ten
3 mm circles at F3000 took 12.2 s as `G2` lines and 28.0 s as the same 420 chords sent as `G1` lines; the emulator
gives 7.1 s against 19.1 s of pure motion. Let the post-processor emit arcs with `I J`.

## Comparing two post-processor outputs

```bash
for f in variant_a.nc variant_b.nc; do
    me500emu run "$f" --rom max --report "${f%.nc}.json" || echo "$f: exit $?"
done
python3 - <<'EOF'
import json
a, b = (json.load(open(f)) for f in ("variant_a.json", "variant_b.json"))
for k in ("machine_time_s", "cut_bounds_mm", "final_mm"):
    print("%-16s %s\n%-16s %s" % (k, a[k], "", b[k]))
print("b/a time: %.3f" % (b["machine_time_s"] / a["machine_time_s"]))
EOF
```

Relative comparisons like this (same ROM, same NVRAM, same baud rate) are the most robust use of the emulator.

## Pitfalls

- **Origin.** With ORIGIN CENTER the plotter origin is the table centre (241.5 / 152.5 mm): a program written for
  0 ... 300 mm then leaves the table. The default NVRAM (`device_example.bin`) is LOW LEFT; the factory NVRAM
  (`iic10_default.bin`) is CENTER. Programs that go below X/Y 0 by more than the origin offset run into the end stop
  or into the firmware's limit (G-code: `STOP`).
- **Z range.** In G-code mode Z0 is 10 mm below the top of the stroke by default, so anything above `Z10` runs into
  the top stop. Use `--z0` to move Z0 down if your program retracts higher. The Z axis is not a reliable witness near
  its end stops.
- **F is mm/min.** `F254` is 4.2 mm/s (10 in/min); a CAM set to inches per minute produces very slow jobs.
  HP-GL output of the same geometry carries no feed and runs at the panel's XY-ES.
- **Lines longer than 79 characters** stop a 1.50MAX G-code job - this includes comment lines. Keep comments short.
- **The factory ramp fault.** In the stock firmware's motion executor (which the 1.50MAX modes use as well; the UI
  checks HP-GL and G-code jobs alike) axis-parallel moves of 16.4 to 25.5 mm above 40 mm/s after a predecessor of about 2 mm or more get a huge plateau
  (`8000:366a`). The machine copes; in the emulator the record never ends and the run ends with a timeout. Not every
  move in that range triggers it (the 20 mm square in `examples/` does not). If a job times out with the axes at a
  table edge, suspect this; the browser UI detects it and names the segment. Workaround for testing: a feed of at most
  40 mm/s (XY-ES on the panel, or `F2400` in G-code).
- **Timeouts cost wall time.** `--timeout` is emulated time; at 2-5x real time a 3600 s timeout can take 10-30
  minutes of wall clock. Set it to a small multiple of the expected job time.
- **Baud rate.** Use 9600. At 19200 the machine loses bytes behind the motion tick, in the emulator as on the real
  machine (`status.uart_overruns`; in G-code mode lost characters stop the job).
- **End of job.** The runner stops 1.2 s of machine time after the last motion. The 1.50MAX idle retract (about 3.5 s
  after the last record) is therefore not part of the report - end G-code jobs with an explicit retract.
- **Not modelled:** spindle, AUTO VIEW hold, the Z surface probe. Following-error and overcurrent aborts never
  happen: an aggressive profile that passes here can still fail on the machine.

## In CI

The simplest robust way is the CLI in a subprocess: one emulator per process, and the exit code already says whether
the job ran. A pytest example that skips when no ROM image is available:

```python
# tests/test_cam_output.py
import json
import subprocess
import sys

import pytest

from me500emu import paths


def rom_or_skip(which):
    try:
        return paths.rom_path(which)
    except paths.RomNotFound:
        pytest.skip("no %s ROM image (see roms/README.md)" % which)


def run(job, rom, tmp_path, *args):
    report = tmp_path / "report.json"
    p = subprocess.run([sys.executable, "-m", "me500emu", "run", job, "--rom", rom,
                        "--report", str(report), "--timeout", "300", *args])
    return p.returncode, json.loads(report.read_text())


@pytest.mark.slow
def test_square_stays_on_the_table(tmp_path):
    code, r = run("examples/square.hpgl", rom_or_skip("stock"), tmp_path)
    assert code == 0
    assert r["bytes_sent"] == r["bytes_total"]
    assert r["bounds_mm"]["x"][0] >= 0 and r["bounds_mm"]["x"][1] <= 483
    assert r["final_mm"]["x"] == pytest.approx(0.0, abs=0.01)
    assert r["final_mm"]["y"] == pytest.approx(0.0, abs=0.01)
```

In-process use is also possible (`Session` + `run_job`, see [api.md](api.md)); create the sessions one after another,
never two at the same time. The first run per ROM/NVRAM configuration does the cold boot (1-2 minutes); cache
`~/.cache/me500emu` (or `ME500EMU_CACHE`) between CI runs to avoid it. Never put the ROM image into a public CI
system - supply it from a private location through `ME500_ROM`.
