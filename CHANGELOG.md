# Changelog

## Unreleased

- Z sensor (port `0x06` bit 6) follows a profile measured on a real machine (set 0-2.1 mm and 3.9-6.1 mm below the top
  position, clear elsewhere), during the homing run too; the fitted dog band is gone and Z starts at its rest position.
  The firmware's Z reference now runs its measured path (centre of the sensor gap, table 0 = 3 mm above it) and ends
  at the top with bit 6 set. **Intended behaviour change:** golden reference regenerated (instruction count +69 and
  RAM hash; path, strobes, bounds, time and LCD unchanged); snapshots rebuild automatically (axis geometry key).
- Flatness sensor encoder (ports `0x30/0x34`) reads 0 by default, as measured on a machine without the sensor;
  `m.counter.source` selects the old strobe-accumulating model or a surface model. Snapshot format 4. Intended behaviour
  change: golden reference regenerated (instruction count -69 and RAM hash; path, strobes, time and LCD unchanged).
- "Snapshot stale" message goes to stderr (it corrupted the JSON report of `me500emu run`).
- Bridge: panel command `z_to Z [zero]`.
- Docs: port `0x06` and Z travel as measured on a real machine.

## 0.1.0 - initial public release

First release as a project of its own, split out of a private reverse-engineering notebook on the Mimaki ME-500.

- Runs the unmodified ME-500 firmware 1.50 (and optional 1.50MAX images) on Unicorn 2.1.4, with the C core as the only
  core: slice loop, 8259, 8253 ticks, UART pacing, interrupt dispatch, motion window and axis model in C.
- Device models: LCD and key matrix, 8251 UART with byte timing, DTR and XON/XOFF, 8259, 8253, NVRAM, page registers
  and paged store, end switches, sub-CPU handshakes and an encoder-count axis model with the LOW LEFT frame.
- Time base calibrated against the real machine: 2000 units = 1 ms, per-instruction-class costs fitted to seven
  instruction-mix measurements, executor surcharge fitted to a workshop run.
- `me500emu` command line: `info`, `boot`, `run` (JSON report, CSV trace), `bridge` (pty), `ui` (browser UI with 3D
  path view, job loader and deviation measurement; the job reference includes `G2/G3` arcs and helices with the
  1.50MAX semantics).
- Python API: `Session`, `run_job`, `snapshot_state.booted`, `harness`.
- Boot and session snapshots in `~/.cache/me500emu`, keyed on ROM, NVRAM and axis geometry.
- NVRAM seeds: factory contents (MGL-IIc 10 um) and the NVRAM of a real calibrated machine.
- No firmware included; see `roms/README.md`.
