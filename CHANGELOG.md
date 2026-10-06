# Changelog

## Unreleased

- Z sensor (port `0x06` bit 6) after homing follows a profile measured on a real machine (set 0-2.1 mm and 3.9-6.1 mm
  below the top position, clear elsewhere); the homing run still uses the fitted dog band. Golden reference unchanged.
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
