# ROM images, NVRAM seeds and the cache

## ROM images

The firmware is not part of this project. How to read it from the machine and where to put it:
[roms/README.md](../roms/README.md).

**Lookup** (`paths.rom_path`), first match wins:

1. an explicit path (`--rom FILE`, or a path passed to `Session`/`rom_path`);
2. the environment: `ME500_ROM` for the stock image, `ME500_ROM_MAX` for a 1.50MAX image;
3. `roms/` of the source checkout: `mimaki-me500-c-0mv1.50.bin` (stock) and `me500_max.bin` (1.50MAX);
4. `roms/` below the current working directory, same names.

`--rom stock` and `--rom max` select by role; anything else is taken as a path. `me500emu info` shows what was found.

**Identification** (`rominfo.version`): a 1.50MAX image carries a version block at `0xFF000` (file offset `0x7F000`),
`$VER$ 1.50MAX BUILD nnnnnn <time>$`; the stock image is recognised by its MD5
(`bc21fcc4f50d8d79456f19ddaf7b7680`). Anything else is reported as "unrecognised image". An unrecognised image still
runs, but the modes `opt`/`gcode` are refused and there is no guarantee the device models match it. If your dump does
not match, check the byte order first.

1.50MAX status cells used by the runner (`session.MAX_CELLS`) are those of build 000019; other builds may differ.

The ROM must be exactly 512 KiB. It is loaded at physical `0x80000`; the snapshot records the image and refuses to
restore onto a different one.

## NVRAM seed files

The machine keeps its configuration - system parameters, speeds, accelerations, command set, origin - in a 512-cell
NVRAM. The emulator builds its NVRAM in three layers: the system parameters from the ROM's factory table, then a
**seed file**, then (optionally) a **persistent file** that the owner of the machine writes back (the UI does).

Two seed files ship in `src/me500emu/data/nvram/`:

| File | What it is | Used by default by |
|---|---|---|
| `iic10_default.bin` | factory contents with COMMAND = MGL-IIc, 10 um step (the usual setting). ORIGIN = CENTER. Speed block seeded by the emulator (the ROM holds no defaults for it): all four speeds 20 mm/s. MECA CORRECT 6000/6000 (exact 2000 pulses/mm). | `Machine`, `snapshot_state.booted()`, `harness.ready()`, the UI when it has no NVRAM file yet |
| `device_example.bin` | the NVRAM of a real, calibrated ME-500: XY-ES 50, Z-ES 10, XY-MS 80, Z-MS 30 mm/s and its accelerations, MECA CORRECT 5998/5999, ORIGIN LOW LEFT, COMMAND MGL-IIc 10 um, AUTO VIEW off | `Session`, `me500emu run`, `boot`, `bridge` |

Why the example machine is the default for jobs: its speeds and accelerations are the ones the timing validation was
done with, and LOW LEFT puts HP-GL/G-code (0,0) at the table corner, which is what most CAM setups expect.

To use your own machine's settings, either read the NVRAM part of your machine, or set the same values through the
emulated panel in the UI (CONDITION, MODE SET, ...) and copy the UI's NVRAM file afterwards; then pass the file with `--nvram FILE` or
`Session(nvram=...)`. The file must be exactly 512 bytes, one byte per cell. Cells `0x000..0x03f` must match the
checksum in cell `0x1ff` or the firmware reports a configuration error at boot. Cell layout:
[hardware-model.md](hardware-model.md#nvram).

A different NVRAM is a different machine for the snapshot cache: every seed gets its own boot snapshot (see below).

## The cache directory

`ME500EMU_CACHE`, or `~/.cache/me500emu` by default. Everything in it can be regenerated; deleting it only costs
time.

| File | What |
|---|---|
| `booted-<rom md5>-<crc>.snap` | boot snapshot per ROM image and NVRAM/geometry configuration (a few hundred KB each) |
| `session-<key>.snap` | state after a `Session`'s setup (REMOTE, approach, Z0, mode) per ROM, NVRAM, mode, baud, handshake, Z0, origin and debug flag |
| `booted-ui-<rom name>-<crc>.snap` | the UI's boot snapshot per ROM and UI NVRAM file |
| `costs_<hash>.tab` | instruction cost table per ROM image and cost parameters (512 KB) |
| `ui-nvram.bin` | the UI's persistent NVRAM (created on the first UI exit) |
| `libfastcore.so` | the compiled C core, only when the package directory is not writable (installed read-only) |

In a source checkout the C core is built next to its source, `src/me500emu/libfastcore.so`; it is rebuilt when
`fastcore.c` is newer. `make clean` removes it.

Snapshots are checked on load (format version, axis geometry, ROM contents); a stale one is rebuilt automatically.
Use `me500emu boot --rebuild` (or `rebuild=True`) after changing anything that affects the boot, or simply delete the
directory.

## Other environment variables

| Variable | |
|---|---|
| `ME500_ROM`, `ME500_ROM_MAX` | ROM image paths |
| `ME500EMU_CACHE` | cache directory |
| `CC` | C compiler for the core (default `gcc`) |
| `INT23`, `INT24` | kernel timer / service tick period in units at **cold boot** (defaults 20340 / 2035; experiments only) |
| `KOST_EXEC` | executor-range cost surcharge in sixteenths of a unit (default 8; calibration only) |
| `FASTCORE=0` | refused: the pure-Python core has been retired |

## symbols.txt

`src/me500emu/data/symbols.txt` holds about 4200 names for addresses in RAM and ROM (physical address, `F` for a
function or `L` for a label, name), exported from the reverse-engineering project's Ghidra database. The emulator
uses them only for diagnostics: a bus fault or Unicorn error names the nearest symbol, e.g.
`from PC: 82ecf  <name>+0x12`. The names reflect the state of that analysis and are not authoritative; they contain
no firmware code.
