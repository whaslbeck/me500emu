# The browser UI

```bash
me500emu ui                     # or: make ui   (make passes --rom stock; ROM=max for the other image)
# ME-500 emulator UI:  http://127.0.0.1:8000   ROM me500_max.bin
```

Open the printed address. The server uses the Python standard library only (no Flask, no npm); the 3D view uses
three.js r160, bundled in `src/me500emu/ui/static/`, so no network access is needed. The server listens on
`127.0.0.1` only.

The emulator runs in a background thread in chunks of 900 000 time units (0.45 s of machine time). The page polls the
state; commands from the page are queued and applied between chunks, so the machine is never touched from two threads
at once.

## Options

| Option | Default | |
|---|---|---|
| `--port N` | `8000` | HTTP port |
| `--rom R` | a 1.50MAX image if one is found, else `stock` | `stock`, `max` or a file path |
| `--nvram FILE` | `~/.cache/me500emu/ui-nvram.bin` | persistent NVRAM file (512 bytes); `--nvram ''` disables persistence |
| `--no-snapshot` | off | run the real homing run instead of restoring the snapshot (1-2 minutes; the State field shows the progress) |
| `--no-physics` | off | no axis model: positions are the commanded pulse counts, end switches follow a simple poll-count stand-in (diagnostics only) |

## What you see

- **Header**: state (`homing`, `approach`, `ready`, ...), ROM, and the controls:
  - **PATH PLAN** STD / OPT / GCODE (1.50MAX only) - writes the firmware's mode cell like the panel menu does;
  - **Baud** 9600 / 19200 / 38400 - the emulated line rate;
  - **ORIGIN** LOW LEFT / CENTER like the MODE SET menu (the RAM copy changes at once, the NVRAM cell for the next
    boot), or a free origin **X,Y mm** (for programs that go below 0 by the tool radius, e.g. `5,5`);
  - **Z0** *n* mm below the top of the Z stroke, like the Z zero key (applied only at standstill, re-applied at job
    start);
  - **F cap** off / 35 / 30 mm/s - replaces `F` words above the cap while sending a G-code job, to stay clear of the
    factory ramp fault.
- **Tool paths** (3D): the travelled path as a line coloured by depth (blue shallow, red deep), the reference path
  of a loaded program in white (cuts) and dashed grey (rapids), the 483 x 305 mm table outline with a 10/50 mm grid,
  an axis cross at the LOW LEFT origin and a marker at the tool. Left mouse button rotates, wheel zooms, right button
  pans. Buttons: Top, Oblique, Fit; orthographic projection; a Z exaggeration (1-10x) that only stretches the display.
  The line below the view gives the actual mm ranges and the point count. A classic 2D canvas view is still available.
  Z is shown as the program coordinate: negative = below Z0.
- **LCD and function keys**: the four LCD lines exactly as the firmware wrote them into the display RAM, with the
  PAGE and F1-F3 +/- keys beside them as on the real panel.
- **Control panel**: X/Y jog arrows, REMOTE/LOCAL, SPINDLE, END, CANCEL/CE, PAUSE, Z AXIS, two Z jog keys, and the raw
  3 x 8 matrix for everything else. Key map: [hardware-model.md](hardware-model.md#key-matrix).
- **Axes**: position in table mm, travel range, a marker when an end switch is made, clip counters.
- **HP-GL to the UART**: a text area whose contents are sent to the serial port as ASCII - the same path the real
  machine sees - with presets (rectangle, star, arc).
- **Program from file**: see below.
- **Errors**: faults and exceptions from the emulator thread.

As on the machine, a job is only accepted in the REMOTE screen. At startup the UI switches to REMOTE and drives
`IN;SP1;PA0,0;` (to the origin) by itself, and the file loader switches to REMOTE again if needed; if you went to
LOCAL on the panel, press REMOTE before sending from the text area.

## Program from file

Pick an HP-GL or G-code file (format detected or forced). The UI parses the file's **own geometry** into a reference
path (`program.py`): HP-GL `IN PA PR PU PD ZQ !PZ` in MGL-IIc-10 units with the machine's current plotter origin and
PD depth; G-code `G0/G1 X Y Z`, `G90/G91`, `F`, the accepted header codes, and a note where the firmware would stop.
It shows the segment count, cut length and feed, and warns:

- when the program leaves the table (with a hint to use LOW LEFT instead of CENTER, or to move the origin inwards),
- when a program moves above Z0 more than the Z0 setting allows,
- when a G-code feed is less than half the machine's XY-ES (usually a CAM set to inches per minute),
- when segments fall into the factory ramp-fault range (with the segment number).

**Start** streams the file into the UART with DTR handshake at the selected baud rate. During the run the UI measures
the largest and mean **XY deviation** of the travelled path from the reference, the **end point error**, the machine
time, and the bytes still pending. The travelled path is rebuilt from the core's per-strobe positions with the axes'
end-stop clipping and checked against the axes at the end.

The run ends as `done` when no motion happened for about a dozen chunks, `halted (STOP)` when a 1.50MAX G-code job
stops (with the rejected line, the partial line buffer and the counters), or `halted (ramp fault 366a, model limit)`
when a record with the ramp-fault signature is committed - on the machine that record would complete.

**`G2/G3` arcs** are part of the reference, with the semantics of the 1.50MAX G-code mode: modal, `I J` relative to
the start point, start = end is a full circle, `Z` makes a helix, XY plane only. The exact arc is cut into chords of at
most 0.002 mm deviation, so the firmware's own chords (at most 0.01 mm) show up in the measured deviation - expect
about 0.01-0.015 mm maximum deviation on arc jobs (measured: full circle r = 5 mm 0.012 mm, mixed G2/G3/helix job
0.014 mm, end point error below 0.002 mm). Where the firmware refuses an arc (`R` form, `G18/G19`, start and end radius
differing by more than 0.02 mm, radius under 0.01 mm, helix pitch over 65 mm) the reference ends with "stop
expected" at that line, like the firmware. The program card shows the number of arc lines next to the segments.

What the reference does not know: the idle retract and re-plunge of the G-code mode (they show up in Z, not in XY) and
`CI`/`AA` arcs in HP-GL (reported as ignored).

## NVRAM persistence and snapshots

The NVRAM is a non-volatile part, so the UI keeps it in a file: read at startup, written on shutdown and with the
**save** button (the header shows changed / saved / unchanged). A setting made through the panel menus survives a
restart. Without an existing file the UI starts from the Machine default seed (`iic10_default.bin`, factory contents
with ORIGIN CENTER); copy `src/me500emu/data/nvram/device_example.bin` to the NVRAM file path to start from the
example machine instead.

The UI boots from its own snapshot, `booted-ui-<rom file name>-<crc32 of the NVRAM file>.snap` in the cache directory,
so a changed NVRAM or the other ROM image gets its own snapshot. The first start per configuration performs the real
homing run once (1-2 minutes).

## Shutdown

Stop the server with **Ctrl-C** (or `SIGTERM`). The UI stops the HTTP server, asks the emulator thread to leave
Unicorn at the next slice boundary, waits for it, saves the NVRAM and exits with code 0. (Ending the process while
the thread is inside Unicorn with Python callbacks on the stack would crash the interpreter.)

## HTTP interface

For scripting the UI without a browser:

| Request | |
|---|---|
| `GET /` | the page |
| `GET /state` | JSON: phase, ROM, mode, baud, origin, Z0, LCD, axes, time counter, strobes, job status, NVRAM state |
| `GET /ref` | JSON: the reference path of the loaded program |
| `GET /path?from=N&gen=G` | travelled path, binary: header `<III>` (generation, first index, total), then float32 triples x, y, z_axis in mm. A different generation (after clear or decimation) restarts from 0. |
| `POST /key` | `{"col": c, "bit": b}` press and release a matrix key |
| `POST /send` | `{"text": "..."}` send text to the UART |
| `POST /load` | `{"name": "...", "text": "...", "fmt": "hpgl" \| "gcode" \| ""}` load a program |
| `POST /start`, `POST /stop` | start / abort the loaded program |
| `POST /mode` | `{"mode": "STD" \| "OPT" \| "GCODE"}` (1.50MAX) |
| `POST /baud` | `{"baud": 9600}` |
| `POST /origin` | `{"origin": "LOW LEFT" \| "CENTER" \| "x,y"}` |
| `POST /zzero` | `{"mm": 10}` |
| `POST /fcap` | `{"mm_s": 35}` (0 = off) |
| `POST /clearpath`, `POST /savenvram` | |

The job's machine time in the UI is computed from the strobe count (0.959 ms per strobe) and can differ slightly from
the headless runner's `machine_time_s`.
