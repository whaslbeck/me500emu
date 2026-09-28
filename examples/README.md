# Examples

Two small jobs to check an installation and to start from. Both need a ROM image (see [roms/README.md](../roms/README.md)).

| File | Firmware | What it does |
|---|---|---|
| `square.hpgl` | stock 1.50 (or 1.50MAX in `std`/`opt`) | HP-GL/MGL: a 20 mm square from X/Y 20 to 40 mm, pen down at the machine's depth setting |
| `circle.nc` | 1.50MAX, G-code mode | G-code: plunge 0.5 mm, one full `G2` circle of radius 5 mm around X15 Y10, retract, back to X0 Y0 |

## square.hpgl

```
IN;SP1;PA0,0;
PU2000,2000;
PD4000,2000,4000,4000,2000,4000,2000,2000;
PU0,0;
```

Coordinates are MGL-IIc 10 um units (0.01 mm), so `2000` is 20 mm. `PD` plunges to the machine's depth ZD (7 mm below
Z0 in the default NVRAM); the feed is the machine's XY-ES (50 mm/s). No feed or depth is in the file - as usual for
HP-GL.

```bash
me500emu run examples/square.hpgl
```

With the default configuration this reports about 3.9 s of machine time, `bounds_mm` x and y `[0.0, 40.0]` (within a
few micrometres: the example machine's axis-scale correction), z `[-7.0, 0.0]`, and a final position of 0/0/0.

## circle.nc

```
G21 G90 G17
G0 Z2
G0 X10 Y10
G1 Z-0.5 F300
G2 X10 Y10 I5 J0 F1200
G0 Z2
G0 X0 Y0
```

X/Y are relative to the work origin, which the session puts 5 mm/5 mm from the table corner; Z is relative to Z0,
10 mm below the top of the Z stroke. `I J` are the centre relative to the start point; start = end makes a full circle.
Feeds are mm/min (`F1200` = 20 mm/s).

```bash
me500emu run examples/circle.nc --rom max
```

Reports about 3.6 s of machine time, `cut_bounds_mm` x `[10.0, 20.0]`, y `[5.0, 15.0]`, z `[-0.5, ...]`, and a final
position of 0/0/2. Note that comment lines count towards the 79-character line limit of the G-code mode; a longer line
stops the job.

Numbers are from build 000019 of 1.50MAX and the default NVRAM; other builds or settings give other times.
