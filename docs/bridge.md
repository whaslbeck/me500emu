# The pty bridge

`me500emu bridge` makes the emulated machine appear as a serial device. It opens a pseudo terminal, prints the slave
path (for example `/dev/pts/7`) and moves bytes between the pty and the emulated 8251 while the firmware runs in
REMOTE. Any program that talks to a serial port - a terminal, a job sender, a CAM post-processor's streamer, a test
script - can open that path instead of `/dev/ttyUSB0` and does not need to know it is talking to an emulator.

```bash
me500emu bridge --rom max --mode gcode
# Booting .../roms/me500_max.bin (1.50MAX BUILD ...), mode gcode ...
# Pty: /dev/pts/7
# Machine ready (gcode, REMOTE, hard), position (0.0, 0.0, 0.0), LCD [GCODE] IDLE | ...

picocom -b 9600 /dev/pts/7          # in a second terminal
```

The bridge runs until `--duration` wall-clock seconds have passed (default 900), the pty is closed, or the firmware
reaches its halt loop. At the end it prints bytes in/out, UART overruns, the position (table mm), the strobe count and
the LCD.

## Options

| Option | Default | |
|---|---|---|
| `--rom R` | `stock` | `stock`, `max` or a file path |
| `--mode M` | `gcode` on a 1.50MAX image, else `std` | `std`, `opt`, `gcode` (`opt`/`gcode` need 1.50MAX) |
| `--baud B` | `9600` | `9600`, `19200`, `38400` - the emulated line rate |
| `--handshake H` | `hard` | `hard` = DTR flow control, `code` = XON/XOFF |
| `--debug` | off | 1.50MAX only: DEBUG = ON, enables the `~` measurement commands |
| `--nvram FILE` | `data/nvram/device_example.bin` | NVRAM seed |
| `--duration S` | `900` | wall-clock seconds before the bridge exits |
| `--path-file FILE` | none | also write the pty slave path into this file (for scripts) |
| `--realtime` | off | pin emulated time to the wall clock |
| `--panel FILE` | none | control file for operator actions, see below |

The machine is prepared exactly like a `Session` (see [api.md](api.md)): booted from the snapshot cache, REMOTE,
approach move to the origin, and in G-code mode Z0 = 10 mm below the top and an XY origin offset of 5/5 mm.

## Flow control on a pty

A pty has no modem lines, and the baud rate set on it by the host is ignored. The bridge therefore:

- applies **DTR flow control inside the emulator**: while the firmware holds DTR low, the emulated UART receives
  nothing, and
- only pulls as many bytes from the pty as a real wire would hold (16). Everything else stays in the pty buffer, so
  a host that writes a whole file at once is throttled by the kernel, as it would be by a real handshake.

With `--handshake code` the firmware sends XON/XOFF bytes, which reach the host through the pty; enable software flow
control in the host program.

The byte rate is emulated: at 9600 baud the firmware sees one byte every 1.04 ms of emulated time, whatever speed the
host writes at.

## Real-time pacing

Without `--realtime` the emulator runs as fast as it can - typically 2-5x faster than the machine - so a job finishes
sooner than on the machine, and host-side timeouts see a faster machine. With `--realtime` emulated time is pinned to
the wall clock (2000 units = 1 ms): the bridge sleeps whenever it is ahead, so job durations and timeouts measured by
the host come out as on the machine. If the host computer cannot keep up, the bridge prints
`(emulator slower than real time ...)`.

Time spent on panel actions (below) does not count against the pacing; an operator also takes as long as they need.

## Panel control file

Some host workflows need an operator: set the zero points, toggle LOCAL/REMOTE. With `--panel FILE` a host program
can ask the bridge to perform these key sequences on the emulated panel. The host writes **one command** into `FILE`;
the bridge picks it up (and deletes it), performs the keys, and writes its answer to `FILE.ok` (atomically, via
`FILE.new`).

| Command | What the bridge does | Answer |
|---|---|---|
| `lcd` | nothing | the four LCD lines, joined with ` \| ` |
| `local_remote` | REMOTE/LOCAL key (to LOCAL), then back to REMOTE | `ok <LCD>` |
| `zero X Y Z` | LOCAL; open `<MOVE>`; jog X+ until the head is at X mm, then Y+ until Y mm; press the XY origin key; jog Z down until Z mm; press the Z zero key; CE; back to REMOTE | `ok position (x, y, z), LCD when set <LCD>` |
| anything else | nothing | `unknown command` |

The `zero` coordinates are **table coordinates**: X/Y in mm from LOW LEFT, Z in mm below the top of the stroke. The
jogs only move in the positive direction (X+, Y+, Z down) and stop at the first position at or beyond the target, so
the result is within a jog step of the request; the answer reports the actual position.

Example from a shell script:

```bash
me500emu bridge --rom max --panel /tmp/me500panel --path-file /tmp/me500pty --realtime --duration 3600 &
until [ -s /tmp/me500pty ]; do sleep 1; done
PTY=$(cat /tmp/me500pty)

echo "zero 20 20 12" > /tmp/me500panel             # origin at X20 Y20, Z0 12 mm below the top
until [ -e /tmp/me500panel.ok ]; do sleep 0.5; done
cat /tmp/me500panel.ok; rm /tmp/me500panel.ok
```

## A minimal sender

The bridge is a normal serial port, so a sender needs nothing emulator-specific. With pyserial:

```python
import serial, sys

port = serial.Serial(sys.argv[1], 9600, timeout=1, dsrdtr=False, rtscts=False)
with open(sys.argv[2], "rb") as f:
    for line in f:
        port.write(line)          # the bridge throttles through the pty buffer
port.flush()
print(port.read(1000))            # whatever the machine answered
```

Run it as `python send.py /dev/pts/7 examples/circle.nc` with the bridge started in G-code mode on a 1.50MAX image.
(pyserial is not a dependency of this project.)

## Diagnostics

- `!! watchdog: executor ring ... stuck for 60 M instructions` - motion records are queued but nothing progressed for
  30 s of machine time. Usually the factory ramp fault (see [cam-testing.md](cam-testing.md#pitfalls)).
- `!! firmware halt trap (8000:450f) reached` - the firmware stopped hard; the bridge exits. The LCD in the final line
  usually says why.
- UART overruns in the final line mean the firmware lost bytes (at 19200 baud this is expected; use 9600).
