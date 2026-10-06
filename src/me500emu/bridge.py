"""Pty bridge: the emulated machine as a serial device.

Opens a pseudo terminal, prints the slave path (e.g. /dev/pts/7) and moves bytes between the pty master and the
emulated UART while the firmware runs in REMOTE. Any host program that talks to a serial port - a sender, a CAM
post-processor's streamer, a terminal - can open that path instead of /dev/ttyS0.

    me500emu bridge --rom max --mode gcode [--realtime] [--panel FILE] [--duration 900]
    # in a second window, e.g.:
    picocom -b 9600 /dev/pts/7

A pty has no modem lines: DTR flow control is applied inside the emulator (the machine's receive side stalls), and
the bridge only pulls as many bytes from the pty as a real wire would hold (16). With --realtime the emulator is
pinned to the wall clock (2000 instructions = 1 ms), so timeouts and job durations measured by the host come out as
on the machine; without it the emulator runs as fast as it can (typically 2..5x real time).

--panel FILE is a control file for operator actions: the host writes one command into FILE, the bridge performs the
key sequence and answers in FILE.ok:
    zero X Y Z     LOCAL, <MOVE>, jog to X/Y mm, XY origin key, jog Z down to Z mm, Z zero key, CE, REMOTE
    z_to Z [zero]  LOCAL, <MOVE>, jog Z to Z mm below the top (up or down; stops early where Z no longer moves),
                   optionally the Z zero key there, CE, REMOTE. Answers with the position reached BEFORE CE: leaving
                   <MOVE> with CE drives Z back to the top (firmware behaviour)
    local_remote   LOCAL, then REMOTE again
    lcd            the four LCD lines
"""
import argparse
import functools
import os
import select
import struct
import time

from . import harness as H
from .session import Session, SessionError, INSTR_PER_S

print = functools.partial(print, flush=True)

WIRE = 16                   # bytes "on the wire" between host and UART


def panel_command(s, words):
    m = s.m
    ax = m.subcpu.axes
    if words[0] == "lcd":
        return s.lcd()
    if words[0] == "local_remote":
        s.press(1, 3); s.run(2000000)
        H.remote(m); s.run(2000000)
        return "ok " + s.lcd()
    if words[0] == "zero":
        x, y, z = (float(v) for v in words[1:4])
        s.press(1, 3); s.run(2000000)                              # LOCAL
        s.press(2, 6)                                              # <MOVE>
        s.hold_until(1, 4, lambda: ax["X"].table_mm() >= x)        # X+
        s.hold_until(1, 5, lambda: ax["Y"].table_mm() >= y)        # Y+
        H.drain(m, 4000000)
        s.press(2, 7)                                              # XY key: origin
        s.hold_until(2, 5, lambda: ax["Z"].table_mm() >= z)        # Z down
        H.drain(m, 4000000)
        s.press(2, 4)                                              # Z zero key
        lcd = s.lcd()
        s.press(1, 0)                                              # CE
        H.remote(m); s.run(2000000)
        return "ok position %s, LCD when set %s" % (s.position_mm(), lcd)
    if words[0] == "z_to":
        target = float(words[1])
        z = ax["Z"]
        s.press(1, 3); s.run(2000000)                              # LOCAL
        s.press(2, 6)                                              # <MOVE>
        H.drain(m, 4000000)
        down = target > z.table_mm()
        key = (2, 5) if down else (2, 6)                           # Z down / Z up in <MOVE>
        still, last = 0, z.pos
        for _ in range(900):
            if (z.table_mm() >= target) if down else (z.table_mm() <= target):
                break
            m.panel.press(*key)
            s.run(200000)
            still = still + 1 if z.pos == last else 0
            last = z.pos
            if still >= 10:                                        # 1 s held without motion: the firmware stops here
                break
        m.panel.release_all()
        s.run(H.KEY_GAP)
        H.drain(m, 4000000)
        reached = z.table_mm()
        lcd = s.lcd()
        if len(words) > 2 and words[2] == "zero":
            s.press(2, 4)                                          # Z zero key
            lcd = s.lcd()
        s.press(1, 0)                                              # CE
        H.remote(m); s.run(2000000)
        return "ok Z %.3f mm below the top (before CE), LCD %s" % (reached, lcd)
    return "unknown command"


def main(argv=None):
    ap = argparse.ArgumentParser(prog="me500emu bridge", description=__doc__.split("\n\n")[0])
    ap.add_argument("--rom", default="stock", help="stock | max | FILE (default stock)")
    ap.add_argument("--mode", default=None, choices=["std", "opt", "gcode"],
                    help="command mode (default: gcode for a 1.50MAX image, std otherwise)")
    ap.add_argument("--baud", type=int, default=9600, choices=[9600, 19200, 38400])
    ap.add_argument("--handshake", default="hard", choices=["hard", "code"],
                    help="hard = DTR flow control (default), code = XON/XOFF")
    ap.add_argument("--debug", action="store_true", help="DEBUG = ON (1.50MAX: enables the ~ measurement commands)")
    ap.add_argument("--nvram", default=None, help="NVRAM seed file (default: data/nvram/device_example.bin)")
    ap.add_argument("--duration", type=float, default=900.0, help="wall-clock seconds before the bridge exits")
    ap.add_argument("--path-file", default=None, help="also write the pty slave path into this file")
    ap.add_argument("--realtime", action="store_true", help="pin emulated time to the wall clock")
    ap.add_argument("--panel", default=None, help="control file for operator actions (see above)")
    a = ap.parse_args(argv)

    try:
        from . import paths, rominfo
        rom = paths.rom_path(a.rom)
        mode = a.mode or ("gcode" if rominfo.is_max(rom) else "std")
        print("Booting %s (%s), mode %s ..." % (rom, rominfo.version(rom) or "unknown image", mode))
        s = Session(rom=rom, mode=mode, baud=a.baud, handshake=a.handshake, nvram=a.nvram, debug=a.debug)
    except (SessionError, OSError) as e:
        raise SystemExit(str(e))
    m, u = s.m, s.u

    master, slave = os.openpty()
    path = os.ttyname(slave)
    print("Pty: %s" % path)
    if a.path_file:
        open(a.path_file, "w").write(path + "\n")
    os.set_blocking(master, False)
    print("Machine ready (%s, REMOTE, %s), position %s, LCD %s" % (mode, a.handshake, s.position_mm(), s.lcd()))

    rd = lambda addr: struct.unpack("<H", bytes(u.mem_read(addr, 2)))[0]
    t0 = time.time()
    i0 = m.instr
    n_in = n_out = 0
    watch = dict(key=None, since=m.instr, reported=False)

    def pull():
        nonlocal n_in
        r, _, _ = select.select([master], [], [], 0.0)
        room = WIRE - len(m.uart.rx)
        if r and room > 0:
            data = os.read(master, room)
            if data:
                m.uart.rx.extend(data)
                n_in += len(data)

    while time.time() - t0 < a.duration:
        if a.panel and os.path.exists(a.panel):
            try:
                words = open(a.panel).read().split()
                os.remove(a.panel)
            except OSError:
                words = []
            if words:
                before = m.instr
                answer = panel_command(s, words)
                # an operator action takes as long as the operator needs; it must not count against the realtime pacing
                i0 += m.instr - before
                print("   panel %s -> %s" % (" ".join(words), answer))
                open(a.panel + ".new", "w").write(answer + "\n")
                os.replace(a.panel + ".new", a.panel + ".ok")
        if a.realtime and m.instr - i0 > (time.time() - t0) * INSTR_PER_S:
            time.sleep(0.005)
            try:
                pull()                    # keep serving the line while waiting
            except (BlockingIOError, InterruptedError):
                pass
            except OSError:
                break
            continue
        # watchdog: records queued in the executor ring but no progress for 60 M instructions -> dump state
        key = (bytes(u.mem_read(0x1004, 2)), rd(0x1006))
        if key != watch["key"]:
            watch.update(key=key, since=m.instr, reported=False)
        if not s.ring_empty() and m.instr - watch["since"] > 60000000 and not watch["reported"]:
            watch["reported"] = True
            print("!! watchdog: executor ring %02x/%02x stuck for 60 M instructions, strobes %d, position %s, LCD %s"
                  % (u.mem_read(0x1004, 1)[0], u.mem_read(0x1005, 1)[0], m.motion.strobes, s.position_mm(), s.lcd()))
        try:
            pull()
        except (BlockingIOError, InterruptedError):
            pass
        except OSError:
            break
        # small steps while bytes are pending: 16 bytes at 9600 baud are 33 k instructions
        t_run = time.time()
        step = 20000 if len(m.uart.rx) else 400000
        m.run(m.instr + step)
        if a.realtime and step == 400000 and time.time() - t_run > 0.3:
            print("   (emulator slower than real time: 400 k instructions took %.2f s)" % (time.time() - t_run))
        tx = s.read_tx()
        if tx:
            try:
                os.write(master, tx)
                n_out += len(tx)
            except OSError:
                break
        if s.halted:
            print("!! firmware halt trap (8000:450f) reached")
            break
    print("Bridge done: %d bytes in, %d bytes out, UART overruns %d, position %s, strobes %d, LCD %s"
          % (n_in, n_out, m.uart_overruns, s.position_mm(), s.status()["strobes"], s.lcd()))


if __name__ == "__main__":
    main()
