"""A browser UI for the ME-500 emulator.

Run it, open the address it prints, and you get:

  * a 3D view of the travelled path, colour- and height-coded by Z
  * the LCD exactly as the firmware writes it
  * the 24-key panel matrix, clickable
  * a text area whose contents go to the UART as ASCII
  * a file loader for HP-GL and G-code programs: the file's own geometry is drawn
    as the reference, the run is measured against it (deviation, end point, time)

Standard library only - no Flask, no npm. The emulator runs in a background
thread and the page polls it; commands from the page are queued and applied
between run chunks so the machine is never touched from two threads at once.

    me500emu ui [--port 8000] [--rom stock|max|<file>] [--no-physics]
"""
import sys, os, json, threading, queue, argparse, signal, binascii, time, struct, array, urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    from ..machine import Machine, MOTION_TICK_CALIBRATED
    from .. import harness as HARN
    from .. import snapshot_state as snap
    from .. import peer
    from .. import program
    from .. import paths, rominfo
except ModuleNotFoundError as e:
    if "unicorn" not in str(e):
        raise
    sys.stderr.write(
        "\nThe emulator needs Unicorn, and this Python does not have it.\n"
        "Create the project venv once and start the UI from it:\n\n"
        "    make venv\n\n"
        "or install the dependencies into the active environment:\n\n"
        "    pip install -r requirements.txt\n\n")
    raise SystemExit(1)

HERE = os.path.dirname(os.path.abspath(__file__))
MODE_CELL = 0x1E04                          # 1.50MAX: 0xa5 = OPT, 0x5a = GCODE, otherwise STD
# The machine's ORIGIN setting (menu MODE SET: LOW LEFT / CENTER; NVRAM cell 0x18c
# 1 / 0) and its RAM copy [0xb500]/[0xb504] in table steps: CENTER = half the table.
ORIGINS = {"LOW LEFT": (0, 0), "CENTER": (48300, 30500)}
ORIGIN_ALIAS = {"1": "LOW LEFT", "LOWLEFT": "LOW LEFT", "LL": "LOW LEFT", "0": "CENTER"}
ORIGIN_NVRAM = {"LOW LEFT": 1, "CENTER": 0}
NVRAM_ORIGIN_CELL = 0x18C
MODES = {"STD": 0x00, "OPT": 0xA5, "GCODE": 0x5A}
BAUD = {9600: 2083, 19200: 1042, 38400: 521}   # instructions per byte in the core (10 bits per byte)
TICK_MS = 0.959                             # one motion tick, measured
# The NVRAM is a real part and keeps its contents across a power cycle, so the
# emulator keeps it in a file: read at startup, written back on shutdown and on
# demand. Without it every restart threw away whatever was set in the menus.
NVRAM = paths.cache_path("ui-nvram.bin")
# seed for a fresh NVRAM file: the same real-machine NVRAM the runner and the bridge use (ORIGIN LOW LEFT)
UI_NVRAM_SEED = paths.data_path("nvram", "device_example.bin")

CHUNK = 900000          # instructions per step; with trace="fast" this is
                        # well under a second and keeps the page responsive
PATH_CAP = 300000       # points kept before decimating (the 3D view renders them on the GPU;
                        # transported incrementally over /path, so the count is a memory question)
STATIC = os.path.join(HERE, "static")   # three.js r160 (module build), OrbitControls, licence


class Emu(object):
    def __init__(self, physics=True, nvram=NVRAM, snapshot=True, rom=None):
        self.rom = os.path.normpath(rom)
        self.is_max = rominfo.is_max(self.rom)
        self.job = None                 # the loaded program and its run
        self.ref = []                   # reference path in mm: (x, y, z, cutting)
        self.ref_id = 0
        # axis steps at table value 0 (origin LOW LEFT): the axis frame
        # (peer.ORIGIN_MARGIN_MM), taken over when the machine is attached
        self.orig = [int(peer.ORIGIN_MARGIN_MM[k] * spm) for k, spm in (("X", 2000), ("Y", 2000), ("Z", 4000))]
        self.baud = 9600
        self.fcap = 0.0                 # test bench: F cap in mm/s (0 = off)
        self.z_zero_target = None       # requested Z zero (mm below top), re-applied at job start
        self._z_zero_latched = None     # latched Z zero (z_zero_mm)
        self.timing = {"run": 0.0, "harvest": 0.0, "state": 0.0, "chunks": 0, "start": time.time()}
        self.cmds = queue.Queue()
        self.lock = threading.Lock()
        self.physics = physics
        self.nvram = nvram
        self.snapshot = snapshot
        self.path = []
        self.path_gen = self.path_gen + 1 if hasattr(self, "path_gen") else 0   # invalidates the client buffers (/path)
        self.phase = "starting"
        self.err = None
        self.m = None
        self.stop = False
        self.saved = None               # last NVRAM save, for the status line
        self._last = None

    # ---- the emulator thread -----------------------------------------
    def boot(self):
        # trace="fast" drops the per-instruction hook: 2.3x faster on the real
        # workload, at the cost of an instruction count that is off by a few
        # wherever a hook stops the engine early. The UI does not need exact counts.
        rom = self.rom
        if self.snapshot:
            # the power-on reference is deterministic; restoring it costs about
            # a second instead of the ninety the cold boot takes. The cache is
            # keyed on the ROM and the NVRAM file, so a machine with stored
            # settings - or the other image - gets its own cache rather than
            # being reset to the default one (or rebooted every other start).
            key = 0
            if self.nvram and os.path.exists(self.nvram):
                key = binascii.crc32(open(self.nvram, "rb").read()) & 0xFFFFFFFF
            cache = paths.cache_path("booted-ui-%s-%08x.snap"
                                     % (os.path.splitext(os.path.basename(rom))[0], key))
            m = snap.booted(rom, cache=cache, physics=self.physics, trace="fast",
                            strict=True, nvram_path=self.nvram, nvram_seed=UI_NVRAM_SEED,
                            on_create=self._attach)
        else:
            m = Machine(rom, strict=True, physics=self.physics, trace="fast",
                        nvram_path=self.nvram, nvram_seed=UI_NVRAM_SEED)
            self._attach(m)
            if self.physics:
                m.subcpu.sign_window()
        # calibrated time base (a restored snapshot carries it already): INT 23h every 10.17 ms, INT 24h ~1 ms
        m.tick_interval = 20340
        m.service_tick_interval = 2035
        m.motion_tick_interval = MOTION_TICK_CALIBRATED
        self.m = m
        # Record watchdog at the commit 8000:2ecf: the signature of the ramp fault (ramp-up
        # 0x10aa = 4266, plateau > 30000 ticks) - the test bench's executor never lets such a
        # record end (harmless on the machine; the executor model is not the machine's).
        # Report the job as halted.
        import unicorn as _uc
        from unicorn.x86_const import UC_X86_REG_BP as _BP

        def commit(u_, a_, s_, d_):
            j = self.job
            if j is None or j["status"] != "running":
                return
            bp = u_.reg_read(_BP)
            cx = int.from_bytes(bytes(u_.mem_read(bp + 2, 2)), "little")
            plateau = int.from_bytes(bytes(u_.mem_read(bp + 4, 2)), "little")
            if cx >= 4000 or plateau >= 30000:
                dx = int.from_bytes(bytes(u_.mem_read(bp + 0x0C, 4)), "little", signed=True)
                dy = int.from_bytes(bytes(u_.mem_read(bp + 0x10, 4)), "little", signed=True)
                j["fault_record"] = "record dx %.1f dy %.1f mm, ramp-up %d ticks, plateau %d" % (dx / 2000.0, dy / 2000.0, cx, plateau)
        m.cpu.uc.hook_add(_uc.UC_HOOK_CODE, commit, None, 0x82ECF, 0x82ECF)
        # The core's path buffer carries the COMMANDED position (the accumulator of the
        # motion window); the axes are absolute and CLIP at the end stop - `IN;` drives
        # against the switch and commands far more than the axis can travel. The
        # physical path is therefore rebuilt from the strobe differences with the same
        # clipping (start = axis position at boot); at job end it is checked against the axes.
        self.pp = None
        self.pp_last = None
        if self.physics and getattr(m, "fc", None) is not None:
            # After a cold boot the core's path buffer still holds the strobes of the
            # homing run (from the snapshot it comes empty): discard them, otherwise
            # `_harvest` replays them from the rest position and shifts the path by the frame
            # (smoke test: 9.1 mm deviation only after a cold boot).
            m.fc.motion.path_n = 0
            a, q = m.subcpu.axes, m.motion.pos
            self.pp = [a["X"].pos, a["Y"].pos, a["Z"].pos]
            self.pp_lim = [a["X"].travel_steps, a["Y"].travel_steps, a["Z"].travel_steps]
            self.pp_last = [q["X"], q["Y"], q["Z"]]
            self.orig = [a["X"].origin_steps, a["Y"].origin_steps, a["Z"].origin_steps]
            # Pulses per table step and per mm from the axes: since MECA CORRECT is in the
            # axis model (peer.py) these are 10*6000/corr and 2000*6000/corr - reference path
            # and mm display must carry the same scale as the firmware, otherwise the
            # correction shows up as "deviation" in the result (33 um mean on the plt case).
            self.imp = [a["X"].STEPS_PER_MM / 200.0, a["Y"].STEPS_PER_MM / 200.0, a["Z"].STEPS_PER_MM / 200.0]
            self.spm = [a["X"].STEPS_PER_MM, a["Y"].STEPS_PER_MM, a["Z"].STEPS_PER_MM]
        return m

    def _attach(self, m):
        """Sample the path on every strobe, and make the machine interruptible.

        With the C core (fastcore) the core itself tracks the position after every strobe
        (`motion.path`); a hooked `write` would force the motion window back into
        Python. In that case the path is harvested between run chunks (`_harvest`)."""
        self.m = m
        if getattr(m, "fc", None) is not None:
            return
        mr = m.motion
        orig = mr.write

        def w(off, size, value, pc):
            r = orig(off, size, value, pc)
            if off == mr.STROBE and (value & 0xFF) == 1:
                self._sample()
            return r
        mr.write = w

    def _sample(self):
        m = self.m
        if self.physics:
            a = m.subcpu.axes
            p = (round(a["X"].table_mm(), 3), round(a["Y"].table_mm(), 3), round(a["Z"].table_mm(), 3))
        else:
            s = m.motion.pos
            p = self._mm_from((s["X"], s["Y"], s["Z"]))
        self._add(p)

    def _add(self, p):
        if p == self._last:
            return
        self._last = p
        self.path.append(p)
        if len(self.path) > PATH_CAP:
            self.path = self.path[::2]          # decimate, keep the shape
            self.path_gen += 1
        j = self.job
        if j is not None and j["status"] == "running" and len(self.ref) > 1:
            self._measure(p)

    def _harvest(self):
        """Fetch the positions from the C core's path buffer and empty the buffer."""
        m = self.m
        fc = getattr(m, "fc", None)
        if fc is None:
            return
        n = fc.motion.path_n
        if not n:
            return
        arr = fc.path
        pts = [(arr[3 * i], arr[3 * i + 1], arr[3 * i + 2]) for i in range(n)]
        fc.motion.path_n = 0
        if self.pp is None:
            for q in pts:
                self._add(self._mm_from(q))
            return
        pp, last, lim = self.pp, self.pp_last, self.pp_lim
        for q in pts:
            for i in range(3):
                v = pp[i] + (q[i] - last[i])
                pp[i] = 0 if v < 0 else (lim[i] if v > lim[i] else v)
                last[i] = q[i]
            self._add(self._mm_from(pp))

    # ---- program from file -------------------------------------------
    def _pulses(self, t):
        """Table steps (tX, tY, tZ) -> axis steps: origin LOW LEFT of the axes
        (peer.ORIGIN_MARGIN_MM: 23 / 8 mm from the lower end stop) + (10, 10, 20) * table value
        (firmware analysis: PA0,0 at CENTER = +483000/+305000 pulses,
        at LOW LEFT no pulse)."""
        o = self.orig
        i = getattr(self, "imp", [10.0, 10.0, 20.0])
        return [o[0] + int(round(i[0] * t[0])), o[1] + int(round(i[1] * t[1])), o[2] + int(round(i[2] * t[2]))]

    def _mm_from(self, p):
        """Axis steps -> table coordinate in mm (0 = origin LOW LEFT; Z positive downwards
        from the upper end stop)."""
        o = self.orig
        m = getattr(self, "spm", [2000.0, 2000.0, 4000.0])
        return (round((p[0] - o[0]) / m[0], 3), round((p[1] - o[1]) / m[1], 3), round((p[2] - o[2]) / m[2], 3))

    def _target_path(self, points, start_table, phys, lim):
        """Physical reference path (pulses) from the table points: the firmware commands in its
        table, the axes sit offset by `off` and clip at 0 and `lim`. A segment that drives an
        axis into the end stop bends there (break point); after that the offset is new (the
        firmware keeps believing in its target). `IN;` (cutting = None) sets the table to 0
        without motion."""
        phys = list(phys)
        off = [phys[i] - self._pulses(start_table)[i] for i in range(3)]
        out = [tuple(phys) + (False,)]
        cmd_a = self._pulses(start_table)
        for pt in points[1:]:
            if pt[3] is None:
                cmd_a = self._pulses((0, 0, 0))
                off = [phys[i] - cmd_a[i] for i in range(3)]
                continue
            cmd_b = self._pulses(pt)
            ts = {1.0}
            for i in range(3):
                d = cmd_b[i] - cmd_a[i]
                if d:
                    for bound in (0, lim[i]):
                        t = (bound - off[i] - cmd_a[i]) / float(d)
                        if 0.0 < t < 1.0:
                            ts.add(round(t, 9))
            for t in sorted(ts):
                for i in range(3):
                    v = cmd_a[i] + t * (cmd_b[i] - cmd_a[i]) + off[i]
                    phys[i] = 0 if v < 0 else (lim[i] if v > lim[i] else v)
                out.append(tuple(phys) + (bool(pt[3]),))
            off = [phys[i] - cmd_b[i] for i in range(3)]
            cmd_a = cmd_b
        return out

    def _reference(self):
        """The reference path from the loaded text, with the machine's cells as they are NOW
        (origin [0xb500]/[0xb504], PD depth [0x08ce] - only valid after REMOTE, table position
        [0x0fc0..])."""
        j = self.job
        m = self.m
        u = m.cpu.uc
        w = lambda a: int.from_bytes(bytes(u.mem_read(a, 2)), "little")
        sw = lambda a: int.from_bytes(bytes(u.mem_read(a, 4)), "little", signed=True)
        origin = (w(0xB500), w(0xB504))
        start = (sw(0x0FC0), sw(0x0FC4), sw(0x0FC8) - sw(0x04B6))   # Z relative to the panel surface Z0
        zd = w(0x08CE)
        r = program.reference(j["text"], j["fmt"], origin, start, zd)
        if self.physics:
            a = m.subcpu.axes
            phys = [a["X"].pos, a["Y"].pos, a["Z"].pos]
            lim = [a["X"].travel_steps, a["Y"].travel_steps, a["Z"].travel_steps]
        else:
            phys = self._pulses(start)
            lim = [int((483 + 2 * peer.ORIGIN_MARGIN_MM["X"]) * 2000), int((305 + 2 * peer.ORIGIN_MARGIN_MM["Y"]) * 2000), 248000]
        self.ref = [self._mm_from(q) + (q[3],) for q in self._target_path(r.points, start, phys, lim)]
        self.ref_id += 1
        table = (w(0x06AA) * 200, w(0x06AC) * 200)
        notes = list(r.notes) + r.bounds(table[0], table[1])
        # Feed: G-code carries F (mm/min -> mm/s), HP-GL carries none and runs at the machine's XY-ES
        # ([0x09f6], 0.1 mm/s). That explains the time difference of the same geometry in both formats
        # (24x24: F203/F254 = 8/10 in/min = 3.4/4.2 mm/s against XY-ES 50).
        xyes = w(0x09F6) / 10.0
        fs = sorted(set(round(f, 2) for f in r.feeds if f))
        if fs:
            j["feed"] = "F %g..%g mm/s (%g..%g mm/min)" % (fs[0], fs[-1], round(fs[0] * 60, 1), round(fs[-1] * 60, 1))
            if fs[-1] < xyes / 2:
                notes.append("Feed in the program at most %g mm/s (F%g mm/min), machine XY-ES %g mm/s: "
                             "the HP-GL output of the same geometry carries no feed and would run at %g mm/s - "
                             "check the CAM feed (in/min instead of mm/min?)" % (fs[-1], round(fs[-1] * 60, 1), xyes, xyes))
        else:
            j["feed"] = "no F in the program: XY-ES %g mm/s (Z-ES %g)" % (xyes, w(0x09F8) / 10.0)
        rf = r.ramp_faults(xyes)
        if rf:
            notes.append("%d segment(s) hit the ramp fault of the stock firmware (axis-parallel 16.4..25.5 mm "
                         "above 40 mm/s): harmless on the machine, on the test bench the record does not end - "
                         "choose F cap 35 mm/s (header), first at segment %d" % (len(rf), rf[0]))
        j["ramp_faults"] = len(rf)
        j.update(segments=len(r.points) - 1, arcs=r.arcs, cut_mm=round(r.cut_length_mm, 1), notes=notes,
                 ref_id=self.ref_id, origin=origin, zd=zd)

    def z_zero_mm(self):
        """Position of the firmware's Z zero (table Z = 0) in axis mm: axis - 20 * table.
        Only re-determined at standstill (ring empty): during a record the table already sits
        on the target while the axis is still moving - the value would jump by the record depth,
        and the display (everything relative to Z0) with it (spiral.nc)."""
        m = self.m
        if m is None or not self.physics:
            return 0.0
        u = m.cpu.uc
        if self._z_zero_latched is None or u.mem_read(0x1004, 1)[0] == u.mem_read(0x1005, 1)[0]:
            t = (int.from_bytes(bytes(u.mem_read(0x0FC8, 4)), "little", signed=True)
                 - int.from_bytes(bytes(u.mem_read(0x04B6, 4)), "little", signed=True))
            self._z_zero_latched = round((m.subcpu.axes["Z"].pos - 20 * t) / 4000.0, 3)
        return self._z_zero_latched

    def _set_z_zero(self, mm):
        """Put the Z zero `mm` below the top position, like the zero key on the panel: the surface
        [0x04b6] (32 bit, 5 um from the top; table [0x0fc8] stays absolute). Earlier versions shifted
        the table cells [0x0fc8]/[0x0fd4]/[0x193e]/[0x1f60] here - surface = table zero, the path
        that hid the Z-zero bug of ZQ and G-code. Only at standstill."""
        m = self.m
        u = m.cpu.uc
        if u.mem_read(0x1004, 1)[0] != u.mem_read(0x1005, 1)[0]:
            return False
        u.mem_write(0x04B6, int(round(float(mm) * 200)).to_bytes(4, "little", signed=True))
        self._z_zero_latched = None
        return True

    def origin(self):
        """The effective ORIGIN position from the RAM copy [0xb500]/[0xb504] - not the NVRAM cell."""
        if self.m is None:
            return "-"
        u = self.m.cpu.uc
        v = (int.from_bytes(bytes(u.mem_read(0xB500, 2)), "little"), int.from_bytes(bytes(u.mem_read(0xB504, 2)), "little"))
        for k, val in ORIGINS.items():
            if val == v:
                return k
        return "%g,%g mm" % (v[0] / 200.0, v[1] / 200.0)

    def origin_nvram(self):
        """The ORIGIN setting in the NVRAM (cell 0x18c: 1 = LOW LEFT, 0 = CENTER) - takes effect
        at the next boot; the UI updates the RAM copy immediately when switching."""
        if self.m is None:
            return "-"
        v = self.m.nvram.cells[NVRAM_ORIGIN_CELL]
        for k, val in ORIGIN_NVRAM.items():
            if val == v:
                return k
        return "?%d" % v

    def _load(self, arg):
        text = arg.get("text", "")
        fmt = arg.get("fmt") or program.detect_language(text)
        self.job = {"name": arg.get("name", "?"), "fmt": fmt, "text": text, "bytes": len(text),
                    "segments": 0, "cut_mm": 0.0, "notes": [], "status": "loaded", "ref_id": 0,
                    "origin": None, "zd": 0, "k": 1, "maxdev": 0.0, "maxdev_at": None,
                    "sumdev": 0.0, "n": 0, "strobes0": 0, "end_error": None, "time_s": 0.0, "idle": 0}
        self._reference()

    def _approach(self):
        """Like `harness.ready()`: REMOTE, then `IN;SP1;PA0,0;` in HP-GL - the job starts at the
        origin, as on the test bench. After boot the machine sits in the lower left corner,
        and that is exactly what the firmware's position table [0x0fc0..] = (0, 0, 0) says: it counts
        absolutely from LOW LEFT, the ORIGIN position [0xb500] is added to the program coordinates.
        With ORIGIN LOW LEFT the approach moves nothing, with CENTER it drives to the table centre
        (241.5 / 152.5 mm). The "offset of 219 mm" described here earlier was the emulator's wrong
        axis frame (X rest position 22.6 mm before the end stop), not a state of the firmware."""
        m = self.m
        mode_byte = None
        if self.is_max:
            mode_byte = m.cpu.uc.mem_read(MODE_CELL, 1)[0]
            m.cpu.uc.mem_write(MODE_CELL, b"\x00")
        if self._remote():
            m.uart.rx.extend(b"IN;SP1;PA0,0;")
            idle, last = 0, None
            for _ in range(200):
                m.run(m.instr + 2000000)
                self._harvest()
                if len(m.uart.rx) == 0:
                    idle = idle + 1 if m.motion.strobes == last else 0
                    last = m.motion.strobes
                    if idle >= 5 and m.cpu.uc.mem_read(0x1004, 1)[0] == m.cpu.uc.mem_read(0x1005, 1)[0]:
                        break
        if mode_byte is not None:
            m.cpu.uc.mem_write(MODE_CELL, bytes([mode_byte]))
        self.path = []
        self.path_gen = self.path_gen + 1 if hasattr(self, "path_gen") else 0   # invalidates the client buffers (/path)
        self._last = None

    def _remote(self):
        m = self.m
        for _ in range(3):
            t = " ".join(m.panel.text())
            if "[REMOTE]" in t or "[GCODE]" in t:
                return True
            m.panel.press(1, 3)
            m.run(m.instr + HARN.KEY_HOLD)
            m.panel.release_all()
            m.run(m.instr + 1800000)
        t = " ".join(m.panel.text())
        return "[REMOTE]" in t or "[GCODE]" in t

    def _start(self):
        j = self.job
        m = self.m
        if j is None or j["status"] == "running":
            return
        if not self._remote():
            j["notes"] = j["notes"] + ["REMOTE not reached"]
            return
        if self.z_zero_target is not None and abs(self.z_zero_mm() - self.z_zero_target) > 0.01:
            # the firmware has reset the Z table in the meantime (idle): apply again
            if not self._set_z_zero(self.z_zero_target):
                j["notes"] = j["notes"] + ["Z0 %g mm could not be set (ring not empty)" % self.z_zero_target]
        self._reference()               # now with ZD and the position after REMOTE
        if self.is_max and j["fmt"] == "gcode" and self.mode() != "GCODE":
            j["notes"] = j["notes"] + ["Operating mode is %s, the program is G-code" % self.mode()]
        if self.is_max and j["fmt"] == "hpgl" and self.mode() == "GCODE":
            j["notes"] = j["notes"] + ["Operating mode is GCODE, the program is HP-GL"]
        j.update(status="running", k=1, maxdev=0.0, maxdev_at=None, sumdev=0.0, n=0,
                 strobes0=m.motion.strobes, end_error=None, time_s=0.0, idle=0, ref_id=self.ref_id)
        text = j["text"]
        if j["fmt"] == "gcode" and not text.endswith("\n"):
            text += "\n"
        if self.fcap and j["fmt"] == "gcode":
            # test bench cap: replace F words above the cap (mm/min) so that the
            # ramp fault 366a does not trigger; the text in the job stays as loaded
            import re as _re
            bound = int(round(self.fcap * 60))
            text = _re.sub(r'([Ff])\s*(\d+(?:\.\d*)?)', lambda mm: mm.group(1) + str(min(bound, int(float(mm.group(2))))), text)
            j["notes"] = j["notes"] + ["F cap %g mm/s applied" % self.fcap]
        j["fault_record"] = None
        m.uart.rx.extend(text.encode("latin-1", "replace"))

    def _stop(self):
        m = self.m
        try:
            m.uart.rx.clear()
        except Exception:
            del m.uart.rx[:]
        if self.job is not None and self.job["status"] == "running":
            self.job["status"] = "aborted"
        try:
            self.m.cpu.uc.mem_write(0x3300, bytes([self.m.cpu.uc.mem_read(0x3301, 1)[0]]))   # empty the shadow queue
        except Exception:
            pass

    def _measure(self, p):
        """Distance of the new path point from the reference path (XY), only around the current segment."""
        j = self.job
        ref = self.ref
        k = j["k"]
        best, bi = None, k
        for i in range(max(1, k - 2), min(len(ref), k + 40)):
            d = program.distance_2d(p, ref[i - 1], ref[i])
            if best is None or d < best:
                best, bi = d, i
        if best is not None and best > 0.2:
            # The window has lost track (contours that are run several times at different
            # depths keep the pointer on the first copy; a long move then lies outside the
            # 40 segments): search once over the whole reference path.
            # Before this, 8..10 mm "deviation" showed up in the log although the point sat exactly
            # on a segment (24x24.nc.plt, segment 16301; pocket 10.3 mm).
            for i in range(1, len(ref)):
                d = program.distance_2d(p, ref[i - 1], ref[i])
                if d < best:
                    best, bi = d, i
                    if d < 0.01:
                        break
        j["k"] = bi
        j["sumdev"] += best
        j["n"] += 1
        if best > j["maxdev"]:
            j["maxdev"] = best
            j["maxdev_at"] = p

    def _job_tick(self):
        j = self.job
        m = self.m
        if j is None or j["status"] != "running":
            return
        j["time_s"] = round((m.motion.strobes - j["strobes0"]) * TICK_MS / 1000.0, 2)
        if j.get("fault_record") and j["status"] == "running":
            j["status"] = "halted (ramp fault 366a, model limit)"
            j["notes"] = j["notes"] + ["Ramp fault of the stock firmware on the test bench: %s - the test bench's executor does not let it end; harmless on the machine (measured 2026-09-05). Set F cap 35 mm/s and restart." % j["fault_record"]]
            self._stop()
            return
        if self.is_max and m.cpu.uc.mem_read(0x1E79, 1)[0] and j["status"] == "running":
            j["status"] = "halted (STOP)"
            j["notes"] = j["notes"] + ["Firmware STOP: unknown word, limit or overflow - see counters"]
            # Diagnosis straight from the branch's RAM: the rejected line (M118 buffer 0x34d0,
            # length + 79 characters), the partial line buffer 0x1e80 and the counters - otherwise
            # a STOP cause could only be fetched over the line, which still carries the rest of the job
            u = m.cpu.uc
            n = min(u.mem_read(0x34D0, 1)[0], 79)
            rejected = bytes(u.mem_read(0x34D1, n)).decode("latin-1", "replace") if n else ""
            zn = min(u.mem_read(0x1ED0, 1)[0], 79)
            partial = bytes(u.mem_read(0x1E80, zn)).decode("latin-1", "replace") if zn else ""
            j["diagnosis"] = {"rejected": rejected, "line_buffer": partial,
                              "error": u.mem_read(0x1ED1, 1)[0], "limit": int.from_bytes(bytes(u.mem_read(0x1FC0, 2)), "little"),
                              "overflow": u.mem_read(0x1E78, 1)[0], "skip": u.mem_read(0x1E7B, 1)[0]}
            return
        if len(m.uart.rx) == 0:
            if m.motion.strobes == j.get("_strobes_last"):
                j["idle"] += 1
            else:
                j["idle"] = 0
            j["_strobes_last"] = m.motion.strobes
            if j["idle"] >= 12:           # about 12 run chunks without a strobe: the job is done
                j["status"] = "done"
                if self.path and len(self.ref) > 1:
                    e = self.ref[-1]
                    q = self.path[-1]
                    j["end_error"] = round(((q[0] - e[0]) ** 2 + (q[1] - e[1]) ** 2) ** 0.5, 3)
                if self.pp is not None:
                    a = m.subcpu.axes
                    dev = max(abs(self.pp[0] - a["X"].pos), abs(self.pp[1] - a["Y"].pos), abs(self.pp[2] - a["Z"].pos))
                    if dev > 4:
                        j["notes"] = j["notes"] + ["Path reconstruction deviates from the axes: %d pulses" % dev]

    def mode(self):
        if not self.is_max or self.m is None:
            return "-"
        v = self.m.cpu.uc.mem_read(MODE_CELL, 1)[0]
        return {0xA5: "OPT", 0x5A: "GCODE"}.get(v, "STD")

    def run(self):
        try:
            self.phase = "homing"
            m = self.boot()
            if not self.snapshot:
                m.run(60000000)
            if self.stop:
                return
            if self.physics:
                self.phase = "approach"
                self._approach()
            self.phase = "ready"
            while not self.stop:
                while True:
                    try:
                        kind, arg = self.cmds.get_nowait()
                    except queue.Empty:
                        break
                    self._apply(kind, arg)
                t0 = time.perf_counter()
                m.run(m.instr + CHUNK)
                t1 = time.perf_counter()
                self._harvest()
                self._job_tick()
                t2 = time.perf_counter()
                self.timing["run"] += t1 - t0
                self.timing["harvest"] += t2 - t1
                self.timing["chunks"] += 1
        except Exception:                       # surface it in the UI
            import traceback
            self.err = traceback.format_exc()
            self.phase = "error"
        finally:
            self.phase = "stopped" if self.stop else self.phase

    # ---- shutdown ------------------------------------------------------
    def request_stop(self):
        """Called from the main thread on Ctrl-C.

        Unicorn is running in another thread, inside uc_emu_start, with Python
        callbacks hooked into it. Letting the interpreter finalise in that state
        segfaults the process, which is exactly the core dump Ctrl-C produced.
        So: ask the machine to stop at the next slice, and stop the engine now.
        """
        self.stop = True
        m = self.m
        if m is not None:
            m.abort = True
            try:
                m.cpu.uc.emu_stop()
            except Exception:
                pass

    def save_nvram(self):
        m = self.m
        if m is None or not self.nvram:
            return False
        try:
            m.nvram.save(self.nvram)
            self.saved = m.instr
            return True
        except Exception:
            import traceback
            self.err = traceback.format_exc()
            return False

    def _apply(self, kind, arg):
        m = self.m
        if kind == "key":
            m.panel.press(int(arg["col"]), int(arg["bit"]))
            m.run(m.instr + HARN.KEY_HOLD)
            m.panel.release_all()
            m.run(m.instr + 900000)
        elif kind == "send":
            m.uart.rx.extend(arg["text"].encode("latin-1", "replace"))
        elif kind == "savenvram":
            self.save_nvram()
        elif kind == "clearpath":
            self.path = []
            self.path_gen = self.path_gen + 1 if hasattr(self, "path_gen") else 0   # invalidates the client buffers (/path)
            self._last = None
        elif kind == "load":
            self._load(arg)
        elif kind == "start":
            self._start()
        elif kind == "stop":
            self._stop()
        elif kind == "mode":
            if self.is_max and arg.get("mode") in MODES:
                m.cpu.uc.mem_write(MODE_CELL, bytes([MODES[arg["mode"]]]))
        elif kind == "fcap":
            self.fcap = float(arg.get("mm_s", 0) or 0)
        elif kind == "baud":
            b = int(arg.get("baud", 9600))
            if b in BAUD:
                self.baud = b
                m.uart_byte_interval = BAUD[b]
        elif kind == "origin":
            name = str(arg.get("origin", "")).strip()
            name = ORIGIN_ALIAS.get(name.upper().replace(" ", ""), ORIGIN_ALIAS.get(name, name.upper()))
            o = ORIGINS.get(name)
            if o is not None:
                # like the menu MODE SET > ORIGIN: the setting goes into the NVRAM (takes effect at the
                # next boot, "save" writes the file), the RAM copy immediately
                if m.nvram.cells[NVRAM_ORIGIN_CELL] != ORIGIN_NVRAM[name]:
                    m.nvram.cells[NVRAM_ORIGIN_CELL] = ORIGIN_NVRAM[name]
                    m.nvram.dirty = True
            if o is None:
                try:                    # "x,y" in mm: free origin (real-world programs
                    x, y = str(arg.get("origin", "")).split(",")     # go below 0 by the tool radius)
                    o = (int(round(float(x) * 200)), int(round(float(y) * 200)))
                except ValueError:
                    o = None
            if o is not None:
                u = m.cpu.uc
                u.mem_write(0xB500, o[0].to_bytes(2, "little"))
                u.mem_write(0xB504, o[1].to_bytes(2, "little"))
                if self.job is not None and self.job["status"] != "running":
                    self._reference()
        elif kind == "zzero":
            self.z_zero_target = float(arg.get("mm", 0) or 0)
            if self._set_z_zero(arg.get("mm", 0)):
                if self.job is not None and self.job["status"] != "running":
                    self._reference()
            elif arg.get("_attempts", 0) < 200:
                # the ring was not empty yet (the approach is still running out): try again
                # after the next run chunk
                self.cmds.put(("zzero", dict(arg, _attempts=arg.get("_attempts", 0) + 1)))

    # ---- what the page reads -----------------------------------------
    def state(self):
        t0 = time.perf_counter()
        try:
            return self._state()
        finally:
            self.timing["state"] += time.perf_counter() - t0

    def _state(self):
        m = self.m
        if m is None:
            return {"phase": self.phase, "err": self.err}
        try:
            lcd = [r.rstrip() for r in m.panel.text()]
        except Exception:
            lcd = []
        ax = {}
        if self.physics:
            for k, a in m.subcpu.axes.items():
                ax[k] = {"mm": round(a.table_mm(), 3),          # table coordinate, 0 = LOW LEFT
                         "steps": a.pos,
                         "travel": round(a.travel_steps / a.STEPS_PER_MM, 1),
                         "min": round(-a.origin_steps / a.STEPS_PER_MM, 1),
                         "max": round((a.travel_steps - a.origin_steps) / a.STEPS_PER_MM, 1),
                         "home": bool(a.at_home()),
                         "hit": int(a.hit_low) + int(a.hit_high), "clipped": int(a.clipped)}
        job = None
        if self.job is not None:
            j = self.job
            job = {k: j.get(k) for k in ("name", "fmt", "bytes", "segments", "arcs", "cut_mm", "notes", "status", "feed",
                                         "ref_id", "maxdev", "maxdev_at", "n", "end_error", "time_s", "ramp_faults", "diagnosis")}
            job["meandev"] = round(j["sumdev"] / j["n"], 4) if j["n"] else 0.0
            job["maxdev"] = round(j["maxdev"], 3)
            job["rest"] = len(m.uart.rx)
            job["strobes"] = m.motion.strobes - j["strobes0"] if j["status"] != "loaded" else 0
            if self.is_max:
                u = m.cpu.uc
                rd = lambda a, n=1: int.from_bytes(bytes(u.mem_read(a, n)), "little")
                job["stop"] = rd(0x1E79); job["error"] = rd(0x1ED1); job["limit"] = rd(0x1FC0, 2); job["overflow"] = rd(0x1E78)
                job["tx_tail"] = bytes(m.uart.tx[-300:]).decode("latin-1", "replace")   # make the firmware's
                # replies (M114/M118 after a STOP) visible - otherwise the server never reads tx
        return {
            "phase": self.phase, "err": self.err,
            "rom": os.path.basename(self.rom), "max": self.is_max, "mode": self.mode(), "baud": self.baud,
            "origin": self.origin(), "origin_nvram": self.origin_nvram(), "z_zero": self.z_zero_mm(), "z_zero_target": self.z_zero_target, "fcap": self.fcap,
            "ring": [m.cpu.uc.mem_read(0x1004, 1)[0], m.cpu.uc.mem_read(0x1005, 1)[0]],
            "timing": dict(self.timing, wall=time.time() - self.timing["start"], path=len(self.path)),
            "job": job,
            "lcd": lcd,
            "instr": m.instr,
            "strobes": m.motion.strobes,
            "queue": len(m.uart.rx),
            "axes": ax,
            "cmd": dict(m.motion.pos),
            "path_gen": self.path_gen, "path_n": len(self.path),   # the path itself: GET /path?from=N&gen=G
            "nvram": {"file": self.nvram or "-",
                      "dirty": bool(m.nvram.dirty),
                      "saved": self.saved},
        }


EMU = None

PAGE = r"""<!doctype html><html><head><meta charset="utf-8">
<title>ME-500 Emulator</title><style>
:root{--bg:#14161a;--fg:#dfe3ea;--dim:#8b93a3;--line:#2a2f3a;--acc:#5aa9e6}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);
 font:13px/1.45 ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}
header{padding:8px 14px;border-bottom:1px solid var(--line);display:flex;
 gap:18px;align-items:baseline;flex-wrap:wrap}
h1{font-size:14px;margin:0;font-weight:600;letter-spacing:.04em}
.stat{color:var(--dim)}
main{display:grid;grid-template-columns:minmax(420px,1fr) 340px;gap:14px;padding:14px}
@media(max-width:900px){main{grid-template-columns:1fr}}
.card{border:1px solid var(--line);border-radius:8px;padding:10px;background:#181b21}
.card h2{font-size:11px;margin:0 0 8px;color:var(--dim);font-weight:600;
 text-transform:uppercase;letter-spacing:.09em}
canvas{width:100%;height:auto;display:block;background:#0f1115;border-radius:5px}
#lcd{background:#0d2b12;color:#8dff9b;padding:9px 11px;border-radius:5px;
 font-size:15px;letter-spacing:.10em;white-space:pre;line-height:1.6;
 text-shadow:0 0 6px rgba(141,255,155,.45)}
.keys{display:grid;grid-template-columns:repeat(3,1fr);gap:5px}
button{background:#232833;color:var(--fg);border:1px solid var(--line);
 border-radius:5px;padding:6px 4px;cursor:pointer;font:inherit;font-size:11px}
button:hover{border-color:var(--acc);color:#fff}
button:active{background:var(--acc);color:#08111a}
button.mode{padding:2px 7px;font-size:11px}button.mode.on{border-color:var(--acc);color:#fff;background:#2a3c50}
textarea{width:100%;height:120px;background:#0f1115;color:var(--fg);
 border:1px solid var(--line);border-radius:5px;padding:8px;font:inherit;resize:vertical}
.row{display:flex;gap:6px;margin-top:7px;flex-wrap:wrap}
.row button{flex:1;padding:7px}
table{width:100%;border-collapse:collapse}
td{padding:2px 0}td:last-child{text-align:right;color:var(--dim)}
.home{color:#8dff9b}
label{color:var(--dim);font-size:11px}
</style></head><body>
<header>
 <h1>MIMAKI ME-500 &mdash; Emulator</h1>
 <span class="stat">State: <b id="phase">-</b></span>
 <span class="stat">Instr <b id="instr">0</b></span>
 <span class="stat">Strobes <b id="strobes">0</b></span>
 <span class="stat">UART queue <b id="queue">0</b></span>
 <span class="stat">NVRAM <b id="nv">-</b>
  <button style="padding:2px 7px;font-size:11px" onclick="post('/savenvram',{})">save</button></span>
 <span class="stat">ROM <b id="rom">-</b></span>
 <span class="stat" id="modes">PATH PLAN
  <button class="mode" onclick="post('/mode',{mode:'STD'})">STD</button>
  <button class="mode" onclick="post('/mode',{mode:'OPT'})">OPT</button>
  <button class="mode" onclick="post('/mode',{mode:'GCODE'})">GCODE</button></span>
 <span class="stat">Baud <select id="baud" onchange="post('/baud',{baud:+this.value})">
  <option>9600</option><option>19200</option><option>38400</option></select></span>
 <span class="stat" title="The machine's MODE SET > ORIGIN: LOW LEFT = lower left corner of the table (0,0), CENTER = table centre (241.5 / 152.5 mm). Takes effect immediately and is written to the NVRAM (save!)">ORIGIN <b id="origin">-</b>
  <button class="mode" onclick="post('/origin',{origin:'LOW LEFT'})">LOW LEFT</button>
  <button class="mode" onclick="post('/origin',{origin:'CENTER'})">CENTER</button>
  <input id="orxy" type="text" value="5,5" size="6" title="X,Y in mm" style="font:inherit;background:#232833;color:var(--fg);border:1px solid var(--line);border-radius:5px;width:58px">
  <button class="mode" onclick="post('/origin',{origin:document.getElementById('orxy').value})">X,Y mm</button></span>
 <span class="stat">Z0 <b id="zzero">-</b> mm below top
  <input id="znmm" type="number" value="10" min="0" max="50" step="0.5" style="width:52px;font:inherit;background:#232833;color:var(--fg);border:1px solid var(--line);border-radius:5px">
  <button class="mode" onclick="post('/zzero',{mm:+document.getElementById('znmm').value})">set</button></span>
 <span class="stat" title="Test bench: F words above the cap are replaced when sending (ramp fault 366a from 40 mm/s)">F cap
  <select id="fcap" onchange="post('/fcap',{mm_s:+this.value})"><option value="0">off</option><option value="35">35 mm/s</option><option value="30">30 mm/s</option></select></span>
</header>
<main>
 <div>
  <div class="card">
   <h2>Tool paths &mdash; reference and actual path, colour by depth (3D: left mouse button rotates, wheel zooms, right button pans)</h2>
   <div id="v3" style="width:100%;height:640px;background:#0f1115;border-radius:5px;overflow:hidden"></div>
   <canvas id="cv" width="1000" height="640" style="display:none"></canvas>
   <div class="row" style="align-items:center">
    <label><input type="radio" name="vm" value="3d" checked> 3D</label>
    <label><input type="radio" name="vm" value="2d"> 2D (classic)</label>
    <button onclick="if(window.v3)v3.view('top')" title="straight from above">Top</button>
    <button onclick="if(window.v3)v3.view('iso')" title="oblique from front left">Oblique</button>
    <button onclick="if(window.v3)v3.view('fit')" title="fit path and reference, keep the viewing direction">Fit</button>
    <label><input type="checkbox" id="ortho"> orthographic</label>
    <label title="Z is stretched for display; colours and numbers stay real">Z x<input type="range" id="zx" min="1" max="10" step="1" value="3" style="width:70px;vertical-align:middle"><span id="zxv">3</span></label>
    <label><input type="checkbox" id="iso" checked> isometric (2D)</label>
    <label><input type="checkbox" id="grid" checked> Grid</label>
    <label><input type="checkbox" id="showref" checked> Reference</label>
    <button onclick="clearPath()">Clear path</button>
   </div>
   <div id="v3info" style="color:#6b7385;font-size:11px;margin-top:5px;font-family:inherit">-</div>
  </div>
  <div class="card"><h2>Program from file &mdash; HP-GL or G-code, against its own geometry</h2>
   <div class="row" style="margin-top:0">
    <input type="file" id="file" accept=".plt,.hpgl,.hgl,.gcode,.nc,.ngc,.txt,.gc,.tap" style="flex:2;font:inherit;color:var(--dim)">
    <select id="fmt" style="font:inherit;background:#232833;color:var(--fg);border:1px solid var(--line);border-radius:5px">
     <option value="">Format: auto</option><option value="hpgl">HP-GL</option><option value="gcode">G-code</option></select>
    <button onclick="startJob()" id="btnstart">Start</button>
    <button onclick="post('/stop',{})">Stop</button>
   </div>
   <table id="job" style="margin-top:8px"><tr><td>no program loaded</td><td></td></tr></table>
  </div>
 </div>
 <div>
  <div class="card"><h2>LCD and function keys</h2>
   <div style="display:flex;gap:8px;align-items:stretch">
    <div id="lcd" style="flex:1">...</div>
    <div id="soft" style="display:flex;flex-direction:column;gap:3px;justify-content:flex-end"></div>
   </div></div>
  <div class="card"><h2>Control panel</h2>
   <div id="panel"></div>
   <details style="margin-top:9px">
    <summary style="cursor:pointer;color:var(--dim);font-size:11px">
     Raw matrix &mdash; 3 columns x 8 bits</summary>
    <div class="keys" id="keys" style="margin-top:7px"></div>
   </details>
  </div>
  <div class="card"><h2>Axes</h2><table id="axes"></table></div>
  <div class="card"><h2>HP-GL to the UART</h2>
   <textarea id="hpgl">IN;SP1;PA0,0;PD1000,0;PD1000,1000;PD0,1000;PD0,0;PU;</textarea>
   <div class="row">
    <button onclick="send()">Send</button>
    <button onclick="preset('rect')">Rectangle</button>
    <button onclick="preset('star')">Star</button>
    <button onclick="preset('arc')">Arc</button>
   </div>
  </div>
  <div class="card"><h2>Errors</h2><pre id="err" style="white-space:pre-wrap;color:#ff9b9b;margin:0">-</pre></div>
 </div>
</main>
<script>
const cv=document.getElementById('cv'),cx=cv.getContext('2d');
let path=[],axes={},zzero=0;
const TABLE=[483,305];   // model set [0x06aa]/[0x06ac], table coordinates from LOW LEFT
// Display Z: program coordinate, negative = below the zero point (the cutter plunges
// DOWN); the axis counts positive downwards from the upper end stop.
const zp=p=>[p[0],p[1],-(p[2]-zzero),p[3]];

// The panel as the operation manual lays it out on page 2-2: 17 keys.
// Only REMOTE/LOCAL is a known matrix position; the rest are marked null until
// a key sweep assigns them. A null key is drawn dimmed and
// does nothing, so the picture never pretends to a mapping it does not have.
const PANEL=[
 [['REMOTE/LOCAL',[1,3]],['SPINDLE ON/OFF',[1,1]]],
 [['END',[1,2]],['CANCEL / CE',[1,0]]],
 [['PAUSE',[2,2]],['Z AXIS / depth',[2,3]]],
 [['Z ▼ (down)',[2,5]],['Z ▲ / MOVE',[2,6]]],
 [['XY ORIGIN',[2,7]],['Z ZERO',[2,4]]]
];
// To the right of the LCD, as on the real panel: the four lines are labelled there
// PAGE, F1, F2, F3, and each F key is a PAIR of + and -.
// The UI long carried them as "line 2" plus a nameless twin -
// wrong, although the firmware analysis had long named the pair Fn+/Fn-:
// (0,5) steps the COMMAND line forward, (0,2) backward.
const SOFT=[['PAGE',[0,0],[0,1]],
            ['F1',[0,7],[0,4]],
            ['F2',[0,6],[0,3]],
            ['F3',[0,5],[0,2]]];
// Directions measured inside <MOVE>: 1/4 drives X positive, 1/7 negative,
// 1/5 Y positive, 1/6 negative.
const ARROWS=[[null,['▲ Y+',[1,5]],null],
              [['◀ X−',[1,7]],['XY',null],['▶ X+',[1,4]]],
              [null,['▼ Y−',[1,6]],null]];

function keyBtn(label,pos){
  const b=document.createElement('button');
  b.textContent=label;
  if(pos){b.onclick=()=>post('/key',{col:pos[0],bit:pos[1]});
          b.title='Matrix '+pos[0]+'/'+pos[1];}
  else{b.disabled=true;b.style.opacity=.35;b.title='matrix position not yet known';}
  return b;
}
const sf=document.getElementById('soft');
for(const [lbl,plus,minus] of SOFT){
  const d=document.createElement('div');
  d.style.cssText='display:flex;gap:3px;align-items:center';
  const tag=document.createElement('span');
  tag.textContent=lbl;
  tag.style.cssText='width:30px;color:var(--dim);font-size:10px;text-align:right';
  const p=keyBtn('+',plus); p.style.cssText+=';font-size:11px;padding:3px 8px';
  const mn=keyBtn('−',minus); mn.style.cssText+=';font-size:11px;padding:3px 8px';
  p.title=lbl+' +   Matrix '+plus[0]+'/'+plus[1];
  mn.title=lbl+' −   Matrix '+minus[0]+'/'+minus[1];
  d.appendChild(tag); d.appendChild(p); d.appendChild(mn); sf.appendChild(d);
}
const pn=document.getElementById('panel');
const arr=document.createElement('div');
arr.style.cssText='display:grid;grid-template-columns:repeat(3,1fr);gap:4px;margin-bottom:9px';
for(const row of ARROWS)for(const cell of row){
  if(!cell){const d=document.createElement('div');arr.appendChild(d);continue;}
  arr.appendChild(keyBtn(cell[0],cell[1]));
}
pn.appendChild(arr);
for(const row of PANEL){
  const d=document.createElement('div');
  d.style.cssText='display:grid;grid-template-columns:repeat('+row.length+',1fr);gap:4px;margin-bottom:4px';
  for(const cell of row) d.appendChild(keyBtn(cell[0],cell[1]));
  pn.appendChild(d);
}
const kb=document.getElementById('keys');
for(let bit=0;bit<8;bit++)for(let col=0;col<3;col++){
  const b=document.createElement('button');
  b.textContent=col+'/'+bit;
  b.onclick=()=>post('/key',{col,bit});
  kb.appendChild(b);
}

function post(u,d){return fetch(u,{method:'POST',body:JSON.stringify(d)});}
function send(){post('/send',{text:document.getElementById('hpgl').value});}
function clearPath(){post('/clearpath',{});}
let ref=[],refId=0,jobName='';
document.getElementById('file').addEventListener('change',ev=>{
  const f=ev.target.files[0]; if(!f)return;
  const rd=new FileReader();
  rd.onload=()=>{jobName=f.name;
    post('/load',{name:f.name,text:rd.result,fmt:document.getElementById('fmt').value});};
  rd.readAsText(f,'latin1');
});
function startJob(){post('/start',{});}
function fmtJob(j){
  if(!j)return '<tr><td>no program loaded</td><td></td></tr>';
  const r=[['File',j.name+' ('+(j.fmt==='gcode'?'G-code':'HP-GL')+', '+j.bytes.toLocaleString('en')+' bytes)'],
   ['Segments / cut length',j.segments+(j.arcs?' ('+j.arcs+' arcs)':'')+' / '+j.cut_mm+' mm'],
   ['Feed',j.feed||'-'],
   ['Status',j.status+(j.status==='running'?' &mdash; '+j.rest.toLocaleString('en')+' bytes pending':'')],
   ['Machine time',j.time_s.toFixed(2)+' s  ('+(j.strobes||0).toLocaleString('en')+' strobes)'],
   ['XY deviation max / mean',(j.n?j.maxdev.toFixed(3)+' / '+j.meandev.toFixed(3)+' mm':'-')
     +(j.maxdev_at?' at X '+j.maxdev_at[0].toFixed(2)+' Y '+j.maxdev_at[1].toFixed(2):'')],
   ['End point vs. reference',j.end_error==null?'-':j.end_error.toFixed(3)+' mm'],
   ['Firmware: STOP / error / limit / overflow',j.stop==null?'-':j.stop+' / '+j.error+' / '+j.limit+' / '+j.overflow]];
  let h=r.map(x=>'<tr><td>'+x[0]+'</td><td>'+x[1]+'</td></tr>').join('');
  if(j.notes&&j.notes.length)h+='<tr><td colspan=2 style="color:#ffd166">'+j.notes.join('<br>')+'</td></tr>';
  return h;
}
const P={rect:'IN;SP1;PA0,0;PD2000,0;PD2000,2000;PD0,2000;PD0,0;PU;',
 star:'IN;SP1;PA0,0;'+Array.from({length:12},(_,i)=>{const a=i*5*Math.PI/6;
  return 'PD'+Math.round(1500+1400*Math.cos(a))+','+Math.round(1500+1400*Math.sin(a))+';'}).join('')+'PU;',
 arc:'IN;SP1;PA0,0;'+Array.from({length:60},(_,i)=>{const a=i*Math.PI/120;
  return 'PD'+Math.round(2000*Math.sin(a))+','+Math.round(2000-2000*Math.cos(a))+';'}).join('')+'PU;'};
function preset(k){document.getElementById('hpgl').value=P[k];send();}

function is2d(){const r=document.querySelector('input[name=vm]:checked');return !!r&&r.value==='2d';}
function showMode(){const d2=is2d();document.getElementById('cv').style.display=d2?'':'none';
  document.getElementById('v3').style.display=d2?'none':'';if(window.v3)v3.setActive(!d2);}
for(const r of document.querySelectorAll('input[name=vm]'))r.addEventListener('change',()=>{showMode();draw();});
function draw(){
  showMode();
  if(is2d()){draw2d();return;}
  let x0=Infinity,x1=-Infinity,y0=Infinity,y1=-Infinity;
  for(const p of path){if(p[0]<x0)x0=p[0];if(p[0]>x1)x1=p[0];if(p[1]<y0)y0=p[1];if(p[1]>y1)y1=p[1];}
  const i=window.v3?v3.info():null;
  document.getElementById('v3info').textContent=path.length?('X '+x0.toFixed(1)+'..'+x1.toFixed(1)+' mm   Y '+y0.toFixed(1)+'..'+y1.toFixed(1)
    +' mm   Z '+(i&&isFinite(i.zlo)?i.zlo.toFixed(2)+'..'+i.zhi.toFixed(2):'-')+' mm (program, negative = below Z0)   '+path.length+' points'
    +(window.v3?'':'   (3D library not loaded - /static/three.module.js?)')):'no motion yet';
}
// Actual path incrementally: /path delivers float32 triples from `have`; a new generation (clear,
// decimation) resets the buffers and reloads from the start
let have=0,gen=-1;
async function fetchPath(s){
  if(s.path_gen===gen&&s.path_n===have)return;
  for(let k=0;k<8;k++){
    const r=await fetch('/path?from='+have+'&gen='+gen);const buf=await r.arrayBuffer();
    if(buf.byteLength<12)return;
    const dv=new DataView(buf);const g=dv.getUint32(0,true),from=dv.getUint32(4,true),total=dv.getUint32(8,true);
    const f=new Float32Array(buf,12);
    if(g!==gen||from!==have){gen=g;have=0;path=[];if(window.v3)v3.reset();if(from!==0)continue;}
    for(let j=0;j<f.length;j+=3)path.push([f[j],f[j+1],f[j+2]]);
    if(window.v3)v3.append(f);
    have+=f.length/3;
    if(have>=total)break;
  }
}
function draw2d(){
  const W=cv.width,H=cv.height,iso=document.getElementById('iso').checked;
  cx.fillStyle='#0f1115';cx.fillRect(0,0,W,H);
  const showRef=document.getElementById('showref').checked&&ref.length>1;
  if(!path.length&&!showRef){cx.fillStyle='#4a5160';cx.font='13px monospace';
    cx.fillText('no motion yet',20,28);return;}
  const all=(showRef?path.concat(ref):path).map(zp);
  // bounds by loop: Math.min(...arr) breaks at ~100 000 points (argument limit)
  let x0=Infinity,x1=-Infinity,y0=Infinity,y1=-Infinity,z0=Infinity,z1=-Infinity;
  for(const p of all){if(p[0]<x0)x0=p[0];if(p[0]>x1)x1=p[0];if(p[1]<y0)y0=p[1];if(p[1]>y1)y1=p[1];
    if(p[2]<z0)z0=p[2];if(p[2]>z1)z1=p[2];}
  const spanx=Math.max(1,x1-x0),spany=Math.max(1,y1-y0),spanz=Math.max(0.001,z1-z0);
  const pad=46;
  const s=iso?Math.min((W-2*pad)/(spanx+spany),(H-2*pad)/((spanx+spany)*0.5+40))
             :Math.min((W-2*pad)/spanx,(H-2*pad)/spany);
  function pt(p){
    const x=p[0]-x0,y=p[1]-y0,z=(p[2]-z0);
    if(!iso)return[pad+x*s,H-pad-y*s];
    return[W/2+(x-y)*s*0.87, H-pad-((x+y)*s*0.5)-z*s*3.0];
  }
  if(document.getElementById('grid').checked){
    cx.strokeStyle='#1d2230';cx.lineWidth=1;
    const step=Math.pow(10,Math.floor(Math.log10(Math.max(spanx,spany)/4)));
    for(let gx=Math.ceil(x0/step)*step;gx<=x1;gx+=step){
      cx.beginPath();const a=pt([gx,y0,z0]),b=pt([gx,y1,z0]);
      cx.moveTo(a[0],a[1]);cx.lineTo(b[0],b[1]);cx.stroke();}
    for(let gy=Math.ceil(y0/step)*step;gy<=y1;gy+=step){
      cx.beginPath();const a=pt([x0,gy,z0]),b=pt([x1,gy,z0]);
      cx.moveTo(a[0],a[1]);cx.lineTo(b[0],b[1]);cx.stroke();}
  }
  // table outline (0..483 x 0..305 mm, origin LOW LEFT) for orientation
  cx.strokeStyle='rgba(90,169,230,.35)';cx.lineWidth=1;cx.setLineDash([6,4]);
  {const c=[[0,0],[TABLE[0],0],[TABLE[0],TABLE[1]],[0,TABLE[1]],[0,0]].map(q=>pt([q[0],q[1],z0]));
   cx.beginPath();cx.moveTo(c[0][0],c[0][1]);for(const q of c.slice(1))cx.lineTo(q[0],q[1]);cx.stroke();}
  cx.setLineDash([]);
  if(showRef){
    cx.lineWidth=1;cx.lineJoin='round';
    for(let i=1;i<ref.length;i++){
      const cut=ref[i][3];
      cx.strokeStyle=cut?'rgba(255,255,255,.55)':'rgba(140,150,170,.45)';
      cx.setLineDash(cut?[]:[4,4]);
      const a=pt(zp(ref[i-1])),b=pt(zp(ref[i]));
      cx.beginPath();cx.moveTo(a[0],a[1]);cx.lineTo(b[0],b[1]);cx.stroke();
    }
    cx.setLineDash([]);
  }
  cx.lineWidth=1.6;cx.lineJoin='round';cx.lineCap='round';
  for(let i=1;i<path.length;i++){
    const q=zp(path[i]),t=(z1-q[2])/spanz;        // deep = red
    cx.strokeStyle='hsl('+Math.round(205-165*t)+',78%,'+Math.round(38+26*t)+'%)';
    const a=pt(zp(path[i-1])),b=pt(q);
    cx.beginPath();cx.moveTo(a[0],a[1]);cx.lineTo(b[0],b[1]);cx.stroke();
  }
  if(path.length){const last=pt(zp(path[path.length-1]));
  cx.fillStyle='#ffd166';cx.beginPath();cx.arc(last[0],last[1],4,0,7);cx.fill();}
  cx.fillStyle='#6b7385';cx.font='11px monospace';
  cx.fillText('X '+x0.toFixed(1)+'..'+x1.toFixed(1)+' mm   Y '+y0.toFixed(1)+'..'+y1.toFixed(1)
    +' mm   Z '+z0.toFixed(2)+'..'+z1.toFixed(2)+' mm   '+path.length+' points',12,H-10);
}

async function tick(){
  try{
    const s=await (await fetch('/state')).json();
    document.getElementById('phase').textContent=s.phase||'-';
    document.getElementById('instr').textContent=(s.instr||0).toLocaleString('en');
    document.getElementById('strobes').textContent=(s.strobes||0).toLocaleString('en');
    document.getElementById('queue').textContent=s.queue||0;
    const nv=s.nvram||{};
    document.getElementById('nv').textContent=
      nv.file?(nv.dirty?'modified':(nv.saved?'saved':'unchanged')):'off';
    document.getElementById('err').textContent=s.err||'-';
    document.getElementById('rom').textContent=s.rom||'-';
    document.getElementById('modes').style.display=s.max?'':'none';
    for(const b of document.querySelectorAll('button.mode'))b.classList.toggle('on',b.textContent===s.mode);
    if(document.activeElement!==document.getElementById('baud'))document.getElementById('baud').value=s.baud||9600;
    if(document.activeElement!==document.getElementById('fcap'))document.getElementById('fcap').value=String(s.fcap||0);
    document.getElementById('job').innerHTML=fmtJob(s.job);
    document.getElementById('origin').textContent=(s.origin||'-')+(s.origin_nvram&&s.origin_nvram!==s.origin?' (NVRAM '+s.origin_nvram+')':'');
    for(const b of document.querySelectorAll('button.mode'))if(b.textContent==='LOW LEFT'||b.textContent==='CENTER')b.classList.toggle('on',b.textContent===s.origin);
    zzero=s.z_zero||0;document.getElementById('zzero').textContent=zzero.toFixed(1);if(window.v3)v3.setZZero(zzero);
    if(s.job&&s.job.ref_id!==refId){refId=s.job.ref_id;
      fetch('/ref').then(r=>r.json()).then(r=>{ref=r.pts||[];if(window.v3)v3.setRef(ref);});}
    document.getElementById('lcd').textContent=(s.lcd||[]).map(r=>r.padEnd(16)).join('\n')||'...';
    let h='';for(const k of ['X','Y','Z']){const a=(s.axes||{})[k];if(!a)continue;
      h+='<tr><td>'+k+(a.home?' <span class="home">&#9679;</span>':'')+(a.hit?' <span style="color:#ff9b9b" title="limit switch hit / pulses discarded">&#9888; '+a.hit+'/'+a.clipped+'</span>':'')+'</td><td>'
        +a.mm.toFixed(3)+' mm <span style="color:var(--dim)">('+a.min+'..'+a.max+')</span></td></tr>';}
    document.getElementById('axes').innerHTML=h||'<tr><td>no physics</td><td></td></tr>';
    await fetchPath(s);draw();
  }catch(e){}
  setTimeout(tick,500);
}
tick();
</script>
<script type="importmap">{"imports":{"three":"/static/three.module.js","three/addons/":"/static/"}}</script>
<script type="module">
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/OrbitControls.js';
(function(){
const TABLE=[483,305],el=document.getElementById('v3');
const renderer=new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(Math.min(2,window.devicePixelRatio||1));
renderer.domElement.style.display='block';
el.appendChild(renderer.domElement);
const scene=new THREE.Scene();scene.background=new THREE.Color(0x0f1115);
const W=()=>Math.max(200,el.clientWidth),Hh=()=>Math.max(200,el.clientHeight);
// World: X to the right, Y to the back, Z up (program Z, negative = below Z0, times exaggeration)
const persp=new THREE.PerspectiveCamera(40,W()/Hh(),0.5,20000);persp.up.set(0,0,1);
const ortho=new THREE.OrthographicCamera(-1,1,1,-1,-20000,20000);ortho.up.set(0,0,1);
let cam=persp,controls=null;
function newControls(){if(controls)controls.dispose();controls=new OrbitControls(cam,renderer.domElement);
  controls.enableDamping=true;controls.dampingFactor=0.12;controls.screenSpacePanning=true;
  controls.mouseButtons={LEFT:THREE.MOUSE.ROTATE,MIDDLE:THREE.MOUSE.DOLLY,RIGHT:THREE.MOUSE.PAN};}
newControls();
// Table (0..483 x 0..305 mm from LOW LEFT), grid 10/50 mm, axes cross at the origin
const stat=new THREE.Group();scene.add(stat);
{const g=[],G=[];
 for(let x=0;x<=TABLE[0];x+=10)(x%50?g:G).push(x,0,0,x,TABLE[1],0);
 for(let y=0;y<=TABLE[1];y+=10)(y%50?g:G).push(0,y,0,TABLE[0],y,0);
 const mk=(arr,c)=>{const ge=new THREE.BufferGeometry();ge.setAttribute('position',new THREE.Float32BufferAttribute(arr,3));
   return new THREE.LineSegments(ge,new THREE.LineBasicMaterial({color:c}));};
 stat.add(mk(g,0x1d2230),mk(G,0x2b3446));
 const um=new THREE.BufferGeometry();um.setAttribute('position',new THREE.Float32BufferAttribute([0,0,0,TABLE[0],0,0,TABLE[0],TABLE[1],0,0,TABLE[1],0],3));
 const uml=new THREE.LineLoop(um,new THREE.LineDashedMaterial({color:0x5aa9e6,dashSize:6,gapSize:4,transparent:true,opacity:.6}));
 uml.computeLineDistances();stat.add(uml);stat.add(new THREE.AxesHelper(30));}
// Actual path: preallocated buffers, append only; colour as in 2D (shallow blue, deep red)
let cap=1<<16,n=0,raw=new Float32Array(cap*3),pos=new Float32Array(cap*3),col=new Float32Array(cap*3);
let geo=new THREE.BufferGeometry();
function bindGeo(){geo.setAttribute('position',new THREE.BufferAttribute(pos,3));geo.setAttribute('color',new THREE.BufferAttribute(col,3));geo.setDrawRange(0,n);}
bindGeo();
const line=new THREE.Line(geo,new THREE.LineBasicMaterial({vertexColors:true}));line.frustumCulled=false;scene.add(line);
const marker=new THREE.Mesh(new THREE.SphereGeometry(1.2,12,12),new THREE.MeshBasicMaterial({color:0xffd166}));marker.visible=false;scene.add(marker);
let zzero=0,zx=3,zlo=Infinity,zhi=-Infinity;
const zd=za=>-(za-zzero);
const tmp=new THREE.Color();
function colour(i,z){const t=(zhi-zlo)>1e-6?(zhi-z)/(zhi-zlo):0;tmp.setHSL((205-165*t)/360,.78,(38+26*t)/100);col[3*i]=tmp.r;col[3*i+1]=tmp.g;col[3*i+2]=tmp.b;}
function refill(from){for(let i=from;i<n;i++){const z=zd(raw[3*i+2]);pos[3*i]=raw[3*i];pos[3*i+1]=raw[3*i+1];pos[3*i+2]=z*zx;colour(i,z);}
  geo.attributes.position.needsUpdate=true;geo.attributes.color.needsUpdate=true;geo.setDrawRange(0,n);
  marker.visible=n>0;if(n)marker.position.set(pos[3*n-3],pos[3*n-2],pos[3*n-1]);}
function grow(need){while(cap<need)cap*=2;const r2=new Float32Array(cap*3);r2.set(raw.subarray(0,3*n));raw=r2;
  pos=new Float32Array(cap*3);col=new Float32Array(cap*3);geo.dispose();geo=new THREE.BufferGeometry();line.geometry=geo;bindGeo();refill(0);}
function append(f){const m=f.length/3|0;if(!m)return;if(n+m>cap)grow(n+m);raw.set(f.subarray(0,3*m),3*n);
  let lo=zlo,hi=zhi;for(let i=0;i<m;i++){const z=zd(f[3*i+2]);if(z<lo)lo=z;if(z>hi)hi=z;}
  const n0=n;n+=m;if(lo!==zlo||hi!==zhi){zlo=lo;zhi=hi;refill(0);}else refill(n0);}
function reset(){n=0;zlo=Infinity;zhi=-Infinity;geo.setDrawRange(0,0);marker.visible=false;}
// Reference path: cuts solid white, rapids dashed grey (as in 2D)
let refRaw=[];const refGrp=new THREE.Group();scene.add(refGrp);
function buildRef(){for(const c of refGrp.children){c.geometry.dispose();c.material.dispose();}refGrp.clear();
  if(refRaw.length<2)return;const cut=[],rap=[];
  for(let i=1;i<refRaw.length;i++){const a=refRaw[i-1],b=refRaw[i];(b[3]?cut:rap).push(a[0],a[1],zd(a[2])*zx,b[0],b[1],zd(b[2])*zx);}
  if(cut.length){const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(cut,3));
    refGrp.add(new THREE.LineSegments(g,new THREE.LineBasicMaterial({color:0xffffff,transparent:true,opacity:.55})));}
  if(rap.length){const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(rap,3));
    const l=new THREE.LineSegments(g,new THREE.LineDashedMaterial({color:0x8c96aa,dashSize:3,gapSize:3,transparent:true,opacity:.45}));l.computeLineDistances();refGrp.add(l);}
  refGrp.visible=document.getElementById('showref').checked;}
function setRef(pts){refRaw=pts||[];buildRef();view('fit');}
// Views
function bbox(){const b=new THREE.Box3(),v=new THREE.Vector3();
  if(n){const st=Math.max(1,(n/20000)|0);for(let i=0;i<n;i+=st)b.expandByPoint(v.set(pos[3*i],pos[3*i+1],pos[3*i+2]));b.expandByPoint(v.set(pos[3*n-3],pos[3*n-2],pos[3*n-1]));}
  for(const p of refRaw)b.expandByPoint(v.set(p[0],p[1],zd(p[2])*zx));
  if(b.isEmpty())b.set(new THREE.Vector3(0,0,0),new THREE.Vector3(TABLE[0],TABLE[1],0));return b;}
function view(kind){const b=bbox(),c=b.getCenter(new THREE.Vector3()),sz=b.getSize(new THREE.Vector3());
  const r=Math.max(sz.x,sz.y,sz.z,10)*0.6;marker.scale.setScalar(Math.max(0.05,r/60));
  let dir=kind==='top'?new THREE.Vector3(0,-0.02,1):new THREE.Vector3(-0.8,-0.9,0.7);
  if(kind==='fit'){dir=cam.position.clone().sub(controls.target);if(dir.length()<1e-6)dir.set(-0.8,-0.9,0.7);}
  dir.normalize();
  if(cam===persp){const d=r/Math.tan(THREE.MathUtils.degToRad(persp.fov/2))*1.15;persp.position.copy(c).addScaledVector(dir,d);}
  else{const asp=W()/Hh();ortho.left=-r*1.15*asp;ortho.right=r*1.15*asp;ortho.top=r*1.15;ortho.bottom=-r*1.15;ortho.zoom=1;
    ortho.updateProjectionMatrix();ortho.position.copy(c).addScaledVector(dir,r*4+1000);}
  controls.target.copy(c);cam.lookAt(c);controls.update();}
function setOrtho(on){const t=controls.target.clone(),d=cam.position.clone().sub(t);cam=on?ortho:persp;cam.position.copy(t).add(d);
  newControls();controls.target.copy(t);view('fit');}
function resize(){const w=W(),h=Hh();renderer.setSize(w,h,false);renderer.domElement.style.width='100%';renderer.domElement.style.height='100%';
  persp.aspect=w/h;persp.updateProjectionMatrix();const asp=w/h,half=(ortho.top-ortho.bottom)/2;ortho.left=-half*asp;ortho.right=half*asp;ortho.updateProjectionMatrix();}
new ResizeObserver(resize).observe(el);resize();
let active=true;
function loop(){requestAnimationFrame(loop);if(!active||el.offsetParent===null)return;controls.update();renderer.render(scene,cam);}
loop();
document.getElementById('zx').addEventListener('input',e=>{zx=+e.target.value;document.getElementById('zxv').textContent=zx;refill(0);buildRef();});
document.getElementById('ortho').addEventListener('change',e=>setOrtho(e.target.checked));
document.getElementById('showref').addEventListener('change',e=>{refGrp.visible=e.target.checked;});
document.getElementById('grid').addEventListener('change',e=>{stat.visible=e.target.checked;});
window.v3={append,reset,setRef,view,setZZero(z){if(z!==zzero){zzero=z;refill(0);buildRef();}},setActive(a){active=a;},info(){return {n,zlo,zhi};}};
view('iso');
})();
</script></body></html>"""


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, body, ctype="application/json"):
        b = body.encode("utf-8") if isinstance(body, str) else body
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        if self.path.startswith("/state"):
            with EMU.lock:
                self._send(json.dumps(EMU.state()))
        elif self.path.startswith("/ref"):
            self._send(json.dumps({"id": EMU.ref_id, "pts": EMU.ref}))
        elif self.path.startswith("/path"):
            # Actual path incrementally: header <III> = generation, from, total count, then float32 triples
            # (x, y, z_axis in mm) from `from`. If the generation does not match (clear, decimation),
            # the path comes from the start; at most 200 000 points per response.
            q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            frm = int(q.get("from", ["0"])[0]); gen = int(q.get("gen", ["-1"])[0])
            with EMU.lock:
                pg = EMU.path_gen; pth = EMU.path
                if gen != pg or frm > len(pth):
                    frm = 0
                chunk = pth[frm:frm + 200000]; total = len(pth)
            a = array.array("f")
            for x, y, z in chunk:
                a.append(x); a.append(y); a.append(z)
            self._send(struct.pack("<III", pg, frm, total) + a.tobytes(), "application/octet-stream")
        elif self.path.startswith("/static/"):
            name = os.path.basename(urllib.parse.urlparse(self.path).path)
            f = os.path.join(STATIC, name)
            if not os.path.isfile(f):
                self.send_response(404); self.end_headers(); return
            ctype = "application/javascript; charset=utf-8" if name.endswith(".js") else "text/plain; charset=utf-8"
            self._send(open(f, "rb").read(), ctype)
        else:
            self._send(PAGE, "text/html; charset=utf-8")

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        try:
            arg = json.loads(self.rfile.read(n) or b"{}")
        except Exception:
            arg = {}
        kind = self.path.strip("/")
        EMU.cmds.put((kind, arg))
        self._send("{}")


def main(argv=None):
    global EMU
    ap = argparse.ArgumentParser(prog="me500emu ui")
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--no-physics", action="store_true")
    ap.add_argument("--nvram", default=NVRAM,
                    help="NVRAM file (512 bytes); '' disables persistence")
    ap.add_argument("--no-snapshot", action="store_true",
                    help="real homing run instead of the snapshot (takes ~90 s)")
    ap.add_argument("--rom", default=None,
                    help="'stock', 'max' or a file path (default: the 1.50MAX image if one is found, else stock)")
    a = ap.parse_args(argv)
    try:
        if a.rom:
            rom = paths.rom_path(a.rom)
        else:
            try:
                rom = paths.rom_path("max")
            except paths.RomNotFound:
                rom = paths.rom_path("stock")
    except paths.RomNotFound as e:
        sys.exit(str(e))
    EMU = Emu(physics=not a.no_physics, nvram=a.nvram or None,
              snapshot=not a.no_snapshot, rom=rom)
    th = threading.Thread(target=EMU.run, daemon=True)
    th.start()

    def _term(_sig, _frm):              # same orderly path as Ctrl-C
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), H)
    print("ME-500 emulator UI:  http://127.0.0.1:%d   ROM %s" % (a.port, os.path.basename(rom)))
    if a.nvram:
        print("NVRAM: %s%s" % (a.nvram,
              "" if os.path.exists(a.nvram) else "  (created on exit)"))
    if a.no_snapshot:
        print("The homing run takes a few minutes; "
              "the State field at the top shows the progress.")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down ...")
    finally:
        shutdown(srv, th)


def shutdown(srv, th):
    """Ordered teardown - the reason Ctrl-C used to dump core.

    The emulator thread is inside Unicorn. Stopping the interpreter while a C
    frame holds Python hook callbacks is a segfault, so: stop serving, ask the
    machine to leave the engine, wait briefly, persist the NVRAM, and leave
    through os._exit so no finaliser can run into a still-live Unicorn object.
    """
    try:
        srv.shutdown()
    except Exception:
        pass
    srv.server_close()
    EMU.request_stop()
    th.join(timeout=10)
    if th.is_alive():
        print("Emulator thread does not respond - exiting hard.")
    elif EMU.nvram:
        if EMU.save_nvram():
            print("NVRAM saved: %s" % EMU.nvram)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
