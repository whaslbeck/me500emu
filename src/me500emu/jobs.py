"""Run a whole job headless and report what the machine did - the CAM test bench.

    from me500emu.session import Session
    from me500emu.jobs import run_job
    report = run_job(Session(rom="max", mode="gcode"), open("part.nc").read())

The report holds the emulated machine time, the travelled bounds per axis, the final position, the firmware's own
status (strobes, halt, overruns, the 1.50MAX G-code counters) and the LCD. Positions are sampled on every motion
strobe (about one per millisecond of machine time), so bounds are exact to the step, not to a sampling interval.

Work coordinates: X/Y relative to the firmware's XY origin (plotter origin cells [0xb500]/[0xb504]; with ORIGIN
LOW LEFT and no offset this is the table corner the machine homes to), Z relative to Z0 with the G-code sign
convention (negative = below the surface). HP-GL coordinates in the MGL-IIc 10 um setting are 0.01 mm, so an
HP-GL job's work X/Y in mm are its plotter units / 100.
"""
import time

from . import session as S

CUT_EPS_MM = 0.01          # below Z0 by more than this counts as cutting (encoder rounding is ~0.001 mm)


class PathRecorder(object):
    """Physical axis positions at every strobe, from the fast core's path buffer.

    The buffer holds the COMMANDED position (the running sum of the motion window). The axes stop at their end
    switches, so the physical position is rebuilt from the commanded differences with the same clipping the axis
    model applies (0 .. travel)."""

    def __init__(self, sess, offset=(0.0, 0.0, 0.0)):
        """offset: (x0, y0, z0) table mm of the work origin; work = (x - x0, y - y0, z0 - z)."""
        self.s = sess
        self.off = offset
        m = sess.m
        self.fc = m.fc
        self.fc.motion.path_n = 0
        a, q = m.subcpu.axes, m.motion.pos
        self.axes = [a["X"], a["Y"], a["Z"]]
        self.pp = [ax.pos for ax in self.axes]
        self.lim = [ax.travel_steps for ax in self.axes]
        self.last = [q["X"], q["Y"], q["Z"]]
        self.points = []           # (instr, x, y, z) work coordinates in mm, only when the position changed
        self.lo = [None] * 3       # bounds of all motion, work coordinates
        self.hi = [None] * 3
        self.cut_lo = [None] * 3   # bounds while the tool is below Z0 (z < 0)
        self.cut_hi = [None] * 3
        self.last_change = m.instr

    def mm(self, p):
        """Axis steps -> work coordinates in mm."""
        t = [(p[i] - self.axes[i].origin_steps) / self.axes[i].STEPS_PER_MM for i in range(3)]
        return (round(t[0] - self.off[0], 4), round(t[1] - self.off[1], 4), round(self.off[2] - t[2], 4))

    @staticmethod
    def _extend(lo, hi, p):
        for i in range(3):
            if lo[i] is None or p[i] < lo[i]:
                lo[i] = p[i]
            if hi[i] is None or p[i] > hi[i]:
                hi[i] = p[i]

    def harvest(self, keep_points=True):
        fc = self.fc
        n = fc.motion.path_n
        if not n:
            return
        arr = fc.path
        pp, last, lim = self.pp, self.last, self.lim
        t = self.s.m.instr
        for k in range(n):
            changed = False
            for i in range(3):
                q = arr[3 * k + i]
                v = pp[i] + (q - last[i])
                v = 0 if v < 0 else (lim[i] if v > lim[i] else v)
                if v != pp[i]:
                    changed = True
                pp[i] = v
                last[i] = q
            if changed:
                p = self.mm(pp)
                self._extend(self.lo, self.hi, p)
                if p[2] < -CUT_EPS_MM:
                    self._extend(self.cut_lo, self.cut_hi, p)
                self.last_change = t
                if keep_points:
                    self.points.append((t,) + p)
        fc.motion.path_n = 0


def run_job(sess, text, timeout_s=3600.0, keep_points=False, progress=None):
    """Stream `text` into the session's machine and run until it is idle again.

    timeout_s is EMULATED time. Returns a JSON-serialisable report; `points` (instr, x, y, z) is included when
    keep_points is set (for a trace CSV)."""
    m = sess.m
    data = text.encode("latin-1") if isinstance(text, str) else text
    z0 = sess.z0_mm()
    ox, oy = sess.xy_origin_mm()
    rec = PathRecorder(sess, (ox, oy, z0))
    start = m.instr
    wall = time.time()
    limit = start + int(timeout_s * S.INSTR_PER_S)
    pos = 0
    sess.read_tx()
    tx = bytearray()
    still, prev = 0, None
    timed_out = False
    while True:
        if pos < len(data):
            pos += sess.feed(data[pos:])
        m.run(m.instr + (20000 if pos < len(data) or len(m.uart.rx) else 400000))
        rec.harvest(keep_points)
        tx += sess.read_tx()
        if progress:
            progress(pos, len(data), m.instr - start)
        if sess.halted:
            break
        if m.instr > limit:
            timed_out = True
            break
        if pos >= len(data) and not len(m.uart.rx):
            now = tuple(a.pos for a in rec.axes)
            still = still + 1 if now == prev and sess.ring_empty() else 0
            prev = now
            # idle: all bytes delivered, executor ring empty, axes still for 6 x 400k instructions (1.2 s)
            if still >= 6:
                break
    end_motion = rec.last_change
    x, y, z = sess.position_mm()

    def bounds(lo, hi):
        return dict(x=[lo[0], hi[0]], y=[lo[1], hi[1]], z=[lo[2], hi[2]])
    report = dict(
        rom=sess.version, mode=sess.mode, baud=sess.baud, handshake=sess.handshake,
        bytes_sent=pos, bytes_total=len(data),
        machine_time_s=round((end_motion - start) / float(S.INSTR_PER_S), 3),
        wall_time_s=round(time.time() - wall, 1),
        timed_out=timed_out,
        bounds_mm=bounds(rec.lo, rec.hi),
        cut_bounds_mm=bounds(rec.cut_lo, rec.cut_hi),
        final_mm=dict(x=round(x - ox, 3), y=round(y - oy, 3), z=round(z0 - z, 3)),
        work_origin_table_mm=dict(x=ox, y=oy, z=z0),
        status=sess.status(),
        lcd=[r.rstrip() for r in m.panel.text()],
        machine_output=tx.decode("latin-1"),
    )
    if keep_points:
        report["points"] = [(round((t - start) / float(S.INSTR_PER_S), 4), px, py, pz) for t, px, py, pz in rec.points]
    return report
