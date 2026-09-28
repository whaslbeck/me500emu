"""Reference geometry of a program for the emulator UI: HP-GL or G-code -> the sequence
of target points as the firmware should build them, in table steps (5 um), and from
that the nominal path in machine mm. This lets an existing program with known geometry
be laid over the path actually driven (UI: "program from file").

Units (firmware analysis; the 1.50MAX G-code branch):
  HP-GL   MGL-IIc-10: 1 unit = 0.01 mm = 2 table steps; absolute coordinates
          add the plotter origin [0xb500]/[0xb504] (48300/30500 with ORIGIN CENTER).
          PD plunges to the ZD depth [0x08ce] (table steps, positive = below zero),
          PU lifts to Z 0; ZQ x,y,z with z in units, negative = below zero.
  G-code  mm; X/Y absolute plus origin, Z negative = below zero -> table positive;
          G90/G91 as in the branch, unknown words stop the machine.
          G2/G3 as in 1.50MAX (build >= 000014): modal, X Y end point (G90/G91 like G1), I J centre
          relative to the start point, start = end is a full circle, Z makes a helix (Z linear with
          the angle); XY plane only. The branch stops on the R form, on an arc after G18/G19, on start
          and end radius differing by more than 0.02 mm, on a radius under 0.01 mm and on a helix
          pitch over 65 mm - the reference stops there too. The reference cuts the exact arc into
          chords of at most ARC_TOL_MM deviation (the firmware uses 0.01 mm, so its chord error shows up
          in the measured deviation).
  Machine: table steps count absolutely from the lower left table corner (LOW LEFT), the
          rest position after homing; axis steps = origin of the axis (peer.ORIGIN_MARGIN_MM,
          23 / 8 mm from the lower end stop) + (10, 10, 20) * table value, mm = X/2000, Y/2000, Z/4000.
          The UI computes everything in table coordinates (0 = LOW LEFT) and before the job drives
          IN;SP1;PA0,0 (with CENTER: to the table centre).
"""
import math
import re

STEPS_PER_UNIT = 2          # HP-GL 0.01 mm -> 5 um steps
STEPS_PER_MM = 200

G_ALLOWED = {0, 1, 2, 3, 90, 91, 21, 4, 17, 18, 19, 40, 49, 54, 55, 56, 57, 58, 59, 61, 64, 80, 94}


def detect_language(text):
    """'gcode' or 'hpgl' - whichever dominates the first lines."""
    t = text[:6000].upper()
    g = len(re.findall(r'(?m)^\s*(?:N\d+\s*)?[GM%(]', t)) + len(re.findall(r'\bG0?[01]\b', t))
    h = len(re.findall(r'\bP[ADUR]\b|IN;|SP\d|ZQ', t))
    return "gcode" if g > h else "hpgl"


class Reference(object):
    """points: list of (tX, tY, tZ, cutting) in table steps; the first point is the start.
    notes: what the reference does not model (ignored commands, expected stop).
    ignored: {command: count}."""

    def __init__(self, start):
        self.points = [(start[0], start[1], start[2], False)]
        self.feeds = [None]            # mm/s per point (G-code: F; HP-GL: None = the machine's XY-ES)
        self.notes = []
        self.ignored = {}
        self._feed = None
        self.arcs = 0                  # G2/G3 lines cut into points

    def move_to(self, x, y, z, cutting):
        px, py, pz, _ = self.points[-1]
        if (x, y, z) != (px, py, pz):
            self.points.append((int(x), int(y), int(z), bool(cutting)))
            self.feeds.append(self._feed)

    def ramp_faults(self, xyes_mm_s):
        """Segments that hit the ramp fault of the factory firmware (8000:366a, user guide
        section 6): axis-parallel, 16.4..25.5 mm, feed above 40 mm/s. Harmless on the machine
        (measured 2026-09-05); in the emulator the record does not end (model limit).
        Returns the list of segment indices."""
        out = []
        for i, (a, b) in enumerate(zip(self.points, self.points[1:]), 1):
            dx, dy = abs(b[0] - a[0]), abs(b[1] - a[1])
            if (dx == 0) != (dy == 0) and b[3]:
                l = max(dx, dy)
                v = self.feeds[i] if self.feeds[i] is not None else xyes_mm_s
                if 3275 <= l <= 5100 and v is not None and v > 40.0:
                    out.append(i)
        return out

    @property
    def cut_length_mm(self):
        s = 0.0
        for a, b in zip(self.points, self.points[1:]):
            if b[3] and a[3] is not None:
                s += ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5
        return s / STEPS_PER_MM

    def finish(self):
        for k, n in sorted(self.ignored.items()):
            self.notes.append("%s ignored (%d)" % (k, n))
        return self

    def bounds(self, table_x, table_y, zlim=5000):
        """Notes on where the program leaves the firmware's table (STOP, B2; factory ROM: offscale)
        - in table coordinates mm, with the advice to choose ORIGIN LOW LEFT instead of CENTER."""
        pts = [q for q in self.points[1:] if q[3] is not None]
        if not pts:
            return []
        xs = [q[0] for q in pts]; ys = [q[1] for q in pts]; zs = [q[2] for q in pts]
        h = []
        mm = lambda t: t / float(STEPS_PER_MM)
        for name, vs, lim in (("X", xs, table_x), ("Y", ys, table_y)):
            if min(vs) < 0 or max(vs) > lim:
                advice = " - ORIGIN LOW LEFT instead of CENTER?" if max(vs) > lim and min(vs) >= 0 else (
                      " - move the origin inwards by the tool radius (UI: X,Y mm)" if min(vs) < 0 and min(vs) > -1000 else "")
                h.append("%s %.1f..%.1f mm leaves the table 0..%.0f: STOP (limit B2) at the first record outside%s"
                         % (name, mm(min(vs)), mm(max(vs)), mm(lim), advice))
        if min(zs) < -zlim or max(zs) > zlim:
            h.append("Z %.1f..%.1f mm beyond the limit +-%.0f mm: STOP" % (-mm(max(zs)), -mm(min(zs)), mm(zlim)))
        if min(zs) < 0:
            h.append("program moves up to %.1f mm above the Z zero point - set Z0 low enough in the emulator (header line)" % -mm(min(zs)))
        return h


def _numbers(s):
    return [float(x) for x in re.findall(r'[-+]?(?:\d+\.?\d*|\.\d+)', s)]


def reference_hpgl(text, origin, start, zd):
    """origin = (tX, tY) of the plotter origin, start = (tX, tY, tZ) of the machine, zd = PD depth."""
    r = Reference(start)
    x, y, z = start
    absolute = True
    down = False
    depth = zd
    for m in re.finditer(r'([A-Z][A-Z]|![A-Z][A-Z])\s*([^A-Z!;]*)', text.upper()):
        cmd, par = m.group(1), m.group(2)
        n = _numbers(par)
        if cmd == "IN":
            absolute = True             # IN resets mode and pen, NOT the firmware's position
            down = False                # table (emulator, 2026-09-07: the table survives IN;)
        elif cmd in ("PA", "PR", "PU", "PD"):
            if cmd == "PA":
                absolute = True         # PA/PR with coordinates are moves themselves, in
            if cmd == "PR":             # the current pen state
                absolute = False
            if cmd == "PD" and (not down or z != depth):
                # plunge to the depth; a `PD;` with a changed depth (`!PZ-1000;PD;`,
                # Estlcam multi-pass step-down) moves Z in place (firmware analysis)
                z = depth
                r.move_to(x, y, z, True)
                down = True
            if cmd == "PU" and down:
                z = 0
                r.move_to(x, y, z, False)
                down = False
            for i in range(0, len(n) - 1, 2):
                if absolute:
                    x, y = origin[0] + n[i] * STEPS_PER_UNIT, origin[1] + n[i + 1] * STEPS_PER_UNIT
                else:
                    x, y = x + n[i] * STEPS_PER_UNIT, y + n[i + 1] * STEPS_PER_UNIT
                r.move_to(x, y, z, down)
        elif cmd == "ZQ":
            for i in range(0, len(n) - 2, 3):
                x, y = origin[0] + n[i] * STEPS_PER_UNIT, origin[1] + n[i + 1] * STEPS_PER_UNIT
                z = -n[i + 2] * STEPS_PER_UNIT
                depth = z
                down = z > 0
                r.move_to(x, y, z, True)
        elif cmd == "!PZ":
            # depth command: sets the PD depth (cells 0x08ce/0x09c0, 0.01 mm, negative = below
            # zero); a positive value (above zero) is rejected with ERR11 PARAMETER and
            # changes nothing (firmware analysis) - Estlcam writes `!PZ500;` before every `PU`.
            if n and n[0] < 0:
                depth = -n[0] * STEPS_PER_UNIT
            elif n:
                r.ignored["!PZ>0 (ERR11 PARAMETER, depth kept)"] = r.ignored.get("!PZ>0 (ERR11 PARAMETER, depth kept)", 0) + 1
        elif cmd in ("SP", "VS", "SC", "IP", "IW", "DF", "PG", "NR", "LT", "PT", "PW", "AP"):
            pass
        else:
            r.ignored[cmd] = r.ignored.get(cmd, 0) + 1
    return r.finish()


ARC_TOL_MM = 0.002           # chord deviation of the reference arc (well below the firmware's 0.01 mm)
ARC_RADIUS_MISMATCH_MM = 0.02
ARC_MIN_RADIUS_MM = 0.01
ARC_MAX_PITCH_MM = 65.0


def _arc_points(sx, sy, ex, ey, cx, cy, clockwise, tol=ARC_TOL_MM):
    """Points (x, y, fraction) along an arc in mm, excluding the start, including the end.
    start == end is a full circle. Returns (points, radius, sweep angle)."""
    r = math.hypot(sx - cx, sy - cy)
    a0 = math.atan2(sy - cy, sx - cx)
    a1 = math.atan2(ey - cy, ex - cx)
    sweep = (a0 - a1) if clockwise else (a1 - a0)
    sweep %= 2 * math.pi
    if sweep < 1e-9:
        sweep = 2 * math.pi
    step = 2 * math.acos(max(-1.0, 1 - tol / r)) if r > tol else math.pi / 2
    n = max(1, int(math.ceil(sweep / step)))
    pts = []
    for i in range(1, n + 1):
        f = i / float(n)
        a = a0 - f * sweep if clockwise else a0 + f * sweep
        pts.append((cx + r * math.cos(a), cy + r * math.sin(a), f))
    pts[-1] = (ex, ey, 1.0)
    return pts, r, sweep


def reference_gcode(text, origin, start):
    r = Reference(start)
    x, y, z = start
    px, py = float(x), float(y)
    pz_mm = -z / float(STEPS_PER_MM)         # programmed Z position in mm
    absolute = True
    plane = 17
    motion = None                            # modal motion: 0 rapid, 1 feed, 2/3 arc
    for nr, line in enumerate(text.splitlines(), 1):
        s = re.sub(r'\(.*?\)', ' ', line).split(';')[0].strip().upper()
        if not s or s.startswith('%'):
            continue
        words = re.findall(r'([A-Z])\s*([-+]?(?:\d+\.?\d*|\.\d+))', s)
        gs = [int(round(float(v))) for k, v in words if k == 'G']
        for g in gs:
            if g not in G_ALLOWED:
                r.notes.append("stop expected: line %d (%s)" % (nr, s[:24]))
                return r.finish()
        for k, v in words:
            if k not in "GXYZIJFMNSTOHDPLQR":
                r.notes.append("stop expected: line %d (word %s)" % (nr, k))
                return r.finish()
        if 90 in gs:
            absolute = True
        if 91 in gs:
            absolute = False
        for g in (17, 18, 19):
            if g in gs:
                plane = g
        for g in (0, 1, 2, 3):
            if g in gs:
                motion = g
        for k, v in words:
            if k == 'F':
                r._feed = min(50.0, max(0.5, float(v) / 60.0))   # mm/min -> mm/s, capped like the branch
        d = {k: float(v) for k, v in words if k in "XYZIJR"}
        is_arc = motion in (2, 3) and any(k in d for k in "XYZIJR")
        if is_arc:
            why = None
            if 'R' in d:
                why = "arc in R form"
            elif plane != 17:
                why = "arc outside the XY plane (G%d)" % plane
            if why:
                r.notes.append("stop expected: line %d (%s)" % (nr, why))
                return r.finish()
        d = {k: v for k, v in d.items() if k in "XYZIJ"}
        if not any(k in d for k in "XYZ") and not is_arc:
            continue
        sx, sy, sz_mm = px, py, pz_mm
        if 'X' in d:
            px = (origin[0] + d['X'] * STEPS_PER_MM) if absolute else px + d['X'] * STEPS_PER_MM
        if 'Y' in d:
            py = (origin[1] + d['Y'] * STEPS_PER_MM) if absolute else py + d['Y'] * STEPS_PER_MM
        if 'Z' in d:
            pz_mm = d['Z'] if absolute else pz_mm + d['Z']
        z = -pz_mm * STEPS_PER_MM
        cutting = motion != 0                # modal: G0 stays rapid until G1/G2/G3 comes
        if is_arc:
            m = float(STEPS_PER_MM)
            cx, cy = sx / m + d.get('I', 0.0), sy / m + d.get('J', 0.0)
            ex, ey = px / m, py / m
            r0 = math.hypot(sx / m - cx, sy / m - cy)
            r1 = math.hypot(ex - cx, ey - cy)
            pts, rad, sweep = _arc_points(sx / m, sy / m, ex, ey, cx, cy, motion == 2)
            why = None
            if min(r0, r1) < ARC_MIN_RADIUS_MM:
                why = "arc radius under %.2f mm" % ARC_MIN_RADIUS_MM
            elif abs(r0 - r1) > ARC_RADIUS_MISMATCH_MM:
                why = "arc start/end radius differ by %.3f mm" % abs(r0 - r1)
            elif pz_mm != sz_mm and abs(pz_mm - sz_mm) * 2 * math.pi / sweep > ARC_MAX_PITCH_MM:
                why = "helix pitch over %g mm" % ARC_MAX_PITCH_MM
            if why:
                r.notes.append("stop expected: line %d (%s)" % (nr, why))
                return r.finish()
            r.arcs += 1
            for ax_, ay_, f in pts:
                zz = -(sz_mm + f * (pz_mm - sz_mm)) * STEPS_PER_MM
                r.move_to(round(ax_ * m), round(ay_ * m), round(zz), True)
            continue
        r.move_to(round(px), round(py), round(z), cutting)
    return r.finish()


def reference(text, fmt, origin, start, zd):
    return reference_hpgl(text, origin, start, zd) if fmt == "hpgl" else reference_gcode(text, origin, start)


def distance_2d(p, a, b):
    """Distance of point p from the segment a-b, all in mm, X/Y only."""
    ax, ay, bx, by = a[0], a[1], b[0], b[1]
    dx, dy = bx - ax, by - ay
    l2 = dx * dx + dy * dy
    if l2 <= 1e-12:
        return ((p[0] - ax) ** 2 + (p[1] - ay) ** 2) ** 0.5
    t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / l2
    t = 0.0 if t < 0 else (1.0 if t > 1 else t)
    qx, qy = ax + t * dx, ay + t * dy
    return ((p[0] - qx) ** 2 + (p[1] - qy) ** 2) ** 0.5
