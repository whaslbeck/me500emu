"""Common scaffolding for the measurement scripts.

`drain`, the REMOTE key tap and the approach move used to be copied verbatim into
twelve scripts - each one a chance to change a constant without noticing. Here they
exist once.

The shape follows what the measurements actually need:

    m = ready()                     snapshot, REMOTE, approach move done
    n = strobes(m, job)             send a job and run until standstill
    with counting(m, {0x85a29: "chain"}) as c:   instruction counters at addresses

`drain` is the most important part and the reason for this module: a fixed time
window is not enough. A long PA approach needs more than 26 million instructions,
and its remainder then ended up in the next measurement - that once turned a
20 000-step move into one with 28 140. So it waits until the axes have stood still
for several chunks.
"""
import collections
import contextlib

import unicorn
from unicorn.x86_const import UC_X86_REG_CS, UC_X86_REG_IP

from . import snapshot_state as snap

ROM = None                      # None: paths.rom_path("stock"); callers may set a path
UM_PER_UNIT = 10.0              # MGL-IIc-10 (firmware analysis)
STEPS_PER_MM = 2000
APPROACH = b"IN;SP1;PA0,0;"


def drain(m, settle=20000000, chunk=4000000, quiet=3, limit=300000000):
    """Run until the axes have stood still for `quiet` chunks.

    Returns False if they do not within `limit` - the caller should check that
    and not silently carry on.
    """
    still, last, end = 0, None, m.instr + settle
    while m.instr < end or still < quiet:
        m.run(m.instr + chunk)
        now = tuple(a.pos for a in m.subcpu.axes.values())
        still = still + 1 if now == last else 0
        last = now
        if m.instr > end + limit:
            return False
    return True


# Key press in the model: the firmware debounces and repeats keys in timer ticks
# (INT 23h, 8000:0a02 scans the matrix); since the calibration the timer runs every 20340 instructions
# (10.17 ms). 18 ticks = 183 ms press (previously 900000 instructions at 50000 per tick = the same 18 ticks).
KEY_HOLD = 18 * 20340
KEY_TAP = 12 * 20340
KEY_GAP = 1800000


def remote(m, tries=3):
    """Tap into the REMOTE screen; without it the machine accepts no job."""
    for _ in range(tries):
        if "[REMOTE]" in " ".join(m.panel.text()):
            return True
        m.panel.press(1, 3)
        m.run(m.instr + KEY_HOLD)
        m.panel.release_all()
        m.run(m.instr + KEY_GAP)
    return "[REMOTE]" in " ".join(m.panel.text())


def ready(approach=APPROACH, **kw):
    """Machine from the snapshot, in REMOTE, approach move done and settled."""
    from . import paths
    m = snap.booted(paths.rom_path("stock", ROM), physics=True, trace="fast", **kw)
    if not remote(m):
        raise RuntimeError("REMOTE not reached: %s"
                           % " | ".join(r.rstrip() for r in m.panel.text()))
    if approach:
        m.uart.rx.extend(approach)
        if not drain(m, 30000000):
            raise RuntimeError("approach move does not settle")
    return m


Z0_BELOW_TOP = 2000      # test-bench Z0: 10 mm below the top position (5 um steps)


def set_z0_surface(m, z_steps=Z0_BELOW_TOP):
    """Set Z0 as the workpiece surface, the way the panel zero-point key does: [0x04b6] (32 bit, 5 um from the top),
    the table stays absolute. Earlier test gates instead poked the table cells [0x0fc8]/[0x0fd4]/[0x193e]/
    [0x1f60] to -2000 (surface = table zero) and therefore missed the Z zero-point bug; the real key path
    goes through the panel keys. Call before switching to GCODE (entering it puts ZSAFE on Z0)."""
    m.cpu.uc.mem_write(0x04B6, int(z_steps).to_bytes(4, "little", signed=True))


def units(mm):
    return int(round(mm * 1000.0 / UM_PER_UNIT))


def steps(mm):
    return int(round(mm * STEPS_PER_MM))


def line(n, mm, axis="X", sign=-1):
    """n chords of equal length on one axis; -X has room after homing."""
    u = units(mm)
    out = []
    for i in range(n):
        d = sign * u * (i + 1)
        out.append("PD%d,0;" % d if axis == "X" else "PD0,%d;" % d)
    return "".join(out) + "PU;"


@contextlib.contextmanager
def counting(m, sites):
    """sites: {address: name} -> Counter that counts only inside the with block."""
    c = collections.Counter()
    on = [False]

    def mk(name):
        def h(uc, addr, size, ud):
            if on[0]:
                c[name] += 1
        return h
    for addr, name in sites.items():
        m.cpu.uc.hook_add(unicorn.UC_HOOK_CODE, mk(name), None, addr, addr)
    on[0] = True
    try:
        yield c
    finally:
        on[0] = False


@contextlib.contextmanager
def watching(m, lo, hi, out):
    """Log write accesses to [lo, hi] into `out`: (address, size, value, pc)."""
    def hw(uc, access, address, size, value, ud):
        pc = uc.reg_read(UC_X86_REG_CS) * 0x10 + uc.reg_read(UC_X86_REG_IP)
        out.append((address, size, value, pc))
    m.cpu.uc.hook_add(unicorn.UC_HOOK_MEM_WRITE, hw, None, lo, hi)
    yield out


def moving_strobes(m, job, settle=30000000):
    """Send a job, let it settle, and count the strobes that move something."""
    marks = []
    mr, orig = m.motion, m.motion.write
    last = [None]

    def w(off, size, value, pc):
        if off == mr.STROBE and (value & 0xFF) == 1:
            p = tuple(a.pos for a in m.subcpu.axes.values())
            if last[0] is not None and p != last[0]:
                marks.append(1)
            last[0] = p
        return orig(off, size, value, pc)
    mr.write = w
    before = {k: a.pos for k, a in m.subcpu.axes.items()}
    m.uart.rx.extend(job.encode() if isinstance(job, str) else job)
    ok = drain(m, settle)
    mr.write = orig
    delta = {k: m.subcpu.axes[k].pos - before[k] for k in before}
    return len(marks), delta, ok
