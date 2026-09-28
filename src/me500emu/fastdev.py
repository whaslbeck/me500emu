"""Views onto the stage-2 C models (fastcore.c): motion window, counter, axes.
Same attributes and methods as devices.MotionRegs, devices.Counter, peer.Axis
and peer.SubCpu - scripts read `m.motion.pos["X"]`, hook `m.motion.write`, walk
`m.subcpu.axes[...]`, and the snapshot writes `m.motion.regs[:]`. The state
lives in the C struct; Python computes nothing here itself, it only passes
through.
"""
import collections.abc

from . import peer as _peer
from . import devices as _dev


class PosView(collections.abc.MutableMapping):
    KEYS = ("X", "Y", "Z")

    def __init__(self, arr):
        self._a = arr

    def __getitem__(self, k):
        return int(self._a[self.KEYS.index(k)])

    def __setitem__(self, k, v):
        self._a[self.KEYS.index(k)] = int(v)

    def __delitem__(self, k):
        raise TypeError("fixed axes")

    def __iter__(self):
        return iter(self.KEYS)

    def __len__(self):
        return 3

    def __repr__(self):
        return repr(dict(self))


def _cprop(name, cast=int):
    def get(self):
        return cast(getattr(self._c, name))

    def set_(self, v):
        setattr(self._c, name, v)
    return property(get, set_)


class AxisC(_peer.Axis):
    """peer.Axis over the C struct. The fields are properties; the physics
    (owe/advance) calls the core, so that script and slice loop compute the same."""
    pos = _cprop("pos")
    pending = _cprop("pending")
    travel_steps = _cprop("travel")
    STEPS_PER_MM = _cprop("steps_per_mm", float)
    margin_steps = _cprop("margin")
    home_at_low = _cprop("home_at_low", bool)
    both_ends = _cprop("both_ends", bool)
    clipped = _cprop("clipped")
    hit_low = _cprop("hit_low")
    hit_high = _cprop("hit_high")
    max_command = _cprop("max_command")
    follow_error_max = _cprop("follow_error_max")
    follow_trips = _cprop("follow_trips")

    @property
    def sensor_band(self):
        return (int(self._c.band_lo), int(self._c.band_hi)) if self._c.has_band else None

    @sensor_band.setter
    def sensor_band(self, v):
        if v is None:
            self._c.has_band = 0
        else:
            self._c.has_band = 1
            self._c.band_lo, self._c.band_hi = int(v[0]), int(v[1])

    def __init__(self, core, index, cstruct, *a, **k):
        self._core, self._i, self._c = core, index, cstruct
        self._c.steps_per_mm = _peer.Axis.STEPS_PER_MM
        _peer.Axis.__init__(self, *a, **k)
        self._c.follow_limit = self.follow_limit

    def owe(self, steps):
        self._core.L.fc_axis_owe(self._i, int(steps))

    def advance(self, max_steps):
        return int(self._core.L.fc_axis_advance(self._i, int(max_steps)))


class SubCpuC(_peer.SubCpu):
    """peer.SubCpu with axes in C. `consume_step` and `advance` go into the core;
    whoever hooks them is served by the core via the callback (consume_py)."""

    def __init__(self, machine, core, motion, enable_physics=True, start_distance_mm=10.0):
        self._core, self._mo = core, motion
        motion.instr_per_step = _peer.SubCpu.INSTR_PER_STEP
        _peer.SubCpu.__init__(self, machine, enable_physics=enable_physics,
                              start_distance_mm=start_distance_mm)
        # The parent constructor creates Python axes; same parameters, but in C.
        axes = {}
        for i, name in enumerate(("X", "Y", "Z")):
            a = self.axes[name]
            c = AxisC(core, i, motion.axes[i], name, a.travel_steps / a.STEPS_PER_MM,
                      home_at_low=a.home_at_low, steps_per_mm=a.STEPS_PER_MM,
                      margin_steps=a.margin_steps, both_ends=a.both_ends, sensor_band=a.sensor_band)
            c.pos, c.pending, c.clipped = a.pos, a.pending, a.clipped
            axes[name] = c
        self.axes = axes

    @property
    def enable_physics(self):
        return bool(self._mo.physics)

    @enable_physics.setter
    def enable_physics(self, v):
        self._mo.physics = 1 if v else 0

    @property
    def _instr_credit(self):
        return int(self._mo.instr_credit)

    @_instr_credit.setter
    def _instr_credit(self, v):
        self._mo.instr_credit = int(v)

    @property
    def INSTR_PER_STEP(self):
        return int(self._mo.instr_per_step)

    @INSTR_PER_STEP.setter
    def INSTR_PER_STEP(self, v):
        self._mo.instr_per_step = int(v)

    @property
    def steps_consumed(self):
        return int(self._mo.steps_consumed)

    @steps_consumed.setter
    def steps_consumed(self, v):
        self._mo.steps_consumed = int(v)

    def consume_step(self, dx, dy, dz):
        self._core.L.fc_consume(int(dx), int(dy), int(dz))

    def advance(self, instr_delta):
        self._core.L.fc_advance(int(instr_delta))


class CounterC(object):
    """devices.Counter over the C struct (the core increments it on every strobe)."""
    def __init__(self, log, motion):
        self.log = log
        self._c = motion

    value = _cprop("counter_value")
    latched = _cprop("counter_latched")

    def advance(self, steps):
        self._c.counter_value = (self._c.counter_value + steps) & 0xFFFF

    def port_read(self, port, pc):
        if port == 0x30:
            self._c.counter_latched = self._c.counter_value & 0xFFFF
            return self._c.counter_latched & 0xFF
        if port == 0x34:
            return (self._c.counter_latched >> 8) & 0xFF
        return 0


class MotionRegsC(object):
    """devices.MotionRegs over the C struct. `write`/`read` pass into the core;
    a script that hooks `write` sees every access as before (the core then
    routes all accesses to the window to Python)."""
    SIZE = _dev.MotionRegs.SIZE
    GROUPS = _dev.MotionRegs.GROUPS
    STROBE = _dev.MotionRegs.STROBE
    SYNC_LO, SYNC_HI = _dev.MotionRegs.SYNC_LO, _dev.MotionRegs.SYNC_HI

    def __init__(self, log, core, motion, counter=None, subcpu=None):
        self.log = log
        self._core, self._c = core, motion
        self.counter = counter
        self.subcpu = subcpu
        self.regs = motion.regs
        self.pos = PosView(motion.pos)
        motion.sync_lo, motion.sync_hi = self.SYNC_LO, self.SYNC_HI

    strobes = _cprop("strobes")
    handshakes = _cprop("handshakes")
    sync_ptr = _cprop("sync_ptr")
    sync_posts = _cprop("sync_posts")
    sync_acks = _cprop("sync_acks")

    @property
    def path(self):
        return self._core.path_list()

    @property
    def other_writes(self):
        return self._core.other_list()

    def _word(self, off):
        v = self.regs[off] | (self.regs[off + 1] << 8)
        return v - 0x10000 if v & 0x8000 else v

    def read(self, off, size, pc):
        return int(self._core.L.fc_motion_read(off, size, pc))

    def write(self, off, size, value, pc):
        self._core.L.fc_motion_write(off, size, value, pc)
