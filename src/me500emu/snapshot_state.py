"""Save and restore a booted machine, so tests do not pay for the boot.

Every measurement in this project starts by running the power-on reference:
about 95 million instructions, a minute and a half of wall clock, identical
every time. Snapshotting the machine once and restoring it turns that into
well under a second.

What is saved: all mapped memory, the CPU registers, and the device state that
matters - panel, axes, page registers, counters, the motion window. What is NOT
saved is anything the host owns: open hooks, the Unicorn object itself, the
Python callables. Restoring therefore builds a fresh Machine and pours the state
into it, which also means a restored machine has whatever hooks the caller
installs, not the ones the snapshot was taken with.
"""
import os
import pickle
import zlib
import binascii

from unicorn.x86_const import (
    UC_X86_REG_AX, UC_X86_REG_BX, UC_X86_REG_CX, UC_X86_REG_DX,
    UC_X86_REG_SI, UC_X86_REG_DI, UC_X86_REG_BP, UC_X86_REG_SP,
    UC_X86_REG_CS, UC_X86_REG_DS, UC_X86_REG_ES, UC_X86_REG_SS,
    UC_X86_REG_IP, UC_X86_REG_EFLAGS)

REGS = [("AX", UC_X86_REG_AX), ("BX", UC_X86_REG_BX), ("CX", UC_X86_REG_CX),
        ("DX", UC_X86_REG_DX), ("SI", UC_X86_REG_SI), ("DI", UC_X86_REG_DI),
        ("BP", UC_X86_REG_BP), ("SP", UC_X86_REG_SP), ("CS", UC_X86_REG_CS),
        ("DS", UC_X86_REG_DS), ("ES", UC_X86_REG_ES), ("SS", UC_X86_REG_SS),
        ("IP", UC_X86_REG_IP), ("FL", UC_X86_REG_EFLAGS)]

# 3 since 2026-09-07: the axis geometry is stored in the snapshot and compared on
# load, and the rest position after boot is the origin (peer.rebase_to_origin).
# Older snapshots count as stale and are rebooted instead of silently loading the
# old frame (X 22.6 mm before the end stop).
VERSION = 3


class StaleSnapshot(ValueError):
    """The snapshot does not match this machine: a different ROM, a different
    axis geometry or an older format."""


def geometry(m):
    """Axis geometry as a key: travel, resolution, sensor band, origin."""
    return ";".join("%s:%d:%g:%s:%d" % (k, a.travel_steps, a.STEPS_PER_MM, a.sensor_band,
                                        a.origin_steps)
                    for k, a in sorted((m.subcpu.axes if m.subcpu else {}).items()))


def _regions(m):
    """The memory the machine actually has mapped, from Unicorn itself - without
    the MMIO device windows: they have no backing, a read through them only
    produces padding faults, and a write on load would hit the device callbacks
    with an undefined PC."""
    from . import bus
    dev = [(base, size) for _n, base, size, _w in bus.DEVICE_REGIONS]
    out = []
    for a, b, _perm in m.cpu.uc.mem_regions():
        if any(base <= a < base + ((size + 0xFFF) & ~0xFFF) for base, size in dev):
            continue
        out.append((a, b - a + 1))
    return out


def save(m, path):
    uc = m.cpu.uc
    mem = []
    for base, size in _regions(m):
        try:
            mem.append((base, bytes(uc.mem_read(base, size))))
        except Exception:
            pass                       # an mmio window has no readable backing
    state = {
        "version": VERSION,
        "instr": m.instr,
        "regs": {n: uc.reg_read(r) for n, r in REGS},
        "mem": mem,
        "xa": m.cpu.xa_mode,
        "pages": list(m.pages.regs),
        "store": bytes(m.store.mem),
        "nvram": bytes(m.nvram.cells),
        "sensor_idle": getattr(m.sensors, "IDLE", None),
        "uart_ctrl": m.uart.ctrl,
        "pic": (m.pic.isr, m.pic.imr, m.pic.read_isr, m.pic.icw_left, m.pic.lowest),
        "motion_regs": bytes(m.motion.regs),
        "motion_pos": dict(m.motion.pos),
        "motion_strobes": m.motion.strobes,
        "panel_ddram": bytes(m.panel.ddram),
        "panel_ac": m.panel.ac,
        "panel_keys": list(m.panel.keys),
        "axes": {k: (a.pos, a.pending, a.clipped)
                 for k, a in m.subcpu.axes.items()} if m.subcpu else {},
        "geometry": geometry(m),
        "rebased": bool(getattr(m.subcpu, "rebased", False)) if m.subcpu else False,
        "tick": m.tick_interval,
        "mtick": m.motion_tick_interval,
        "stick": m.service_tick_interval,
    }
    blob = zlib.compress(pickle.dumps(state, 4), 6)
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(blob)
    os.replace(tmp, path)
    return len(blob)


def load(m, path):
    with open(path, "rb") as f:
        state = pickle.loads(zlib.decompress(f.read()))
    if state.get("version") != VERSION:
        raise StaleSnapshot("%s: snapshot version %r does not match %d"
                            % (path, state.get("version"), VERSION))
    if m.subcpu and state.get("geometry") != geometry(m):
        raise StaleSnapshot("%s: axis geometry %r, machine %r"
                            % (path, state.get("geometry"), geometry(m)))
    uc = m.cpu.uc
    from . import bus
    from . import bus as _bus
    _dev = [(b_, sz) for _n, b_, sz, _w in _bus.DEVICE_REGIONS]
    for base, data in state["mem"]:
        if any(b_ <= base < b_ + ((sz + 0xFFF) & ~0xFFF) for b_, sz in _dev):
            continue                   # MMIO window: no memory behind it
        # The snapshot freezes the ROM too. If it does not match this
        # machine's ROM, an OLD image would run from here on - which is the
        # case after every rebuild of a patched image as long as the cache
        # name stays the same. Better to abort than to silently measure the
        # wrong thing.
        if base == bus.ROM_BASE and bytes(data) != bytes(m.rom):
            raise StaleSnapshot("%s carries a different ROM than %s"
                                % (path, getattr(m, "rom_path", "the machine")))
        uc.mem_write(base, data)
    for n, r in REGS:
        uc.reg_write(r, state["regs"][n])
    m.instr = state["instr"]
    m.cpu.xa_mode = state["xa"]
    m.cpu._ivt_cache.clear()
    if state.get("nvram") is not None:
        m.nvram.cells[:] = state["nvram"]
    if state.get("sensor_idle") is not None:
        m.sensors.IDLE = state["sensor_idle"]
    m.pages.regs[:] = state["pages"]
    m.store.mem[:] = state["store"]
    m.uart.ctrl = state.get("uart_ctrl", 0)
    pic = state.get("pic", (0, 0xE0, False, 0))
    m.pic.isr, m.pic.imr, m.pic.read_isr, m.pic.icw_left = pic[:4]
    m.pic.lowest = pic[4] if len(pic) > 4 else 7
    m.motion.regs[:] = state["motion_regs"]
    m.motion.pos.update(state["motion_pos"])
    m.motion.strobes = state["motion_strobes"]
    m.panel.ddram[:] = state["panel_ddram"]
    m.panel.ac = state["panel_ac"]
    m.panel.keys[:] = state["panel_keys"]
    if m.subcpu:
        for k, (pos, pend, clip) in state["axes"].items():
            if k in m.subcpu.axes:
                a = m.subcpu.axes[k]
                a.pos, a.pending, a.clipped = pos, pend, clip
    if m.subcpu:
        m.subcpu.rebased = bool(state.get("rebased", False))
    m.tick_interval = state["tick"]
    m.motion_tick_interval = state["mtick"]
    m.service_tick_interval = state.get("stick", 0)
    m.started = True
    return m


def _default_cache(m):
    """booted-<rom md5 prefix>.snap in the cache directory: one cache dir serves every ROM image."""
    import hashlib
    from . import paths
    return paths.cache_path("booted-%s.snap" % hashlib.md5(bytes(m.rom)).hexdigest()[:8])


HISTORIC_SENSOR_IDLE = 0x8F      # what booted.snap was taken with
# The axis geometry booted.snap was taken with - Z still with 60 mm.
HISTORIC_GEOMETRY = ("X:966000:2000:None:0;Y:610000:2000:None:0;"
                     "Z:240000:4000:(26880, 38880):0")


def _cache_for(m, cache):
    """One cache file per machine configuration.

    Boot copies seven NVRAM blocks into RAM shadows and reads the attachment
    switches, so a machine booted with a different NVRAM or a different sensor
    idle is a different machine - restoring one snapshot over the other would
    silently undo the configuration. The historic combination (factory NVRAM,
    sensor idle 0x8F) keeps the plain name `booted.snap`; every other one gets a
    checksum over both appended, so a stale cache cannot be picked up by mistake.
    """
    if cache:
        return os.path.normpath(cache)
    from .devices import Nvram
    factory = Nvram(m.rom, [])
    idle = getattr(m.sensors, "IDLE", HISTORIC_SENSOR_IDLE)
    # The AXIS GEOMETRY belongs in the key as well. Without it a change to
    # travel or resolution would silently load a snapshot booted with the old
    # geometry - exactly the kind of trap that has invalidated measurements in
    # this project more than once.
    geom = geometry(m)
    if (bytes(m.nvram.cells) == bytes(factory.cells)
            and idle == HISTORIC_SENSOR_IDLE
            and geom == HISTORIC_GEOMETRY):
        return _default_cache(m)
    key = binascii.crc32(bytes(m.nvram.cells) + bytes([idle])
                         + geom.encode()) & 0xFFFFFFFF
    return _default_cache(m).replace(".snap", "-%08x.snap" % key)


def booted(rom, cache=None, physics=True, trace="fast", boot_instr=95000000,
           rebuild=False, on_create=None, **kw):
    """A machine past its power-on reference, from cache when possible.

    The reference is deterministic, so the first call boots and saves and every
    later one restores. Delete the cache, or pass rebuild=True, after changing
    anything that affects the boot - the axis model, the devices, the ROM.
    A non-default NVRAM (nvram_path=...) gets its own cache file automatically.
    """
    from .machine import Machine
    m = Machine(rom, physics=physics, trace=trace, **kw)
    if on_create is not None:
        # the caller gets the machine before the long cold boot, so it can abort
        # it (m.abort) instead of being stuck inside Unicorn on Ctrl-C
        on_create(m)
    path = _cache_for(m, cache)
    if physics:
        m.subcpu.sign_window()
    if not rebuild and os.path.exists(path):
        try:
            load(m, path)
            m.from_snapshot = True
            m.arm_costs()                     # class costs only after the boot
            return m
        except StaleSnapshot as e:
            print("Snapshot stale, rebooting: %s" % e)
            m = Machine(rom, physics=physics, trace=trace, **kw)
            if on_create is not None:
                on_create(m)
            if physics:
                m.subcpu.sign_window()
    from .machine import MOTION_TICK_CALIBRATED
    # Measured on the machine (2026-09-20): 8253 at 4.9152 MHz, counter 1 = 50000 -> INT 23h every
    # 10.17 ms, counter 2 = 5000 -> INT 24h every 1.017 ms, motion tick 1000/s; time scale 2000 instructions per ms
    m.tick_interval = int(os.environ.get("INT23", "20340"))
    m.motion_tick_interval = MOTION_TICK_CALIBRATED
    m.service_tick_interval = int(os.environ.get("INT24", "2035"))
    m.run(boot_instr)
    m.arm_costs()                             # class costs only after the boot
    if physics:
        # The rest position after homing is the origin LOW LEFT (peer.py,
        # ORIGIN_MARGIN_MM) - only after that is the table physically in front of the axis.
        m.subcpu.rebase_to_origin()
    save(m, path)
    m.from_snapshot = False
    return m
