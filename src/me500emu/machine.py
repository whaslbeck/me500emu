"""Wiring: memory map, device stubs, ports, and the run loop.

Stage B implements the BUS only. Devices are stubs that record what the
firmware asks of them; in strict mode they refuse, in survey mode they answer 0
and note it. Survey mode exists to produce the ordered list of devices the boot
path actually needs - which is the input to stage C.
"""
import os
import struct
from unicorn import *
from unicorn.x86_const import *

from . import fastcore
from . import fastdev
from . import bus, symbols, paths
from .peer import SubCpu
from .cpu import Cpu
from . import devices as dev
from .service import Service

# Ports the analysis has named. Anything else is a finding, not a default.
KNOWN_PORTS = {
    0x04: "lcd data/command", 0x06: "sensor register", 0x07: "lcd control",
    0x08: "key matrix in", 0x09: "panel strobe/status", 0x0a: "board ctrl + NVRAM WP",
    0x0b: "boot init", 0x10: "boot init", 0x11: "boot init",
    0x14: "boot init", 0x15: "frontend init", 0x16: "frontend init",
    0x17: "boot init", 0x18: "uart data", 0x19: "uart status",
    0x1c: "frontend status seed", 0x1d: "frontend status seed",
    0x30: "counter latch/low", 0x34: "counter high",
    0x00: "unexplained single site", 0x2e: "unexplained single site",
    0x89: "unexplained single site", 0x8a: "unexplained single site",
    0xff00: "peer register", 0xff12: "peer register", 0xff80: "peer ready",
}
# Found by the emulator at stage B, and NOT by the static port census: boot
# writes 64 consecutive word ports in a loop (8025d: CX=0x40, DX=0xff00,
# ADD DX,2). The census reported only 0xff00 because its back-walk could not
# see DX being advanced. 64 registers x 16 KiB = the whole 1 MiB logical space.
for p in range(0xff00, 0xff80, 2):
    KNOWN_PORTS[p] = "page register idx %d (logical %05x)" % (
        (p - 0xff00) // 2, ((p - 0xff00) // 2) * 0x4000)


class _WindowView(object):
    """Adapts a window's mmio offset back to a logical address."""

    def __init__(self, store, base):
        self.store, self.base = store, base

    def read(self, off, size, pc):
        return self.store.read(self.base + off, size, pc)

    def write(self, off, size, value, pc):
        self.store.write(self.base + off, size, value, pc)


MOTION_TICK_CALIBRATED = 2000

# ---- From ticks to seconds: NOT solved here -----------------------------------
#
# On 2026-08-31 this block briefly carried a derived tick rate of 4000 Hz and an
# acceleration of 1569 mm/s^2 (0.16 g). **Both were wrong**, and the mistake is
# instructive enough to stay here.
#
# The derivation anchored [0x10c4] = 2000 to the maximum speed XY-MS = 80 mm/s.
# In fact [0x1022] = 200, and the rate root works in 0.1 mm/s - so **20 mm/s**,
# exactly what the LCD shows as "XYES:20". Off by a factor of four.
#
# And the cross-check that seemed to agree to three decimals -
# v^2/a = 4.080 mm = [0x10c6] - is **scale invariant**: it holds for any tick
# rate, because v and a scale together. It proves the internal consistency of
# the firmware constants and nothing at all about the absolute time scale.
#
# WHAT IS DOCUMENTED, from the specification table of the service manual:
#
#     X/Y  Moving      20, 40, 60, 80 mm/s      resolution 0.5 um  (= 2000/mm)
#     Z    Engraving   0.5 ... 10 mm/s          resolution 0.25 um (= 4000/mm)
#     Z    Moving      5 ... 30 mm/s
#     Acceleration     Engraving 0.05 G = 490 mm/s^2
#                      Moving    0.30 G = 2943 mm/s^2
#
# So the acceleration need not be derived at all - it is given. And neither
# value can be reconciled with 204 ramp ticks and 40 pulses per tick: 20 mm/s
# would give 0.01 G, 80 mm/s 0.16 G. One of the three quantities - tick rate,
# meaning of [0x10c2], or the mapping of [0x10c4] to a target speed - is not yet
# understood correctly.
#
# HOW TO SETTLE IT, and both are simple:
#   * test point TP7 (CPCK) with an oscilloscope - gives the CPU clock.
#   * or simply a stopwatch on the real machine: a job of known length, time
#     it, and the tick rate follows directly.
# (The tick was later measured on the machine: 1000/s, see the motion tick
# comment in Machine.__init__.)
ACCEL_ENGRAVING_MM_S2 = 0.05 * 9810.0   # from the specification table
ACCEL_MOVING_MM_S2 = 0.30 * 9810.0
XY_SPEEDS_MM_S = (20, 40, 60, 80)       # Moving, specification table
Z_SPEEDS_MOVING_MM_S = (5, 10, 15, 20, 25, 30)
Z_SPEEDS_ENGRAVING_MM_S = (0.5, 1, 2, 3, 5, 8, 10)
CRYSTAL_HZ = 25000000                   # found on the PCB; TP7 would confirm it

# Deliberately NOT defined: TICKS_PER_SECOND and instr_to_seconds(). As long as
# the time scale is not settled, any figure in seconds would be an invention
# with decimals.


# The machine's standard configuration, as the owner operates it: MGL-IIc with a
# 10 um program step (COMMAND = IIC_10, NVRAM cell 0x184 = 1), set through the
# panel and read back off the part (firmware analysis of the mode-set command).
# Pass nvram_seed=None for the factory machine (IIC_25, 25 um), which is what
# every log before 2026-08-31 was taken with.
DEFAULT_NVRAM_SEED = paths.data_path("nvram", "iic10_default.bin")


class Machine(object):
    def __init__(self, rom_path, strict=True, trace_ports=True,
                 sensor_value=None, physics=False, trace="count",
                 nvram_path=None, nvram_seed=DEFAULT_NVRAM_SEED):
        # sensor_value=None lets Endstops use its own IDLE (0x8F = all three
        # switches free). The old default of 0xFF meant "all actuated" under
        # the corrected polarity and silently overrode it.
        self.strict = strict
        self.faults = []
        self.dev_log = []          # (region, "r"/"w", offset, size, value, pc)
        self.port_log = []
        self.unknown_ports = set()
        self.board_0a = 0xC0
        self.rom = open(rom_path, "rb").read()
        assert len(self.rom) == bus.ROM_SIZE
        symbols.load()

        self.cpu = Cpu(self)
        # The C glue layer (fastcore.c): slice loop, 8259, timers, INT dispatch,
        # port/MMIO forwarding. FASTCORE=0 switches it off.
        # The C core is the only core; the Python core (slice loop, counting hooks) has been
        # retired. The counting modes count/full are now done in C as well.
        if not fastcore.ENABLED:
            raise RuntimeError("FASTCORE=0: the Python core has been retired; the C core is required")
        if trace not in ("fast", "count", "full"):
            raise ValueError("trace must be fast, count or full")
        self.fc = None
        if True:
            self.fc = fastcore.Core(self.cpu.uc, self._fc_port_in, self._fc_port_out,
                                    self._fc_mmio_read, self._fc_mmio_write,
                                    self._fc_intr_fallback, self._fc_between)
            self._fc_exc = None
        self.service = Service(self)
        self._irq_dispatches = []
        self._pending_irqs = []
        self._pic_default7_py = 0
        self._dropped_irqs = []
        self._instr = 0
        self._abort = False
        self._tick_interval = self._motion_tick_interval = self._service_tick_interval = 0
        self._uart_byte_interval = 0
        self._uart_overruns_py = 0             # 8251 overruns in the Python core
        self.started = False
        self.tick_interval = 0          # 0 = no INT 23h timer
        # 0 = no INT 20h motion tick. MOTION_TICK_CALIBRATED is the value the
        # machine actually works at - found by a sweep: below it the tick
        # starves the HP-GL frontend and no job is ever submitted; above it the
        # step engine is too slow to drain the queue. Measured on the machine
        # (2026-09-20): the tick runs at 1000/s, so one model instruction is
        # 0.5 us (2000 instructions per ms, ~2 M instructions/s). The machine
        # manages 3.9 M register instructions/s, 1.7 M memory instructions/s,
        # 0.84 M accesses/s to the sub CPU's RAM - the model averages that.
        self.motion_tick_interval = 0
        # INT 24h, the service tick of the frontend side (firmware analysis of
        # the RS232 frontend): 8253 counter 2, 5000 counts, reloaded by the
        # handler 8000:c02f itself. It starts transmission of replies, releases
        # DTR again and services the reply queues. Until 2026-09-06 it was
        # missing; replies (OA, OP..) stayed stuck in the queue. 0 = off.
        self.service_tick_interval = 0
        # Instructions between two received serial bytes. NOT cosmetic: with
        # bytes delivered as a burst - one every ~130 instructions - a job with
        # short coordinates raises ERR10 COMMAND and the coordinates never reach
        # the planner. Spaced out, the same jobs parse and the motion scales
        # linearly with the coordinate. 2000 is the order of magnitude a 9600
        # baud line implies at a few million instructions per second; the exact
        # value does not matter, only that the frontend gets to run between
        # bytes. 0 restores the burst.
        self.uart_byte_interval = 2000
        self._ctr0_latched = None       # 8253 counter 0 (port 0x14), see port_read
        self._ctr1_latched = None       # 8253 counter 1 (port 0x15), LSB/MSB alternating
        self._ctr1_hi = False
        self._last_mtick_py = 0
        self._last_tick_py = 0
        self.watch = {}                 # addr -> hit count
        # Set from another thread to make run() return at the next slice
        # boundary. Needed because the UI's emulator thread sits inside
        # uc_emu_start when Ctrl-C arrives, and letting the interpreter finalise
        # under a running Unicorn hook segfaults the process.
        self.abort = False
        uc = self.cpu.uc

        # ---- stage C devices ------------------------------------------
        # nvram_path makes the part persistent: it is read at construction and
        # written back by whoever owns the machine (the UI does it on shutdown).
        # Without a path the emulator behaves as before - ROM seed every run.
        seed = os.path.normpath(nvram_seed) if nvram_seed else None
        if seed and not os.path.exists(seed):
            raise IOError("NVRAM default configuration missing: %s "
                          "(regenerate it by setting the mode through the panel)" % seed)
        self.nvram = dev.Nvram(self.rom, self.dev_log, path=nvram_path, seed=seed)
        self.panel = dev.Panel(self.dev_log)
        # The second V33 - the MOTOR CONTROL CPU, per the service manual's block
        # diagram. Its program has not been located; six searches now, the
        # latest of which checked whether it is copied into the interface RAM
        # at boot: ZERO bytes of the 1 MiB window store match the ROM. So it is
        # not downloaded there either.
        # This stays a behavioural model with its assumptions named in
        # peer.py - and note the axes are DC SERVOS with encoders, so this
        # model's open-loop payout cannot produce the machine's own
        # following-error and overcurrent aborts.
        if self.fc:
            # Stage 2: motion window, counter and axes in C (fastdev.py: views)
            region = [r[0] for r in bus.DEVICE_REGIONS].index("motion_regs")
            self.fc.init_motion(region, self._fc_consume)
            self.subcpu = fastdev.SubCpuC(self, self.fc, self.fc.motion, enable_physics=physics)
        else:
            self.subcpu = SubCpu(self, enable_physics=physics)
        self.sensors = dev.Endstops(self.dev_log, sensor_value, travel=10,
                                    subcpu=self.subcpu if physics else None)
        self.counter = (fastdev.CounterC(self.dev_log, self.fc.motion) if self.fc
                        else dev.Counter(self.dev_log))
        self.uart = dev.Uart(self.dev_log, state=self.fc.state if self.fc else None)
        self.uart.machine = self                # time source for the transmit model (holding register)
        self.pic = dev.Pic(self.dev_log, self._pending_irqs,
                           state=self.fc.state if self.fc else None,
                           pending_fn=self.fc.pending_list if self.fc else None)
        self.pages = dev.PageRegisters(self.cpu, self.dev_log)
        self.store = dev.PagedStore(self.pages, self.dev_log)
        self.motion = (fastdev.MotionRegsC(self.dev_log, self.fc, self.fc.motion, self.counter, self.subcpu)
                       if self.fc else dev.MotionRegs(self.dev_log, self.counter, self.subcpu))
        self.implemented = {"nvram": self.nvram, "motion_regs": self.motion}
        # the four windows share one backing store, addressed through the page
        # registers; the logical base of each window is added back so the
        # translation sees the true logical address
        for wname, wbase in (("window_prod", 0x30000), ("window_cons", 0x40000),
                             ("window_probe", 0x50000), ("window_font", 0x60000)):
            self.implemented[wname] = _WindowView(self.store, wbase)

        uc.mem_map(bus.RAM_BASE, bus.RAM_SIZE, UC_PROT_ALL)
        uc.mem_map(bus.ROM_BASE, bus.ROM_SIZE, UC_PROT_READ | UC_PROT_EXEC)
        uc.mem_write(bus.ROM_BASE, self.rom)

        # Unicorn's mmio_map needs page granularity (0x1000). Several devices
        # are smaller, so the mapping is rounded up and the callback enforces
        # the REAL size - otherwise the padding would silently accept accesses
        # that belong to nothing.
        PAGE = 0x1000
        self._fc_regions = []
        for name_, base, size, _why in bus.DEVICE_REGIONS:
            mapped = (size + PAGE - 1) & ~(PAGE - 1)
            assert base % PAGE == 0, "%s base not page aligned" % name_
            if self.fc:
                self._fc_regions.append((name_, base, size))
                self.fc.mmio_map(base, mapped, len(self._fc_regions) - 1)
            else:
                uc.mmio_map(base, mapped,
                            self._mk_read(name_, base, size), None,
                            self._mk_write(name_, base, size), None)

        # Instruction accounting. This hook runs for EVERY instruction, so its
        # body is the emulator's single largest cost - the original version did
        # a dict lookup and a list append-and-pop per instruction, which on a
        # 60M-instruction boot is 60M list operations. Three tiers now:
        #
        #   trace="fast"   no hook at all; instr follows the granted budget.
        #                  Off by a few instructions wherever a hook stops the
        #                  engine early (BRKXA, a software INT), which is fine
        #                  for the UI and for anything not counting cycles.
        #   trace="count"  one increment per instruction. Exact, and the
        #                  default, because most analysis here compares counts.
        #   trace="full"   also keeps last_pc, the watch counters and the
        #                  recent-PC ring. Only boot tracing needs the ring.
        self.instr = 0
        self.trace = trace
        self.fc.state.trace_mode = {"fast": 0, "count": 1, "full": 2}[trace]
        self._watch_installed = None

        # (2026-09-25) Instruction costs per class (costs.py, replaces the flat executor 1.4
        # calibration): delta in eighth units per instruction start of the image; in the C core via
        # fc_costs, in the Python core the same hook (fires per repetition for rep). Peer surcharge on
        # the motion window in both cores. Calibrated against the ~B values measured on the machine.
        self._cost_acc = 0; self.cost_extra = 0
        from . import costs as _costs
        _tab = _costs.build_table(self.rom, paths.cache_dir())
        self._cost_table = [(x - 256 if x >= 128 else x) for x in _tab]
        self._cost_peer8 = 0                      # armed only after boot (arm_costs):
        self._costs_armed = False                 # the boot homing run is calibrated to flat
        self._cost_table_raw = _tab                 # time (ERR52 if armed early)

        # The C core's hooks FIRST: after an emu_stop inside a hook, Unicorn no longer calls the remaining
        # hooks of the same instruction. The CPU's BRKXA/RETXA hook stops and redirects - with the counting
        # hook behind it, the XA instruction was missing in count/full (found by bisection: n = 115172, 8000:0457).
        self.fc.install_hooks()
        self.cpu.install(fast=True)

    # ---- views onto the C state (fastcore) ------------------------------
    # Scripts read and set `instr`, `abort`, the tick intervals and the IRQ
    # lists as attributes; with the core they live in the C struct.
    def _st(self):
        return self.fc.state if self.fc else None

    def arm_costs(self):
        """Activate class costs - after the boot homing run (snapshot_state.booted).
        The boot itself runs on flat time, to which its homing run is calibrated."""
        from . import costs as _costs
        if getattr(self, "_costs_armed", False):
            return
        self._costs_armed = True
        self._cost_peer8 = int(round(_costs.PEER_SURCHARGE * 16))
        if self.fc is not None:
            self.fc.set_costs(self._cost_table_raw, self._cost_peer8)

    last_pc = property(lambda self: int(self.fc.state.last_pc))

    @property
    def recent(self):
        """The last (up to 24) executed addresses, oldest first (trace="full" only)."""
        st = self.fc.state
        n, pos = int(st.recent_n), int(st.recent_pos)
        return [int(st.recent[(pos - n + k) % 24]) for k in range(n)]

    instr = property(lambda self: self.fc.state.instr if self.fc else self._instr,
                     lambda self, v: (setattr(self.fc.state, "instr", int(v)) if self.fc
                                      else setattr(self, "_instr", v)))
    abort = property(lambda self: bool(self.fc.state.abort) if self.fc else self._abort,
                     lambda self, v: (setattr(self.fc.state, "abort", 1 if v else 0) if self.fc
                                      else setattr(self, "_abort", v)))

    def _mk_interval(name):
        def get(self):
            return getattr(self.fc.state, name) if self.fc else getattr(self, "_" + name)

        def set_(self, v):
            if self.fc:
                setattr(self.fc.state, name, int(v))
            else:
                setattr(self, "_" + name, v)
        return property(get, set_)
    tick_interval = _mk_interval("tick_interval")
    motion_tick_interval = _mk_interval("motion_tick_interval")
    service_tick_interval = _mk_interval("service_tick_interval")
    uart_byte_interval = _mk_interval("uart_byte_interval")
    del _mk_interval

    @property
    def uart_overruns(self):
        """8251 overruns: a new byte fell due while the previous one was still unread (C core only, counted only)."""
        return int(self.fc.state.uart_overruns) if self.fc else self._uart_overruns_py

    @property
    def jitter(self):
        """Phase jitter of interrupt delivery (C core only): 0 = off, otherwise the seed of the
        random generator. A due interrupt then lands 1..512 instead of exactly 512 instructions
        later, and every tick period varies by +-1/8 (fastcore.c, jrand/jint)."""
        return int(self.fc.state.jitter) if self.fc else 0

    @jitter.setter
    def jitter(self, seed):
        if not self.fc:
            raise RuntimeError("phase jitter exists only in the C core (FASTCORE=1)")
        self.fc.state.jitter = int(seed) & 0xFFFFFFFFFFFFFFFF
        self.fc.state.jrng = ((int(seed) * 0x9E3779B97F4A7C15) | 1) & 0xFFFFFFFFFFFFFFFF

    @property
    def pending_irqs(self):
        return self.fc.pending_list() if self.fc else self._pending_irqs

    @property
    def irq_dispatches(self):
        return self.fc.dispatch_log() if self.fc else self._irq_dispatches

    @property
    def dropped_irqs(self):
        return self.fc.dropped_list() if self.fc else self._dropped_irqs

    # ---- core callbacks ------------------------------------------------
    def _fc_guard(self, f, *a):
        try:
            return f(*a)
        except Exception as e:          # ctypes would swallow the exception
            if self._fc_exc is None:
                self._fc_exc = e
            self.cpu.uc.emu_stop()
            return 0

    def _fc_port_in(self, port, size, pc):
        return self._fc_guard(self.port_read, port, size, pc) & 0xFFFFFFFF

    def _fc_port_out(self, port, size, value, pc):
        self._fc_guard(self.port_write, port, size, value, pc)

    def _fc_mmio_read(self, region, offset, size, pc):
        name_, base, real = self._fc_regions[region]
        return self._fc_guard(self._dev_read, name_, base, real, offset, size, pc) & 0xFFFFFFFFFFFFFFFF

    def _fc_mmio_write(self, region, offset, size, value, pc):
        name_, base, real = self._fc_regions[region]
        self._fc_guard(self._dev_write, name_, base, real, offset, size, value, pc)

    def _fc_intr_fallback(self, intno, pc):
        self._fc_guard(self.service.handle, intno, self.cpu.uc)

    def _fc_between(self, delta):
        if self.subcpu is not None and delta:
            self._fc_guard(self.subcpu.advance, delta)

    def _fc_consume(self, dx, dy, dz):
        # only when a script has rebound `subcpu.consume_step` (consume_py)
        self._fc_guard(self.subcpu.consume_step, dx, dy, dz)

    # ---- device stubs -------------------------------------------------
    def _mk_read(self, name_, base, real_size):
        def rd(uc, offset, size, _ud):
            pc = uc.reg_read(UC_X86_REG_CS) * 16 + uc.reg_read(UC_X86_REG_IP)
            return self._dev_read(name_, base, real_size, offset, size, pc)
        return rd

    def _dev_read(self, name_, base, real_size, offset, size, pc):
            if name_ == "motion_regs":
                self._cost_acc += self._cost_peer8       # (2026-09-25) sub-CPU window: surcharge per access
            uc = self.cpu.uc
            if offset >= real_size:
                self.faults.append(bus.fault_msg(
                    "read past the end of device '%s'" % name_,
                    base + offset, size, pc,
                    "device is 0x%x bytes; this is mapping padding" % real_size))
                uc.emu_stop()
                return 0
            d = self.implemented.get(name_)
            if d is not None:
                return d.read(offset, size, pc)
            self.dev_log.append((name_, "r", offset, size, None, pc))
            if self.strict:
                self.faults.append(bus.fault_msg(
                    "read from unimplemented device '%s'" % name_,
                    base + offset, size, pc, "stage C implements this"))
                uc.emu_stop()
            return 0

    def _mk_write(self, name_, base, real_size):
        def wr(uc, offset, size, value, _ud):
            pc = uc.reg_read(UC_X86_REG_CS) * 16 + uc.reg_read(UC_X86_REG_IP)
            self._dev_write(name_, base, real_size, offset, size, value, pc)
        return wr

    def _dev_write(self, name_, base, real_size, offset, size, value, pc):
            if name_ == "motion_regs":
                self._cost_acc += self._cost_peer8       # (2026-09-25) sub-CPU window: surcharge per access
            uc = self.cpu.uc
            if offset >= real_size:
                self.faults.append(bus.fault_msg(
                    "write past the end of device '%s'" % name_,
                    base + offset, size, pc,
                    "device is 0x%x bytes; this is mapping padding" % real_size))
                uc.emu_stop()
                return
            d = self.implemented.get(name_)
            if d is not None:
                d.write(offset, size, value, pc)
                return
            self.dev_log.append((name_, "w", offset, size, value, pc))
            if self.strict:
                self.faults.append(bus.fault_msg(
                    "write to unimplemented device '%s'" % name_,
                    base + offset, size, pc, "value %04x; stage C implements this" % value))
                uc.emu_stop()

    def _ctr0_live(self):
        """8253 counter 0 as a mode 2 counter with initial value 76 over the motion tick period."""
        per = self.motion_tick_interval or 2000
        since = int(self.fc.state.last_mtick) if self.fc else self._last_mtick_py
        elapsed = (self.instr - since) % per
        if elapsed == 0:
            # Read in the slice in which the tick was delivered (the core counts instructions only at
            # slice boundaries): the service time of the tick itself is not resolved - model: the
            # measured trace of the handler, 392 instructions
            elapsed = min(per - 1, 400)
        count = 76 - (elapsed * 76) // per
        v = self._ctr0_latched
        self._ctr0_latched = None                      # a latch holds for one read
        return max(1, min(76, count)) if v is None else v

    def _ctr1_live(self):
        """8253 counter 1 as a mode 0 counter with initial value 50000 over the timer period (INT 23h)."""
        per = self.tick_interval or 50000
        since = int(self.fc.state.last_tick) if self.fc else self._last_tick_py
        elapsed = self.instr - since
        if elapsed >= per:
            return 0
        return max(0, 50000 - (elapsed * 50000) // per)

    # ---- ports ---------------------------------------------------------
    def port_read(self, port, size, pc):
        if len(self.port_log) < 200000:      # bounded: this grew without limit
            self.port_log.append(("IN", port, size, None, pc))
        if port in (0x04, 0x08, 0x09):
            return self.panel.port_read(port, pc)
        if port == 0x06:
            return self.sensors.port_read(port, pc, self.cpu.uc)
        if port in (0x30, 0x34):
            return self.counter.port_read(port, pc)
        if port == 0x14:
            # 8253 counter 0 (mode 2, 76, LSB only): the motion tick itself. Count = 76 - elapsed since
            # the last IRQ0 delivery, in counter ticks of the period; latched by OUT 0x17, 0x00.
            return self._ctr0_latched if self._ctr0_latched is not None else self._ctr0_live()
        if port == 0x15:
            # 8253 counter 1 (mode 0, 50000, LSB then MSB): the INT 23h timer reloads it every period.
            # Count = 50000 - elapsed since the last IRQ3 delivery; latched by OUT 0x17, 0x40.
            v = self._ctr1_latched if self._ctr1_latched is not None else self._ctr1_live()
            if self._ctr1_hi:
                self._ctr1_hi = False
                self._ctr1_latched = None
                return (v >> 8) & 0xFF
            self._ctr1_hi = True
            self._ctr1_latched = v
            return v & 0xFF
        if port in (0x18, 0x19):
            return self.uart.port_read(port, pc)
        if port in (0x10, 0x11):
            return self.pic.port_read(port, pc)
        if 0xff00 <= port <= 0xff80:
            return self.pages.port_read(port, pc)
        if port == 0x0a:
            return self.board_0a
        if port not in KNOWN_PORTS:
            self.unknown_ports.add(port)
            self.faults.append(bus.fault_msg(
                "IN from unknown port 0x%04x" % port, 0, size, pc,
                "not in the port census - this is a finding"))
            self.cpu.uc.emu_stop()
        return 0

    def port_write(self, port, size, value, pc):
        if len(self.port_log) < 200000:
            self.port_log.append(("OUT", port, size, value, pc))
        if port in (0x04, 0x07, 0x09):
            return self.panel.port_write(port, value, pc)
        if port in (0x18, 0x19):
            return self.uart.port_write(port, value, pc)
        if port in (0x10, 0x11):
            return self.pic.port_write(port, value, pc)
        if port == 0x17 and (value & 0xF0) == 0x00:
            self._ctr0_latched = self._ctr0_live()      # latch command for counter 0
            return
        if port == 0x17 and (value & 0xF0) == 0x40:
            self._ctr1_latched = self._ctr1_live()      # latch command for counter 1
            self._ctr1_hi = False
            return
        if 0xff00 <= port <= 0xff80:
            return self.pages.port_write(port, value, pc)
        if port == 0x0a:
            # bits 6-7 gate NVRAM writes: AND 0x3f|0x00 enables, |0xc0 protects
            self.board_0a = value & 0xFF
            self.nvram.write_enabled = (value & 0xC0) == 0
            return
        if port not in KNOWN_PORTS:
            self.unknown_ports.add(port)
            self.faults.append(bus.fault_msg(
                "OUT to unknown port 0x%04x" % port, 0, size, pc,
                "value %04x - not in the port census" % value))
            self.cpu.uc.emu_stop()

    # ---- run -----------------------------------------------------------
    def run(self, max_instr=200000, slice_size=20000):
        """Sliced run so hardware interrupts can be injected.

        Unicorn does not raise interrupts on its own, so the emulator plays the
        interrupt controller: between slices, if IF is set and something is
        pending, dispatch it through the IVT exactly as the CPU would.

        The slice is capped at the next interrupt deadline. Without that cap a
        fixed 20000-instruction slice could only ever deliver ONE motion tick,
        while a 200-instruction tick interval makes a hundred of them fall due
        in that span - so 99% of ticks were silently dropped. That is not a
        cosmetic timing error: it starves the step engine, moves never run their
        distance counter down, the move queue never drains, and a job needing
        the full ring appears to deadlock. Measured before the fix: 1110 of
        100000 due ticks delivered on the job that stuck, 27487 on the one that
        did not.

        While an interrupt is pending but undeliverable (the kernel's scheduler
        holds a 5-instruction CLI window at 9d6bc..9d6c6 to test a task's wait
        word), the slice shrinks to 64 so delivery happens as soon as IF comes
        back, rather than a whole slice later.
        """
        uc = self.cpu.uc
        if not self.started:
            # cold start; a later run() CONTINUES from the current CS:IP
            # instead of resetting - restarting silently rebooted the machine
            # and made a fed job look like it had been consumed twice.
            uc.reg_write(UC_X86_REG_CS, 0xFFFF)
            uc.reg_write(UC_X86_REG_IP, 0x0000)
            self.started = True
            at = 0xFFFF0
        else:
            at = uc.reg_read(UC_X86_REG_CS) * 16 + uc.reg_read(UC_X86_REG_IP)
        if self.fc:
            return self._run_fast(at, max_instr, slice_size)
        raise RuntimeError("no C core")   # unreachable: __init__ requires the core

    def _run_fast(self, at, max_instr, slice_size):
        """The slice loop in C (fastcore.c, fc_run). Exceptions from Python callbacks
        and from the scripts' Unicorn hooks are re-raised afterwards."""
        uc = self.cpu.uc
        self._fc_exc = None
        uc._hook_exception = None
        # If a script has rebound the window or the step consumer, that part keeps
        # running through Python - otherwise the core stays self-contained.
        mo = self.fc.motion
        mo.motion_py = 1 if ("write" in self.motion.__dict__ or "read" in self.motion.__dict__) else 0
        mo.consume_py = 1 if "consume_step" in self.subcpu.__dict__ else 0
        if self.trace != "fast":                 # mirror watch counters (count/full) into the core
            keys = tuple(sorted(self.watch))
            if keys != self._watch_installed:
                self.fc.set_watch(keys); self._watch_installed = keys
        try:
            self.fc.run(at, int(max_instr), int(slice_size))
        finally:
            if self.trace != "fast" and self._watch_installed:
                for addr, n in self.fc.get_watch().items():
                    if n:
                        self.watch[addr] = self.watch.get(addr, 0) + n
        st = self.fc.state
        if st.err:
            self.faults.append("UcError %s at PC %05x  %s\n"
                               % (UcError(st.err), st.err_pc, symbols.name(st.err_pc)))
            st.err = 0
        if self._fc_exc is not None:
            e, self._fc_exc = self._fc_exc, None
            raise e
        if uc._hook_exception is not None:
            e, uc._hook_exception = uc._hook_exception, None
            raise e
        return self.instr
