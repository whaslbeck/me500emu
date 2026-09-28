"""ctypes binding of the C glue layer (fastcore.c).

Builds `fastcore.so` on first import (gcc, against the Unicorn headers of the venv,
without linking against the library: the entry point addresses come from the
`uclib` already loaded by the Python binding).

Since 2026-09-26 this is the ONLY core: the Python slice loop has been retired.
The counting modes trace="count"/"full" (exact instruction count per hook firing,
watch counters, last_pc, ring of the last 24 addresses) are provided by the C core;
verified bit-identical against the retired Python core over 20 M instructions
with 257560 interrupts.
`FASTCORE=0` aborts with a notice.
"""
import ctypes
import os
import subprocess
import sys

import unicorn
from unicorn import unicorn_const as UCC, x86_const as X
from unicorn.unicorn_py3 import unicorn as _binding

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "fastcore.c")
SO = os.path.join(HERE, "libfastcore.so")
if not os.access(HERE, os.W_OK) and not os.path.exists(SO):
    # installed read-only (site-packages): build into the user cache instead
    from . import paths as _paths
    SO = _paths.cache_path("libfastcore.so")
INCLUDE = os.path.join(os.path.dirname(unicorn.__file__), "include")
ENABLED = os.environ.get("FASTCORE", "1") != "0"
LOG_CAP = 200000


class State(ctypes.Structure):
    _fields_ = [
        ("instr", ctypes.c_uint64),
        ("tick_interval", ctypes.c_uint64), ("motion_tick_interval", ctypes.c_uint64),
        ("service_tick_interval", ctypes.c_uint64), ("uart_byte_interval", ctypes.c_uint64),
        ("abort", ctypes.c_int32), ("pad0", ctypes.c_int32),
        ("resume_at", ctypes.c_int64),
        ("pending", ctypes.c_int32 * 16), ("npending", ctypes.c_int32),
        ("pic_isr", ctypes.c_uint32), ("pic_imr", ctypes.c_uint32), ("pic_read_isr", ctypes.c_uint32),
        ("pic_icw_left", ctypes.c_uint32), ("pic_eois", ctypes.c_uint32), ("pic_blocked", ctypes.c_uint32),
        ("uart_rx_len", ctypes.c_uint32), ("uart_ctrl", ctypes.c_uint32), ("uart_honour_dtr", ctypes.c_uint32),
        ("err", ctypes.c_int32), ("err_pc", ctypes.c_uint64),
        ("dispatched", ctypes.c_uint64), ("dropped", ctypes.c_uint64), ("slices", ctypes.c_uint64),
        ("log_n", ctypes.c_uint32), ("log_cap", ctypes.c_uint32),
        ("dropped_n", ctypes.c_uint32), ("dropped_log", ctypes.c_uint32 * 64),
        ("jitter", ctypes.c_uint64), ("jrng", ctypes.c_uint64),   # phase jitter, 0 = off
        ("uart_overruns", ctypes.c_uint64),                        # 8251 overruns (counted only)
        ("uart_pending_since", ctypes.c_uint64), ("uart_overrun_after", ctypes.c_uint64),
        ("last_mtick", ctypes.c_uint64),                           # last IRQ0 delivery (8253 counter 0 model)
        ("last_tick", ctypes.c_uint64),                            # last IRQ3 delivery (8253 counter 1 model)
        ("pic_lowest", ctypes.c_uint64),                           # 8259: lowest priority (7 = default)
        ("uart_delivered", ctypes.c_uint32), ("uart_oe", ctypes.c_uint32),   # 8251 receive register
        ("uart_drop", ctypes.c_uint32), ("uart_pad", ctypes.c_uint32),
        ("next_tick", ctypes.c_uint64), ("next_mtick", ctypes.c_uint64), ("next_stick", ctypes.c_uint64),
        ("next_byte", ctypes.c_uint64), ("next_txirq", ctypes.c_uint64),   # deadlines across run() calls
        ("watch_if", ctypes.c_uint64), ("watch_count", ctypes.c_uint64), ("watch_last", ctypes.c_uint64),  # IF watch
        ("uart_deep", ctypes.c_uint64),          # control switch: infinitely deep receive buffer, no loss
        ("pic_default7", ctypes.c_uint64),       # 8259A default vector IR7 instead of IRQ1 when RxRDY is already 0 at INTA
        ("cost_acc", ctypes.c_int64), ("cost_extra", ctypes.c_int64),   # class costs: eighths accumulator (signed) and folded net units
        ("uart_tx_hold", ctypes.c_int64), ("uart_tx_free_at", ctypes.c_uint64),   # 8251 holding register / shifter free from
        ("trace_mode", ctypes.c_uint64), ("hcount", ctypes.c_uint64), ("last_pc", ctypes.c_uint64),   # counting modes
        ("recent", ctypes.c_uint64 * 24), ("recent_pos", ctypes.c_uint64), ("recent_n", ctypes.c_uint64),
    ]


class Axis(ctypes.Structure):
    _fields_ = [
        ("pos", ctypes.c_int64), ("pending", ctypes.c_int64), ("travel", ctypes.c_int64),
        ("steps_per_mm", ctypes.c_double),
        ("margin", ctypes.c_int32), ("home_at_low", ctypes.c_int32), ("both_ends", ctypes.c_int32), ("has_band", ctypes.c_int32),
        ("band_lo", ctypes.c_int64), ("band_hi", ctypes.c_int64),
        ("clipped", ctypes.c_uint32), ("hit_low", ctypes.c_uint32), ("hit_high", ctypes.c_uint32), ("max_command", ctypes.c_uint32),
        ("follow_error_max", ctypes.c_uint32), ("follow_trips", ctypes.c_uint32), ("follow_limit", ctypes.c_uint32),
        ("pad", ctypes.c_uint32),
    ]


class Motion(ctypes.Structure):
    _fields_ = [
        ("regs", ctypes.c_uint8 * 256),
        ("pos", ctypes.c_int64 * 3),
        ("strobes", ctypes.c_uint64),
        ("handshakes", ctypes.c_uint32), ("sync_ptr", ctypes.c_uint32), ("sync_posts", ctypes.c_uint32), ("sync_acks", ctypes.c_uint32),
        ("counter_value", ctypes.c_uint32), ("counter_latched", ctypes.c_uint32),
        ("motion_py", ctypes.c_int32), ("consume_py", ctypes.c_int32), ("physics", ctypes.c_int32), ("instr_per_step", ctypes.c_int32),
        ("instr_credit", ctypes.c_int64),
        ("steps_consumed", ctypes.c_uint64),
        ("axes", Axis * 3),
        ("path_n", ctypes.c_uint32), ("path_cap", ctypes.c_uint32),
        ("other_n", ctypes.c_uint32), ("other_cap", ctypes.c_uint32),
        ("sync_lo", ctypes.c_uint64), ("sync_hi", ctypes.c_uint64),
    ]


PATH_CAP = 400000
OTHER_CAP = 4096
CONSUME = ctypes.CFUNCTYPE(None, ctypes.c_int64, ctypes.c_int64, ctypes.c_int64)

PORT_IN = ctypes.CFUNCTYPE(ctypes.c_uint32, ctypes.c_uint32, ctypes.c_int, ctypes.c_uint64)
PORT_OUT = ctypes.CFUNCTYPE(None, ctypes.c_uint32, ctypes.c_int, ctypes.c_uint32, ctypes.c_uint64)
MMIO_READ = ctypes.CFUNCTYPE(ctypes.c_uint64, ctypes.c_int, ctypes.c_uint64, ctypes.c_uint, ctypes.c_uint64)
MMIO_WRITE = ctypes.CFUNCTYPE(None, ctypes.c_int, ctypes.c_uint64, ctypes.c_uint, ctypes.c_uint64, ctypes.c_uint64)
INTR_FALLBACK = ctypes.CFUNCTYPE(None, ctypes.c_uint32, ctypes.c_uint64)
BETWEEN = ctypes.CFUNCTYPE(None, ctypes.c_uint64)

_lib = None


def build():
    if (not os.path.exists(SO)) or os.path.getmtime(SO) < os.path.getmtime(SRC):
        cmd = [os.environ.get("CC", "gcc"), "-O2", "-shared", "-fPIC", "-Wall", "-I", INCLUDE, "-o", SO, SRC]
        subprocess.run(cmd, check=True)
    return SO


def lib():
    global _lib
    if _lib is None:
        path = build()
        L = ctypes.CDLL(path)
        L.fc_init.restype = ctypes.c_int
        L.fc_init.argtypes = [ctypes.c_void_p, ctypes.POINTER(State), ctypes.POINTER(ctypes.c_void_p),
                              ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_void_p),
                              ctypes.POINTER(ctypes.c_uint16), ctypes.c_uint32]
        L.fc_install_hooks.restype = ctypes.c_int
        L.fc_install_hooks.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int]
        L.fc_mmio_map.restype = ctypes.c_int
        L.fc_mmio_map.argtypes = [ctypes.c_uint64, ctypes.c_uint64, ctypes.c_int]
        L.fc_costs.restype = None
        L.fc_watch_set.restype = None
        L.fc_watch_set.argtypes = [ctypes.POINTER(ctypes.c_uint64), ctypes.c_int]
        L.fc_watch_get.restype = None
        L.fc_watch_get.argtypes = [ctypes.POINTER(ctypes.c_uint64)]
        L.fc_costs.argtypes = [ctypes.c_void_p, ctypes.c_uint64, ctypes.c_int64]
        L.fc_dispatch.restype = ctypes.c_int
        L.fc_dispatch.argtypes = [ctypes.c_uint32]
        L.fc_run.restype = ctypes.c_uint64
        L.fc_run.argtypes = [ctypes.c_uint64, ctypes.c_uint64, ctypes.c_uint64]
        L.fc_init_motion.restype = ctypes.c_int
        L.fc_init_motion.argtypes = [ctypes.POINTER(Motion), ctypes.POINTER(ctypes.c_int64), ctypes.c_uint32,
                                     ctypes.POINTER(ctypes.c_uint64), ctypes.c_uint32, ctypes.c_int, ctypes.c_void_p]
        L.fc_motion_read.restype = ctypes.c_uint64
        L.fc_motion_read.argtypes = [ctypes.c_uint64, ctypes.c_uint, ctypes.c_uint64]
        L.fc_motion_write.restype = None
        L.fc_motion_write.argtypes = [ctypes.c_uint64, ctypes.c_uint, ctypes.c_uint64, ctypes.c_uint64]
        L.fc_consume.restype = None
        L.fc_consume.argtypes = [ctypes.c_int64, ctypes.c_int64, ctypes.c_int64]
        L.fc_advance.restype = None
        L.fc_advance.argtypes = [ctypes.c_int64]
        L.fc_axis_advance.restype = ctypes.c_int64
        L.fc_axis_advance.argtypes = [ctypes.c_int, ctypes.c_int64]
        L.fc_axis_owe.restype = None
        L.fc_axis_owe.argtypes = [ctypes.c_int, ctypes.c_int64]
        _lib = L
    return _lib


def _addr(fn):
    return ctypes.cast(fn, ctypes.c_void_p).value


class Core(object):
    """One core per machine (the C side holds exactly one: one machine per process)."""

    def __init__(self, uc, port_in, port_out, mmio_read, mmio_write, intr_fallback, between):
        self.L = lib()
        self.state = State()
        self.state.resume_at = -1
        self.state.pic_lowest = 7          # 8259 reset state: IR0 highest priority
        self.log = (ctypes.c_uint16 * (3 * LOG_CAP))()
        u = _binding.uclib
        fns = (ctypes.c_void_p * 8)(_addr(u.uc_emu_start), _addr(u.uc_emu_stop), _addr(u.uc_reg_read),
                                    _addr(u.uc_reg_write), _addr(u.uc_mem_read), _addr(u.uc_mem_write),
                                    _addr(u.uc_hook_add), _addr(u.uc_mmio_map))
        regs = (ctypes.c_int * 5)(X.UC_X86_REG_CS, X.UC_X86_REG_IP, X.UC_X86_REG_SS,
                                  X.UC_X86_REG_SP, X.UC_X86_REG_EFLAGS)
        # the callbacks must stay alive as long as the core lives
        self._cbs = [PORT_IN(port_in), PORT_OUT(port_out), MMIO_READ(mmio_read),
                     MMIO_WRITE(mmio_write), INTR_FALLBACK(intr_fallback), BETWEEN(between)]
        cbs = (ctypes.c_void_p * 6)(*[_addr(c) for c in self._cbs])
        r = self.L.fc_init(uc._uch, ctypes.byref(self.state), fns, regs, cbs, self.log, LOG_CAP)
        if r:
            raise RuntimeError("fc_init: %d" % r)

    def init_motion(self, region, consume_cb):
        """Stage 2: motion window, counter and axes in C."""
        self.motion = Motion()
        self.path = (ctypes.c_int64 * (3 * PATH_CAP))()
        self.other = (ctypes.c_uint64 * (2 * OTHER_CAP))()
        self._consume = CONSUME(consume_cb)
        r = self.L.fc_init_motion(ctypes.byref(self.motion), self.path, PATH_CAP, self.other, OTHER_CAP,
                                  region, _addr(self._consume))
        if r:
            raise RuntimeError("fc_init_motion: %d" % r)
        return self.motion

    def path_list(self):
        n = self.motion.path_n
        p = self.path
        return [(p[3 * i], p[3 * i + 1], p[3 * i + 2]) for i in range(n)]

    def other_list(self):
        n = self.motion.other_n
        o = self.other
        return [(int(o[2 * i] & 0xFFFF), int(o[2 * i] >> 16), int(o[2 * i + 1])) for i in range(n)]

    def install_hooks(self):
        r = self.L.fc_install_hooks(UCC.UC_HOOK_INTR, UCC.UC_HOOK_INSN, X.UC_X86_INS_IN, X.UC_X86_INS_OUT)
        if r:
            raise RuntimeError("fc_install_hooks: %d" % r)

    def mmio_map(self, base, size, region):
        r = self.L.fc_mmio_map(base, size, region)
        if r:
            raise RuntimeError("fc_mmio_map: %d" % r)

    def set_costs(self, table, peer8):
        """Set the class cost table (bytes/bytearray, int8 per file offset of the image) and the peer surcharge."""
        self._cost_buffer = (ctypes.c_char * len(table)).from_buffer_copy(bytes(table))
        self.L.fc_costs(self._cost_buffer, ctypes.c_uint64(len(table)), ctypes.c_int64(peer8))

    def set_watch(self, addresses):
        """Load the watch addresses (sorted) into the core; the counters start at 0."""
        a = sorted(addresses)
        buf = (ctypes.c_uint64 * max(1, len(a)))(*a)
        self.L.fc_watch_set(buf, len(a))
        self._watch_addrs = a

    def get_watch(self):
        """Counters since the last fetch as {address: hits}; resets them to 0 in the core."""
        a = getattr(self, "_watch_addrs", [])
        if not a:
            return {}
        buf = (ctypes.c_uint64 * len(a))()
        self.L.fc_watch_get(buf)
        return dict(zip(a, buf))

    def run(self, at, max_instr, slice_size):
        return self.L.fc_run(at, max_instr, slice_size)

    def dispatch(self, intno):
        return self.L.fc_dispatch(intno)

    # ---- views for the scripts ------------------------------------------
    def pending_list(self):
        s = self.state
        return [s.pending[i] for i in range(s.npending)]

    def dispatch_log(self):
        n = self.state.log_n
        return [(self.log[3 * i], self.log[3 * i + 2], self.log[3 * i + 1]) for i in range(n)]

    def dropped_list(self):
        s = self.state
        return [s.dropped_log[i] for i in range(min(s.dropped_n, 64))]
