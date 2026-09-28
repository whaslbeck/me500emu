"""A ready-to-use machine: booted, in REMOTE, homed, and in the command mode you asked for.

`Session` is the high-level entry point for everything that talks to the emulated machine like a host would: the
pty bridge, the headless job runner and the tests. It wraps the steps every measurement needs:

    s = Session(rom="max", mode="gcode")      # boot from the snapshot cache, REMOTE, approach, Z0, G-code mode
    s.send(open("part.nc").read())            # stream it through the emulated UART with flow control
    print(s.position_mm(), s.lcd())

Coordinates reported here are TABLE coordinates in mm: X/Y from the LOW LEFT origin the machine homes to, Z as the
depth below the top of the Z travel (positive = down). `z0_mm` is the tool-surface zero (Z0) in the same frame.

Modes: the stock 1.50 firmware only knows HP-GL/MGL (mode "std"). A 1.50MAX image adds "opt" (HP-GL with look-ahead
path planning) and "gcode". Those two modes and the status cells in MAX_CELLS are specific to 1.50MAX builds and are
selected by poking the mode cell the panel's PATH PLAN menu writes.
"""
import hashlib
import os
import struct

import unicorn

from . import harness as H
from . import paths, rominfo
from . import snapshot_state as snap

# Instructions per byte on the serial line (10 bits per byte: start, 8 data, stop), at 2000 instructions per ms.
BAUD_INSTR = {9600: 2083, 19200: 1042, 38400: 521}
INSTR_PER_S = 2000000

# 1.50MAX RAM cells (build 000019). Not present in the stock firmware.
MAX_CELLS = {
    "mode": 0x1E04,            # PATH PLAN: 0x00 STD, 0xA5 OPT, 0x5A GCODE
    "debug": 0x1F70,           # DEBUG menu: 0x5A = ON (enables the ~ measurement commands)
    "stop": 0x1E79,            # G-code: job stopped (unknown command, limit, pause)
    "error": 0x1ED1,           # G-code: parser error count
    "overflow": 0x1E78,        # G-code: receive buffer overflow
    "line_overflow": 0x1ED7,   # G-code: line longer than the line buffer
    "dropped": 0x1FBE,         # G-code: bytes dropped (16 bit)
}
MODES = {"std": 0x00, "opt": 0xA5, "gcode": 0x5A}

# Factory firmware cells used for status and flow control
RING_HEAD, RING_TAIL = 0x1004, 0x1005     # executor ring: equal = no motion record queued
Z0_CELL = 0x04B6                          # Z0 set by the panel zero key, 5 um steps below the top of the Z travel
XY_ORIGIN_CELLS = (0xB500, 0xB504)        # plotter origin offset in table steps (5 um)
HLT_TRAP = 0x8450F                        # the factory firmware's halt loop (8000:450f): reached = machine stopped hard

DEFAULT_NVRAM = "device_example.bin"      # NVRAM of a real machine: rates, accelerations, ORIGIN LOW LEFT
SESSION_CACHE_VERSION = 1                 # bump when the setup sequence changes


class SessionError(RuntimeError):
    pass


class Session(object):
    """One emulated machine with a host-side view of it.

    rom       "stock", "max" or a file path (see paths.rom_path)
    mode      "std" | "opt" | "gcode"; "opt"/"gcode" need a 1.50MAX image
    baud      9600 (default, what the machine runs cleanly) | 19200 | 38400
    handshake "hard" (DTR flow control, the default) or "code" (XON/XOFF)
    nvram     NVRAM seed file (512 bytes); default: data/nvram/device_example.bin
    z0_mm     Z0 (work surface) in mm below the top of the Z travel, set like the panel zero key does.
              Default: 10 mm in G-code mode; in HP-GL modes the firmware's own value is left alone (a poked Z0
              there has driven the Z model into its end stop and stalled PD records in the test bench)
    origin_mm XY origin offset in mm (G-code frame; the default 5/5 mm leaves room for small negative moves)
    debug     DEBUG = ON on a 1.50MAX image (enables the ~ commands)
    rebuild   boot and set up again instead of using cached snapshots
    cache     keep the state after setup as a snapshot (session-<key>.snap in the cache directory)
    """

    def __init__(self, rom="stock", mode="std", baud=9600, handshake="hard", nvram=None, z0_mm=None,
                 origin_mm=(5.0, 5.0), debug=False, rebuild=False, cache=True):
        self.rom = paths.rom_path(rom)
        self.version = rominfo.version(self.rom)
        self.is_max = rominfo.is_max(self.rom)
        if mode not in MODES:
            raise SessionError("unknown mode %r (std, opt, gcode)" % mode)
        if mode != "std" and not self.is_max:
            raise SessionError("mode %s needs a 1.50MAX image, %s is %s" % (mode, self.rom, self.version or "unknown"))
        if debug and not self.is_max:
            raise SessionError("debug needs a 1.50MAX image")
        if baud not in BAUD_INSTR:
            raise SessionError("baud must be one of %s" % sorted(BAUD_INSTR))
        if handshake not in ("hard", "code"):
            raise SessionError("handshake must be hard or code")
        if z0_mm is None and mode == "gcode":
            z0_mm = 10.0
        self.mode, self.baud, self.handshake = mode, baud, handshake
        self.nvram = nvram or paths.data_path("nvram", DEFAULT_NVRAM)
        self.halted = 0

        # The setup below costs ~40 M instructions (about 20 s); its end state is cached like the boot snapshot.
        key = hashlib.sha1(repr((SESSION_CACHE_VERSION, snap.VERSION, hashlib.md5(open(self.rom, "rb").read()).hexdigest(),
                                 open(self.nvram, "rb").read(), mode, baud, handshake, z0_mm, tuple(origin_mm),
                                 debug)).encode()).hexdigest()[:16]
        self.cache_file = paths.cache_path("session-%s.snap" % key) if cache else None
        m = None
        if self.cache_file and not rebuild and os.path.exists(self.cache_file):
            m = self._restore(self.cache_file)
        if m is None:
            m = snap.booted(self.rom, physics=True, trace="fast", nvram_seed=self.nvram, rebuild=rebuild)
            if not m.from_snapshot:
                # A machine that continues straight from its cold boot differs in hidden core state (interrupt
                # phase, pending events) from one restored from the snapshot of that boot. Continue from the
                # restored one, so the result does not depend on whether the cache existed.
                m = snap.booted(self.rom, physics=True, trace="fast", nvram_seed=self.nvram)
            self._attach(m)
            self._setup(z0_mm, origin_mm, debug)
            if self.cache_file:
                snap.save(m, self.cache_file)
                m = self._restore(self.cache_file)       # same reason: every run starts from the loaded state
        self.strobes0 = m.motion.strobes
        self.instr0 = m.instr

    def _attach(self, m):
        self.m = m
        self.u = m.cpu.uc
        m.uart_byte_interval = BAUD_INSTR[self.baud]
        m.uart.honour_dtr = self.handshake == "hard"
        self.u.hook_add(unicorn.UC_HOOK_CODE, self._hlt, None, HLT_TRAP, HLT_TRAP)

    def _restore(self, path):
        from .machine import Machine
        m = Machine(self.rom, physics=True, trace="fast", nvram_seed=self.nvram)
        m.subcpu.sign_window()
        try:
            snap.load(m, path)
        except snap.StaleSnapshot:
            return None
        m.arm_costs()
        self._attach(m)
        return m

    def _setup(self, z0_mm, origin_mm, debug):
        """Same frame as the validated test setup: look-ahead mode (or STD) while homing, REMOTE, approach, idle."""
        m, u, mode = self.m, self.u, self.mode
        if self.is_max:
            u.mem_write(MAX_CELLS["mode"], bytes([MODES["opt"] if mode != "std" else MODES["std"]]))
        if not H.remote(m):
            raise SessionError("REMOTE not reached: %s" % self.lcd())
        m.uart.rx.extend(H.APPROACH)
        H.drain(m, 40000000)
        for _ in range(200):
            if self.ring_empty() and H.drain(m, 4000000, quiet=6):
                break
            m.run(m.instr + 4000000)
        if z0_mm is not None:
            H.set_z0_surface(m, int(round(z0_mm * 200)))
        if mode == "gcode":
            u.mem_write(MAX_CELLS["mode"], bytes([MODES["gcode"]]))
            m.run(m.instr + 3000000)
            for cell, mm in zip(XY_ORIGIN_CELLS, origin_mm):
                u.mem_write(cell, int(round(mm * 200)).to_bytes(2, "little"))
        if debug:
            u.mem_write(MAX_CELLS["debug"], b"\x5a")
        if self.handshake == "code":
            # [0x0828] is the settings shadow (1 HARD, 2 CODE); the receive reset 8000:b49a reloads [0x2040] from it
            u.mem_write(0x0828, b"\x02")
            u.mem_write(0x2040, b"\x02"); u.mem_write(0x2042, b"\x01\x13"); u.mem_write(0x204D, b"\x01\x11")
            m.uart.honour_dtr = False

    # ---- state ---------------------------------------------------------------------------------------------
    def _hlt(self, uc, addr, size, ud):
        self.halted += 1
        uc.emu_stop()

    def byte(self, addr):
        return self.u.mem_read(addr, 1)[0]

    def word(self, addr):
        return struct.unpack("<H", bytes(self.u.mem_read(addr, 2)))[0]

    def ring_empty(self):
        return self.byte(RING_HEAD) == self.byte(RING_TAIL)

    def position_mm(self):
        """(x, y, z) table coordinates in mm, from the physical axis model."""
        a = self.m.subcpu.axes
        return (round(a["X"].table_mm(), 3), round(a["Y"].table_mm(), 3), round(a["Z"].table_mm(), 3))

    def z0_mm(self):
        return struct.unpack("<i", bytes(self.u.mem_read(Z0_CELL, 4)))[0] / 200.0

    def xy_origin_mm(self):
        """The firmware's XY origin offset (plotter origin cells) in table mm."""
        return tuple(self.word(c) / 200.0 for c in XY_ORIGIN_CELLS)

    def lcd(self):
        return " | ".join(r.rstrip() for r in self.m.panel.text())

    def time_s(self):
        """Emulated time since the session became ready."""
        return (self.m.instr - self.instr0) / float(INSTR_PER_S)

    def status(self):
        """Firmware status: strobes since ready, halt trap, UART overruns, and the 1.50MAX G-code counters."""
        st = dict(strobes=self.m.motion.strobes - self.strobes0, halted=self.halted,
                  uart_overruns=int(self.m.uart_overruns))
        if self.is_max:
            for k in ("stop", "error", "overflow", "line_overflow"):
                st[k] = self.byte(MAX_CELLS[k])
            st["dropped"] = self.word(MAX_CELLS["dropped"])
        return st

    # ---- actions -------------------------------------------------------------------------------------------
    def run(self, instr):
        self.m.run(self.m.instr + int(instr))

    def settle(self, limit=400000000):
        """Run until the axes have stood still for a while. False if they do not within `limit` instructions."""
        return H.drain(self.m, 30000000, limit=limit)

    def press(self, col, bit, hold=None, after=None):
        """Press and release one panel key (matrix column, bit); see docs/hardware-model.md for the key map."""
        m = self.m
        m.panel.press(col, bit)
        m.run(m.instr + (hold or H.KEY_HOLD))
        m.panel.release_all()
        m.run(m.instr + (after or H.KEY_GAP))

    def hold_until(self, col, bit, condition, max_steps=900):
        """Hold a key (jog) until condition() is true, then release."""
        m = self.m
        for _ in range(max_steps):
            m.panel.press(col, bit)
            m.run(m.instr + 200000)
            if condition():
                break
        m.panel.release_all()
        m.run(m.instr + H.KEY_GAP)

    def read_tx(self):
        """Bytes the machine has sent since the last call."""
        tx = bytes(self.m.uart.tx)
        self.m.uart.tx.clear()
        return tx

    def feed(self, data, window=16):
        """Move up to `window - queued` bytes of `data` into the UART's wire; returns how many were taken.
        Like a real line: the host cannot push more than a few bytes past the machine's flow control."""
        room = window - len(self.m.uart.rx)
        if room <= 0 or not data:
            return 0
        chunk = data[:room]
        self.m.uart.rx.extend(chunk)
        return len(chunk)

    def send(self, data, settle=True, limit=None, on_step=None):
        """Stream `data` (str or bytes) through the UART with flow control, then optionally wait for standstill.
        Returns False if the machine halted or did not come to rest."""
        if isinstance(data, str):
            data = data.encode("latin-1")
        pos, m = 0, self.m
        end = m.instr + limit if limit else None
        while pos < len(data):
            pos += self.feed(data[pos:])
            m.run(m.instr + 20000)
            if on_step:
                on_step(self)
            if self.halted or (end and m.instr > end):
                return False
        if settle:
            return self.settle()
        return True
