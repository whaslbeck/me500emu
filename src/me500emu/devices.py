"""Stage C devices.

Every behaviour here is traceable to a model document; where the firmware's
expectation is inferred rather than read, the code says so, because a device
that quietly returns a plausible value is exactly the failure this emulator
exists to avoid.
"""
import os
import struct


class Nvram(object):
    """Memory-mapped at 0x24000, byte-wide, EVEN offsets only (DI is doubled).

    512 cells. Boot verifies an 8-bit sum over cells 0x000..0x03f against the
    byte at cell 0x1ff (0x80387) and reports a configuration error on mismatch,
    so the image must be valid or the gate cannot be met.
    """
    SIZE = 0x400
    CELLS = 0x200

    def __init__(self, rom, log, path=None, seed=None):
        """rom -> factory contents; seed -> a stored configuration; path -> live file.

        The three are applied in that order, so `seed` is the machine's standard
        configuration (checked in, read-only) and `path` is the mutable image the
        UI writes back on exit. A missing file at either is not an error - the
        layer below it stands.
        """
        self.cells = bytearray(self.CELLS)
        self.log = log
        self.write_enabled = False
        self.odd_accesses = []
        self.path = path
        self.dirty = False
        self.seed_from_rom(rom)
        if seed:
            self.load(seed)
        if path:
            self.load(path)

    # Cell blocks, from the seven callers of the NVRAM transfer 8000:1b79:
    #   0x000 x0x40 <-> [0x0680]  model record   (checksummed at cell 0x1ff)
    #   0x100 x0x14 <-> [0x0800]
    #   0x140 x0x18 <-> [0x09ae]  the rate root block
    #   0x180 x0x0e <-> [0x0a7c]
    #   0x1c0 x0x08 <-> [0x04ae]
    #   0x1e0 x0x02 <-> [0x08b0]
    #   0x1ff x1    <-> [0x0737]  checksum
    RATE_ROOT_CELL = 0x140          # <-> RAM [0x09ae], 0x18 bytes

    # Cells 0x000..0x03f are the 32 SYSTEM PARAMETERS, two bytes each - the
    # Service Manual lists them with names and initial values, and the match
    # against ROM 8000:2740 holds. This block used to be called the "model
    # record" here; not wrong, but far too unspecific.
    #
    # Checked: 9 of 12 checkable values match directly, and the three
    # deviations confirm the mapping all the more. No. 18/19 are listed in the
    # manual as 150/-100 with a footnote: "With the ROM version 2.00 or
    # more, set the values for 10 times as these values" - our version is
    # 1.50, and the ROM carries 15/-10. No. 29 is the LCD language, English in
    # the ROM instead of the factory default Japanese.
    PARAMETERS = {
        0:  ("Encoder resolution", "flatness encoder, resolution = 3.25 um x value"),
        1:  ("Cutting amount correction", "with flatness ON or AUTO"),
        2:  ("Height for flatness adjustment", "unit 5 um"),
        15: ("Automatic buffer clear switch", "1 = clear receive buffer after 10 s"),
        16: ("Flatness sensor check path switch", "0 = check"),
        17: ("Chip removal attachment max stroke", "mm"),
        18: ("Valid area X reference point", "mm; x10 from ROM 2.00 on"),
        19: ("Valid area Y reference point", "mm; x10 from ROM 2.00 on"),
        20: ("Valid area Z reference point", "mm"),
        21: ("Maximum valid area X", "mm - 310 ME-300, 483 ME-500, 650 ME-650"),
        22: ("Maximum valid area Y", "mm - 220 / 305 / 440"),
        23: ("Maximum valid area Z", "mm"),
        24: ("X axis distance correction", "relative to 300 mm, unit 0.05 mm"),
        25: ("Y axis distance correction", "relative to 300 mm, unit 0.05 mm"),
        27: ("Hash check switch", "0 = check"),
        28: ("Maintenance test enable", "1 = maintenance mode open"),
        29: ("LCD display language", "0 = Japanese, 1 = English"),
        30: ("Model selection switch", "0 = ME-300, 1 = ME-500, 2 = ME-650"),
        31: ("Initialize switch", "1 = write parameters back to the EEPROM"),
    }

    def parameters(self):
        """The 32 system parameters as (number, value, name) - named where the
        Service Manual names them."""
        out = []
        for i in range(32):
            v = self.cells[2*i] | (self.cells[2*i+1] << 8)
            if v & 0x8000:
                v -= 0x10000
            name = self.PARAMETERS.get(i, ("Reserve" if 3 <= i <= 14 else "?", ""))[0]
            out.append((i, v, name))
        return out

    def seed_from_rom(self, rom):
        # The 32 system parameters, from 8000:2740 - for this machine that is
        # 483 x 305 mm travel (no. 21/22) and ME-500 (no. 30 = 1).
        rec = rom[0x2740:0x2780]
        self.cells[0x00:0x40] = rec
        self.cells[0x1ff] = sum(rec) & 0xFF          # config_checksum_8bit_0680
        self.seed_rate_root()

    def seed_rate_root(self):
        """Seed the rate root block at cell 0x140 (<-> RAM [0x09ae], 0x18 bytes).

        EMULATOR ASSUMPTION, not a firmware fact.  The ROM contains NO factory
        defaults for this block - the only 0x80-byte copy near the config read
        (aa62:0db7 -> 8000:1c6f) is RAM [0x0680] -> RAM [0x06e8] - so the block
        is programmed at the factory or through the panel CONDITION menu.  With
        an all-zero NVRAM the executor divides by zero at 83cc8 and takes the
        INT 0 panic at 8000:048a.

        rate_root_loader_d190 (ad190, real CS ad19) copies the block as two
        parallel 4-word records plus two more words:
            A = [0x09ae..0x09b4] -> [0x09f6..0x09fc]
            B = [0x09b6..0x09bc] -> [0x09fe..0x0a04]
            L = [0x09be..0x09c0] -> [0x0a06..0x0a08]
        4a06 takes its DIV dividend from A[3] and its divisor from B[3]; another
        branch at 84adb takes the A[1]/B[1] pair.

        The four speeds come from the ME-500 operation manual, section CONDITION
        (defaults printed on the LCD): XY-ES 1 mm/s, Z-ES 10 mm/s, XY-MS
        80 mm/s, Z-MS 30 mm/s - matching the ROM's own menu strings at 8df08.
        B[3] = 30 also matches the seek builder 8181c, which hardwires divisor
        0x1e at 81860.  L is set to the travel limits 483 x 305 mm, the same
        numbers the model record carries at [0x06aa]/[0x06ac].
        """
        # CORRECTION 2026-08-31: the mapping was swapped. **A carries the four
        # speeds** in 0.1 mm/s - proven twice: the settings measurement writes
        # 30 to [0x09ae] = A[0] when XY-ES is changed to 3 mm/s (firmware
        # analysis), and in operation [0x1022] = A[0] = 200, which the profile
        # builder turns into [0x10c4] = 2000 - and the LCD shows "XYES:20" with
        # it. What B is remains open.
        #
        # CONSEQUENCE, and it is a known inaccuracy: all four speeds here are
        # 20.0 mm/s, because A was seeded with a flat 200. The manual's factory
        # defaults would be XY-ES 1, Z-ES 10, XY-MS 80, Z-MS 30 mm/s - those sit
        # wrongly in B. Correcting that would shift every timing measurement of
        # this project and invalidate the snapshots; it therefore stays until
        # the time scale has been measured on the real machine.
        A = [200, 200, 200, 200]        # XY-ES, Z-ES, XY-MS, Z-MS in 0.1 mm/s
        # B[1] and B[3] are the Z accelerations: they are the divisor of the
        # ramp computation at 8000:3cb7 and end up in record field +0x4e - B[1]
        # on plunge, B[3] on retract. Proven by changing one cell at a time
        # (firmware analysis): B[1] 10 -> 25 shortens the plunge ramp from 20
        # to 8 ticks, B[3] 30 -> 45 the retract ramp from 7 to 5. B[0] and B[2]
        # remain open.
        B = [1, 10, 80, 30]             # [?, Z engrave accel., ?, Z rapid accel.]
        L = [483, 305]                  # travel limits, mm
        words = A + B + L
        base = self.RATE_ROOT_CELL
        for k, val in enumerate(words):
            self.cells[base + k*2] = val & 0xFF
            self.cells[base + k*2 + 1] = (val >> 8) & 0xFF

    def read(self, off, size, pc):
        if off & 1:
            self.odd_accesses.append((off, pc))
        cell = off >> 1
        return self.cells[cell] if cell < self.CELLS else 0

    def write(self, off, size, value, pc):
        if not self.write_enabled:
            # port 0x0a bits 6-7 protect the part; a write while protected is a
            # finding, not something to absorb silently
            self.log.append(("nvram", "write while protected", off, value, pc))
            return
        cell = off >> 1
        if cell < self.CELLS:
            if self.cells[cell] != (value & 0xFF):
                self.dirty = True
            self.cells[cell] = value & 0xFF

    # ---- persistence ---------------------------------------------------
    # The real part keeps its contents across a power cycle, so a setting made
    # through the panel must survive a restart of the emulator too. The file is
    # the 512 cells raw, one byte each - the same view the firmware has, not the
    # doubled byte lane the bus shows.
    def load(self, path):
        try:
            with open(path, "rb") as f:
                blob = f.read()
        except IOError:
            return False
        if len(blob) != self.CELLS:
            raise ValueError("NVRAM file %s has %d bytes, expected %d"
                             % (path, len(blob), self.CELLS))
        self.cells[:] = blob
        self.dirty = False
        return True

    def save(self, path=None):
        path = path or self.path
        if not path:
            return False
        tmp = path + ".tmp"
        with open(tmp, "wb") as f:
            f.write(bytes(self.cells))
        os.replace(tmp, path)
        self.dirty = False
        return True

    def checksum_ok(self):
        """Boot's own gate: 8-bit sum over cells 0x000..0x03f vs cell 0x1ff."""
        return (sum(self.cells[0x00:0x40]) & 0xFF) == self.cells[0x1ff]


class Panel(object):
    """Ports 0x04 / 0x07 / 0x08 / 0x09 - an HD44780-style LCD plus a key matrix.

    Port 0x09 is a control latch. Read off the two writer routines:
      bit 7 (0x80) E strobe        - 80ea3 sets, 80eab clears
      bit 6 (0x40) R/W             - 80eb9/80ec1 set it to read the busy flag
      bit 5 (0x20) RS              - 80f2d sets it for DATA, the command path
                                     (80e94) leaves it clear
      bits 2-4     key column      - 80a06/80a19/80a2e, three scan phases
    The byte is placed on 0x04 first and latched on the FALLING edge of E.

    Port 0x04 read returns the status; bit 7 is BUSY and 80eaf spins on it.
    """
    def __init__(self, log):
        self.log = log
        self.ctrl = 0
        self.pending = 0
        self.cmds, self.data = [], []
        self.ddram = bytearray(b" " * 128)
        self.ac = 0
        # Three scan columns, selected by port 0x09 bits 2/3/4 (80a06/80a19/
        # 80a2e). Active low, so 0xFF means nothing pressed in that column.
        self.keys = [0xFF, 0xFF, 0xFF]

    def port_read(self, port, pc):
        if port == 0x04:
            # bit 7 = BUSY, always clear: the real controller needs ~40us and
            # the emulator has no clock, so a busy phase would only add a spin.
            return 0x00
        if port == 0x08:
            col = {0x04: 0, 0x08: 1, 0x10: 2}.get(self.ctrl & 0x1C)
            return self.keys[col] if col is not None else 0xFF
        if port == 0x09:
            return self.ctrl
        return 0

    def _latch(self, rs, value):
        if rs:
            if self.ac < len(self.ddram):
                self.ddram[self.ac] = value
            self.ac += 1
            self.data.append(value)
        else:
            self.cmds.append(value)
            if value & 0x80:                 # set DDRAM address
                self.ac = value & 0x7F
            elif value in (0x01, 0x02):      # clear / home
                if value == 0x01:
                    self.ddram = bytearray(b" " * 128)
                self.ac = 0

    def port_write(self, port, value, pc):
        value &= 0xFF
        if port == 0x09:
            prev = self.ctrl
            self.ctrl = value
            if (prev & 0x80) and not (value & 0x80):
                self._latch(prev & 0x20, self.pending)
        elif port == 0x04:
            self.pending = value
        elif port == 0x07:
            self.log.append(("panel", "ctrl word", port, value, pc))

    def press(self, col, bit):
        self.keys[col] &= ~(1 << bit) & 0xFF

    def release_all(self):
        self.keys = [0xFF, 0xFF, 0xFF]

    # The operation manual (D202838-13, "Operation Panel") states the display
    # is 16 columns x 4 lines. The emulator previously assumed 40x2, which
    # garbled every screen after the banner.
    COLS, ROWS = 16, 4
    ROW_BASE = (0x00, 0x40, 0x10, 0x50)      # the usual HD44780 4-line layout

    def text(self, cols=None):
        cols = cols or self.COLS
        out = []
        for base in self.ROW_BASE[:self.ROWS]:
            row = self.ddram[base:base + cols]
            out.append("".join(chr(c) if 32 <= c < 127 else " " for c in row).rstrip())
        return out

class Sensors(object):
    """Port 0x06. Bit map from the firmware analysis.

    The idle level of each bit is NOT established by the firmware analysis, so
    it is a parameter, and any wait-loop that spins on it is reported rather
    than hidden.
    """
    def __init__(self, log, value=0xFF):
        self.log = log
        self.value = value
        self.reads = 0

    def port_read(self, port, pc):
        self.reads += 1
        return self.value


class Counter(object):
    """Ports 0x30 / 0x34, read with a latch protocol:
    IN 0x30 latches, IN 0x34 gives the high byte, IN 0x30 the low byte.

    This is the FLATNESS SENSOR's rotary encoder (service manual: ENC-A/ENC-B are the flatness encoder; the service
    screen shows it as FLAT; NVRAM parameter 0 is its resolution). The firmware samples it on every motion tick
    (8000:4f6f) and turns it into a height correction (8000:4f7f, 4f29) - effective only with FLATNESS ON/AUTO.

    Until 2026-10-07 the model accumulated the strobes here and called it motion feedback, on the belief that seeks
    never ended otherwise. Measured on a machine without the sensor, the counter stays 0 during a 20 mm move, and the
    emulator boots and runs jobs identically with 0 (FLATNESS OFF). `source` decides what the firmware reads:
      None       no flatness sensor fitted: 0 (default, as measured)
      "strobes"  the old model: the accumulated strobes (`value`), for comparisons
      callable   a surface model: returns the raw 16-bit encoder count
    MotionRegs still advances `value` on every strobe.
    """
    source = None

    def __init__(self, log):
        self.log = log
        self.value = 0
        self.latched = 0

    def advance(self, steps):
        self.value = (self.value + steps) & 0xFFFF

    def raw(self):
        if self.source is None:
            return 0
        if self.source == "strobes":
            return self.value & 0xFFFF
        return int(self.source()) & 0xFFFF

    def port_read(self, port, pc):
        if port == 0x30:
            self.latched = self.raw()
            return self.latched & 0xFF
        if port == 0x34:
            return (self.latched >> 8) & 0xFF
        return 0


class RxQueue(bytearray):
    """The host's receive buffer. Every change reports the length to the C glue
    layer (fastcore), which derives the byte pacing from it."""
    def __init__(self, *a, **k):
        super(RxQueue, self).__init__(*a, **k)
        self.on_change = None

    def _sync(self):
        if self.on_change is not None:
            self.on_change(len(self))

    def extend(self, b):
        super(RxQueue, self).extend(b); self._sync()

    def append(self, b):
        super(RxQueue, self).append(b); self._sync()

    def pop(self, *a):
        v = super(RxQueue, self).pop(*a); self._sync(); return v

    def clear(self):
        super(RxQueue, self).clear(); self._sync()

    def __delitem__(self, k):
        super(RxQueue, self).__delitem__(k); self._sync()

    def __iadd__(self, b):
        r = super(RxQueue, self).__iadd__(b); self._sync(); return r

    def insert(self, i, b):
        super(RxQueue, self).insert(i, b); self._sync()


class Uart(object):
    """Ports 0x18 (data) / 0x19 (status). Status bits drive the receive ISR."""
    RX_READY = 0x02
    TX_READY = 0x01

    def __init__(self, log, state=None):
        self.log = log
        self._st = state                     # fastcore state (mirror of rx length, ctrl, honour_dtr)
        self.rx = RxQueue()
        if state is not None:
            self.rx.on_change = lambda n: setattr(state, "uart_rx_len", n)
        self._tx = bytearray()
        self._ctrl = 0
        self._honour = True
        # 8251 transmit side (2026-09-24, workshop run with image c3b219e4): holding register + shifter.
        # A byte written to port 0x18 sits in the holding register (TxRDY = 0) until the shifter is free
        # AND TxEN is set; without TxEN it gets stuck - the factory ISR IRQ2 clears TxEN at the end of
        # its queue (8000:c440), and a probe delivered by software (int 21h, no in-service bit) thus
        # stalled in pr_warte on the machine; the stuck character came out before the next reply
        # ("!!F", ":!F"). Before, the model always reported TxRDY as 1 and discarded bytes without TxEN.
        self._tx_hold = -1                   # holding register: -1 empty, else the waiting byte (mirrored in C state)
        self._tx_free_at = 0                 # instruction count from which the shifter is free again
        if state is not None:
            state.uart_tx_hold = -1; state.uart_tx_free_at = 0
        self.tx_overwritten = 0              # bytes that overwrote a full holding register (lost)
        self.tx_overwritten_log = []         # (pc, instr count, old byte, new byte, pc of old) of the first 20
        self._tx_hold_pc = 0                 # who wrote the held byte
        self.machine = None                 # time source (instr, uart_byte_interval); None = unpaced
        self._ctrl_prev = None               # command before the last write to 0x19 (the C core mirrors the
                                             # new value into uart_ctrl BEFORE the Python call)
        # 8251 receive register: a byte only counts as arrived once the pacing has delivered it
        # (deliver); a second byte before the read overwrites it (OE, status bit 4, until ER)
        self._delivered = 0
        self._oe = 0
        self._drop = 0
        self._last = 0
        if state is not None:
            self.honour_dtr = True
            self.ctrl = 0
        # The host honours DTR (bit 1 of the 8251 command, firmware analysis):
        # as long as the firmware has dropped it, it delivers no byte. Until the
        # first command byte (ctrl == 0, e.g. after an old snapshot) DTR counts
        # as set. `honour_dtr = False` models a host without handshake.
        self.dtr_writes = []
        self.tx_without_txen = 0             # bytes written with TxEN = 0: the 8251 does not send them -
                                             # the probe would have stayed silent on the machine while
                                             # the test bench printed it

    @property
    def ctrl(self):
        return self._st.uart_ctrl if self._st is not None else self._ctrl

    @ctrl.setter
    def ctrl(self, v):
        if self._st is not None:
            self._st.uart_ctrl = v & 0xFF
        else:
            self._ctrl = v & 0xFF

    @property
    def honour_dtr(self):
        return bool(self._st.uart_honour_dtr) if self._st is not None else self._honour

    @honour_dtr.setter
    def honour_dtr(self, v):
        if self._st is not None:
            self._st.uart_honour_dtr = 1 if v else 0
        else:
            self._honour = bool(v)

    @property
    def dtr(self):
        return self.ctrl == 0 or (self.ctrl & 0x02) != 0

    def _get(self, name):
        return getattr(self._st, "uart_" + name) if self._st is not None else getattr(self, "_" + name)

    def _set(self, name, v):
        if self._st is not None:
            setattr(self._st, "uart_" + name, v)
        else:
            setattr(self, "_" + name, v)

    delivered = property(lambda self: self._get("delivered"), lambda self, v: self._set("delivered", v))
    tx_hold = property(lambda self: self._get("tx_hold"), lambda self, v: self._set("tx_hold", v))
    tx_free_at = property(lambda self: self._get("tx_free_at"), lambda self, v: self._set("tx_free_at", v))
    oe = property(lambda self: self._get("oe"), lambda self, v: self._set("oe", v))
    drop = property(lambda self: self._get("drop"), lambda self, v: self._set("drop", v))

    def deliver(self):
        """A byte arrives (Python core; the C core does the same in fc_run). True = overrun."""
        lost = False
        if self.delivered:
            self.oe = 1; self.drop = self.drop + 1; lost = True
        self.delivered = 1
        return lost

    def port_read(self, port, pc):
        if port == 0x18:
            while self.drop and len(self.rx):
                self.rx.pop(0); self.drop = self.drop - 1     # the overwritten byte
            if not self.delivered:
                return self._last                             # nothing new: the 8251 returns the old byte
            self.delivered = 0
            self._last = self.rx.pop(0) if self.rx else 0
            return self._last
        if port == 0x19:
            self.tx_advance()
            s = self.TX_READY if self.tx_hold < 0 else 0
            if self.tx_hold < 0 and self._now() >= self.tx_free_at:
                s |= 0x04                                     # TxEMPTY: holding register and shifter empty
            if self.delivered:
                s |= self.RX_READY
            if self.oe:
                s |= 0x10
            return s
        return 0

    def port_write(self, port, value, pc):
        if port == 0x18:
            self.tx_advance()
            if self.tx_hold >= 0:
                self.tx_overwritten += 1                   # holding register full: the old byte is lost
                if len(self.tx_overwritten_log) < 20:
                    self.tx_overwritten_log.append((pc, self._now(), self.tx_hold, value & 0xFF, self._tx_hold_pc))
            if self.ctrl and not (self.ctrl & 0x01):
                self.tx_without_txen += 1                        # written with TxEN = 0: gets stuck
            self.tx_hold = value & 0xFF; self._tx_hold_pc = pc
            self.tx_advance()
        elif port == 0x19:
            old = self._ctrl_prev if self._ctrl_prev is not None else self.ctrl
            self.tx_advance(old)                              # advance with the old command (TxEN) ...
            self.ctrl = value & 0xFF
            self._ctrl_prev = value & 0xFF
            if value & 0x10:                                  # ER: error reset clears OE
                self.oe = 0
            self.dtr_writes.append((pc, self.ctrl))
            self.tx_advance()                              # ... and with the new one (TxEN just set)

    def _now(self):
        return self.machine.instr if self.machine is not None else 0

    def tx_advance(self, ctrl=None):
        """Holding register -> shifter when the shifter is free and TxEN is set (ctrl 0 = no command
        yet = treated as set). Lazy on every port access and on reading `tx`; time is the machine's
        instruction counter, which advances in slices (<= 2000 instructions = 1 ms)."""
        if self.tx_hold < 0:
            return
        c = self.ctrl if ctrl is None else ctrl
        if c and not (c & 0x01):
            return
        j = self._now()
        if j < self.tx_free_at:
            return
        self._tx.append(self.tx_hold); self.tx_hold = -1
        self.tx_free_at = j + (self.machine.uart_byte_interval if self.machine is not None else 0)

    @property
    def tx(self):
        self.tx_advance()
        return self._tx

    @tx.setter
    def tx(self, v):
        self._tx = v


class Pic(object):
    """8259 at 0x10/0x11 - what the firmware programs at 8000:01f0 (firmware analysis):
    ICW1 0x13 (edge, single, ICW4 follows), ICW2 0x20 (IRQ0..7 = INT 20h..27h),
    ICW4 0x01 (8086 mode, NO auto-EOI), OCW1 0xe0 (IRQ5..7 masked).

    Fully nested mode: while an IRQ is in service, only a higher-priority (lower
    number) request is delivered; the same level and lower ones wait in IRR until
    the non-specific EOI (`out 0x10,0x20`, 8000:b5e6) clears the highest in-service
    bit. Without this model the emulator delivered IRQ1 inside the IRQ1 handler
    (the "ISR re-entry" seen in the firmware analysis) and let a handler that skipped
    the EOI run unpunished - on the machine that leaves IRQ1..IRQ7 blocked for good.
    OCW3 0x0a/0x0b selects IRR/ISR for a read of port 0x10 (the DEBUG counters use 0x0b).
    """

    def __init__(self, log, pending=None, state=None, pending_fn=None):
        self.log = log
        self._st = state            # fastcore: state lives in the C struct
        self._pending_fn = pending_fn
        self._v = dict(isr=0, imr=0, read_isr=False, icw_left=0, eois=0, blocked=0, lowest=7)
        self._pending = pending if pending is not None else []

    def _mk(name, cname, cast):
        def get(self):
            return cast(getattr(self._st, cname)) if self._st is not None else self._v[name]

        def set_(self, v):
            if self._st is not None:
                setattr(self._st, cname, int(v))
            else:
                self._v[name] = v
        return property(get, set_)
    isr = _mk("isr", "pic_isr", int)
    imr = _mk("imr", "pic_imr", int)
    read_isr = _mk("read_isr", "pic_read_isr", bool)
    lowest = _mk("lowest", "pic_lowest", int)

    def prio(self, irq):
        """0 = highest priority; default IR0 (lowest = 7), after OCW2 0xC0|n, n is the lowest."""
        return (irq - self.lowest - 1) & 7

    def _eoi_nonspecific(self):
        best = None
        for j in range(8):
            if self.isr & (1 << j) and (best is None or self.prio(j) < self.prio(best)):
                best = j
        if best is not None:
            self.isr &= ~(1 << best)
        self.eois += 1
    icw_left = _mk("icw_left", "pic_icw_left", int)
    eois = _mk("eois", "pic_eois", int)
    blocked = _mk("blocked", "pic_blocked", int)
    del _mk

    @property
    def pending(self):
        return self._pending_fn() if self._pending_fn is not None else self._pending

    def port_write(self, port, value, pc):
        v = value & 0xFF
        if port == 0x10:
            if v & 0x10:                       # ICW1
                self.icw_left = 1 + (1 if v & 0x01 else 0) + (0 if v & 0x02 else 1)
                self.isr = 0
                self.read_isr = False
                self.lowest = 7
            elif v & 0x08:                     # OCW3
                if v & 0x02:
                    self.read_isr = bool(v & 0x01)
            else:                              # OCW2
                kind = v & 0xE0
                if kind == 0x20:               # non-specific EOI
                    self._eoi_nonspecific()
                elif kind == 0x60:             # specific EOI
                    self.isr &= ~(1 << (v & 7))
                    self.eois += 1
                elif kind == 0xC0:             # set priority: v&7 becomes lowest
                    self.lowest = v & 7
                elif kind == 0xA0:             # rotate on non-specific EOI
                    best = None
                    for j in range(8):
                        if self.isr & (1 << j) and (best is None or self.prio(j) < self.prio(best)):
                            best = j
                    self._eoi_nonspecific()
                    if best is not None:
                        self.lowest = best
                elif kind == 0xE0:             # rotate on specific EOI
                    self.isr &= ~(1 << (v & 7))
                    self.eois += 1
                    self.lowest = v & 7
        else:
            if self.icw_left:
                self.icw_left -= 1             # ICW2 / ICW4 - vector base 0x20 is assumed
            else:
                self.imr = v                   # OCW1

    def port_read(self, port, pc):
        if port == 0x10:
            if self.read_isr:
                return self.isr
            irr = 0
            for vec in self.pending:
                if 0x20 <= vec <= 0x27:
                    irr |= 1 << (vec - 0x20)
            return irr
        return self.imr

    def acceptable(self, vec):
        irq = vec - 0x20
        if not 0 <= irq <= 7:
            return True
        if self.imr & (1 << irq):
            return False
        return not any(self.isr & (1 << j) and self.prio(j) <= self.prio(irq) for j in range(8))   # nothing of equal/higher priority in service

    def dispatch(self, vec):
        irq = vec - 0x20
        if 0 <= irq <= 7:
            self.isr |= 1 << irq

    def dispatch_default7(self):
        """8259A: request gone before INTA -> default vector IR7 without in-service bit."""
        self.default7 = getattr(self, "default7", 0) + 1


class PageRegisters(object):
    """0xff00..0xff7e: 64 registers, one per 16 KiB logical page.
    0xff80 bit 0 reports whether expanded addressing is currently ACTIVE - the
    firmware leaves XA before reprogramming a register and re-enters afterwards
    (peer_wait_ready_and_write_dx_ax, 8000:0568).
    """
    def __init__(self, cpu, log):
        self.cpu = cpu
        self.log = log
        self.regs = [0] * 64
        self.writes = 0

    def port_read(self, port, pc):
        if port == 0xff80:
            return 1 if self.cpu.xa_mode else 0
        if 0xff00 <= port < 0xff80:
            return self.regs[(port - 0xff00) >> 1]
        return 0

    def port_write(self, port, value, pc):
        if 0xff00 <= port < 0xff80:
            self.regs[(port - 0xff00) >> 1] = value & 0xFFFF
            self.writes += 1


class PagedStore(object):
    """The backing store behind the four windows.

    A window access at logical address L is translated through the page
    register for that 16 KiB page:

        physical = pages.regs[L >> 14] * 0x4000 + (L & 0x3fff)

    Size is a parameter, not a fact: the firmware displays "1 [MB]" from a ROM
    constant, and nothing in the image states the part's real capacity. Reads
    of a page never written are reported, because that is how a wrong page
    geometry would announce itself.
    """
    def __init__(self, pages, log, size=1 << 20):
        self.pages = pages
        self.log = log
        self.size = size
        self.mem = bytearray(size)
        self.written_pages = set()
        self.unwritten_reads = []

    def _phys(self, logical):
        idx = (logical >> 14) & 0x3F
        return (self.pages.regs[idx] * 0x4000 + (logical & 0x3FFF)) % self.size

    def read(self, logical, size, pc):
        p = self._phys(logical)
        if (p >> 14) not in self.written_pages:
            self.unwritten_reads.append((logical, p, pc))
        return self.mem[p] if size == 1 else int.from_bytes(
            self.mem[p:p + size], "little")

    def write(self, logical, size, value, pc):
        p = self._phys(logical)
        self.written_pages.add(p >> 14)
        if size == 1:
            self.mem[p] = value & 0xFF
        else:
            self.mem[p:p + size] = (value & ((1 << (size * 8)) - 1)).to_bytes(size, "little")


class MotionRegs(object):
    """The motion register file at segment 0x2000.

    Three axis groups, three sub-slots each, plus a write-only strobe:

        0x80 0x82 0x84   X sub-slots 0..2
        0x88 0x8a 0x8c   Y
        0x90 0x92 0x94   Z
        0x98             strobe - written 1, never read

    (From the firmware analysis of the tick and the axis channels.) The strobe
    is the toolpath tap: on each one the nine words are a step command and the
    accumulated position is the machine's path.

    What the real device does with a strobe - how fast it consumes, whether it
    acknowledges - is NOT in the firmware, so this model consumes immediately
    and every emitted step rests on that assumption.
    """
    SIZE = 0x100
    GROUPS = (("X", 0x80), ("Y", 0x88), ("Z", 0x90))
    STROBE = 0x98

    def __init__(self, log, counter=None, subcpu=None):
        self.log = log
        self.counter = counter
        self.subcpu = subcpu          # set by Machine once the peer exists
        self.regs = bytearray(self.SIZE)
        self.strobes = 0
        self.path = []                 # (x, y, z) after each strobe
        self.pos = {"X": 0, "Y": 0, "Z": 0}
        self.other_writes = []
        self.handshakes = 0
        self.sync_ptr = 0
        self.sync_posts = 0
        self.sync_acks = 0

    def _word(self, off):
        v = self.regs[off] | (self.regs[off + 1] << 8)
        return v - 0x10000 if v & 0x8000 else v

    def read(self, off, size, pc):
        if off < 0x100:
            self._peer_sync_read(off, pc)
        return self.regs[off] if size == 1 else self._word(off) & 0xFFFF

    def write(self, off, size, value, pc):
        # CORRECTED at stage D: this window is READ/WRITE MEMORY, not a set of
        # write-only registers. Boot runs a 256-byte pattern test over it
        # (8000:05ad: write BL-1 to every byte, read every byte back, halt on
        # mismatch) and then copies 0xea bytes of ROM table into it. Treating
        # the strobe at 0x98 as write-only made the read-back fail and the
        # firmware halted. So: store everything, and treat a write of exactly 1
        # to 0x98 as the strobe - the value the firmware actually uses.
        if size == 1:
            self.regs[off] = value & 0xFF
        else:
            self.regs[off] = value & 0xFF
            self.regs[off + 1] = (value >> 8) & 0xFF
        if off < 0x100 and size == 1:
            # AFTER the store: the peer clears the byte the CPU just wrote.
            # Calling it before let the store overwrite the clear with 0x55,
            # so the firmware's "wait until it reads 0" spun out and reported
            # a handshake error - which looked exactly like a real timeout.
            self._peer_sync_write(off, value)
        if off == self.STROBE and (value & 0xFF) == 1:
            total = 0
            for ax, base in self.GROUPS:
                d = sum(self._word(base + 2 * k) for k in range(3))
                self.pos[ax] += d
                total += d
            self.strobes += 1
            self.path.append((self.pos["X"], self.pos["Y"], self.pos["Z"]))
            if self.counter is not None:
                self.counter.advance(total)
            # hand the step to the second processor, which is what actually
            # drives the motors on this machine (service doc: "Sub CPU controls
            # the X axis, Y axis and Z axis motors"). self.pos stays as the raw
            # command accumulator; the peer's axes are the physical ones and
            # they clip at the hard stops.
            if self.subcpu is not None:
                d = {}
                for ax, base in self.GROUPS:
                    d[ax] = sum(self._word(base + 2 * k) for k in range(3))
                self.subcpu.consume_step(d["X"], d["Y"], d["Z"])
            return
        if off in (0x00, 0xFF):
            self._maybe_handshake(None)
        if not any(base <= off < base + 6 for _n, base in self.GROUPS):
            self.other_writes.append((off, value, pc))

    # ---- the peer handshake -------------------------------------------
    # Decoded by the emulator at stage D from 8000:0641..806b8, which the
    # firmware itself labels ERR05 HANDSHAKE when it times out:
    #
    #   the main CPU copies a ROM block (cdf0:0100..) into this window at
    #   offset 1..N, writes the length N into BOTH [0x00] and [0xff], then
    #   spins until both read 0xff. It then verifies ES:[i] XOR rom[i] == 0xff.
    #
    # So the peer's contract is: complement every byte of the block, then set
    # both end markers to 0xff. Modelled as an instant response, because the
    # emulator has no clock and the real timing is not in the image.
    def _maybe_handshake(self, rom):
        n = self.regs[0x00]
        if n == 0 or n == 0xFF or self.regs[0xFF] != n:
            return
        for i in range(1, n + 1):
            self.regs[i] ^= 0xFF
        self.regs[0x00] = 0xFF
        self.regs[0xFF] = 0xFF
        self.handshakes += 1

    # ---- the 256-byte peer sync ---------------------------------------
    # Decoded at 8000:6768. If the window does not already carry the ASCII
    # signature "MIMAKI" at 0x1a..0x1f, the main CPU runs a byte-by-byte
    # sync over offsets 0..0xff:
    #
    #   peer writes 0xAA at offset i
    #   main CPU reads it, complements it, writes 0x55 back
    #   peer clears the byte to 0
    #   main CPU moves to i+1
    #
    # 200 retries per byte; on timeout it stores an error code in [0x1984]
    # and the machine later halts with ERR05 HANDSHAKE. On success it writes
    # the signature so a later pass can skip the whole thing.
    #
    # Modelled as an instant peer. The real one is slow enough to need 200
    # retries with a 0x55d-iteration delay each, which is a timing fact this
    # emulator has no way to reproduce.
    # The responder is gated on the READING PC being inside the sync loop
    # (8000:67a0..67f8). That is not a hack for its own sake: the peer only
    # posts during this protocol phase, and the firmware's PC is the only
    # signal the emulator has for which phase it is in. Without the gate the
    # responder would post 0xAA during the earlier 256-byte RAM test at
    # 8000:05ad and make that test fail instead.
    SYNC_LO, SYNC_HI = 0x867a0, 0x867f8

    def _peer_sync_read(self, off, pc):
        if not (self.SYNC_LO <= pc <= self.SYNC_HI):
            return
        if off == self.sync_ptr and self.regs[off] != 0xAA:
            self.regs[off] = 0xAA
            self.sync_posts += 1

    def _peer_sync_write(self, off, value):
        if off == self.sync_ptr and (value & 0xFF) == 0x55:
            self.regs[off] = 0x00
            self.sync_ptr = (self.sync_ptr + 1) & 0xFF
            self.sync_acks += 1


class Endstops(object):
    """Port 0x06 with endstops that are actually reached.

    POLARITY, settled by reading 8000:1000 as a whole: **bit set = switch
    ACTUATED, bit clear = free.**  The proof is the escape move at 81021 -

        8101d  IN AL,0x6 ; AND AL,0x30 ; TEST AL,0x20 ; JZ (skip)
        81025  target = current + 10000        <- move AWAY

    an escape only makes sense when the switch is already made, and it runs
    when the bit is SET. So a free switch reads 0, and at power-up bits 4, 5
    and 6 are all clear.

    (The owner's "free at rest, active low" describes the switch; the port bit
    ends up 0 when free, which is consistent. An earlier version of this file
    had the polarity inverted and labelled 0x8F "all actuated" when it is in
    fact all free.)

    The wait loops then read naturally:

        wait_endstop(BL=1)  loop while all bits SET   -> back off until released
        wait_endstop(BL=2)  loop while all bits CLEAR -> seek until made

    so travel is modelled by clearing the bits during a BL=1 phase and setting
    them during a BL=2 phase, after `travel` polls. The poll count stands in for
    distance, which the emulator cannot derive - it has neither geometry nor a
    clock. A behavioural stand-in, not a hardware fact.
    """
    READ_SITES = {0x81891: 0x30, 0x818e9: 0x40}
    # bits 4,5,6 clear = all three endstops free.
    #
    # Bits 0 and 1 are attachment switches, read at 824eb and turned into the
    # flags [0x0847] and [0x0848]; a CLEAR bit sets its flag. With bit 1 SET the
    # firmware believes the chip-removal attachment is fitted and answers ERR65
    # REMOVE CHIP REMOVAL ATTACHMENT to every PZ, so the Mimaki Z extension is
    # unusable. 0x8D presents it as not fitted, which is the state the tests
    # need and the state the owner asked for ("surface sensor not active").
    # 0x8F is the historical value every log before 2026-08-31 was taken with.
    IDLE = 0x8D

    def __init__(self, log, value=None, travel=40, subcpu=None):
        self.log = log
        self.value = self.IDLE if value is None else value
        self.travel = travel
        # When a sub CPU with physics is attached, the switch level follows the
        # AXIS POSITION instead of a poll count. The poll-count model was always
        # a behavioural stand-in - "the switch makes after n polls" - because
        # the emulator had no geometry. With geometry it has one.
        self.subcpu = subcpu
        self.reads = 0
        self.polls = {}
        self.events = []

    def port_read(self, port, pc, uc=None):
        self.reads += 1
        if self.subcpu is not None and self.subcpu.enable_physics:
            self.value = self.subcpu.endstop_bits(self.IDLE)
            return self.value
        mask = self.READ_SITES.get(pc)
        if mask is None or uc is None:
            return self.value
        from unicorn.x86_const import UC_X86_REG_BL
        phase = uc.reg_read(UC_X86_REG_BL)
        key = (pc, phase)
        n = self.polls.get(key, 0) + 1
        self.polls[key] = n
        if n >= self.travel:
            cur = self.value & mask
            # The firmware has FOUR wait conditions at 81895..818c4, not two.
            # Modelling only 1 and 2 left phases 3 and 4 spinning forever: the
            # fourth seek of the homing run waited 4 million instructions, its
            # move ran out, and 1885 returned AH=1 -> ERR50 X SENSOR, i.e. "the
            # X machine origin cannot be found". The axis was never going to
            # arrive because nothing in this model was ever going to move it.
            #
            #   BL=1  loop while ALL set    -> back off until released
            #   BL=2  loop while ALL clear  -> seek until made
            #   BL=3  loop while ANY set    -> travel until every switch frees
            #   BL=4  loop while not ALL set-> travel until every switch makes
            if phase in (2, 4) and cur != mask:
                self.value |= mask                   # the switch(es) are reached
                self.events.append(("made", mask, self.reads, phase))
                self.polls[key] = 0
            elif phase in (1, 3) and cur != 0:
                self.value &= ~mask & 0xFF           # released
                self.events.append(("released", mask, self.reads, phase))
                self.polls[key] = 0
        return self.value
