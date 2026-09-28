"""Unicorn x86-16 plus the two V33 instructions.

Stage A found that BRKXA cannot be trapped by an invalid-instruction hook:
0F E0 30 is a valid MMX encoding (pavgb) and Unicorn executes it silently. A
whole-ROM byte scan found exactly one BRKXA (0x80457) and one RETXA (0x80470),
so both are trapped with narrow address-ranged code hooks - correct for this
image and far cheaper than hooking every instruction.
"""
import struct
from unicorn import *
from unicorn.x86_const import *

from . import bus
from . import symbols

BRKXA_ADDR = 0x80457
RETXA_ADDR = 0x80470
BRKXA_BYTES = b"\x0f\xe0\x30"
RETXA_BYTES = b"\x0f\xf0\x30"


class Cpu(object):
    def __init__(self, machine):
        self.m = machine
        self.uc = Uc(UC_ARCH_X86, UC_MODE_16)
        self.xa_mode = False
        self.xa_events = []
        self._ivt_cache = {}

    # ---- the two V33 instructions -------------------------------------
    def _xa(self, enter, addr):
        """BRKXA/RETXA: take the 4-byte vector at (n*4) and continue there,
        switching the addressing mode. Operand is the third opcode byte."""
        n = self.uc.mem_read(addr + 2, 1)[0]
        vec = self.uc.mem_read(n * 4, 4)
        off, seg = struct.unpack("<HH", bytes(vec))
        self.xa_mode = enter
        self.xa_events.append((addr, "BRKXA" if enter else "RETXA", n, seg, off))
        self.uc.reg_write(UC_X86_REG_CS, seg)
        self.uc.reg_write(UC_X86_REG_IP, off)
        self.uc.emu_stop()          # restarted by the machine at the new CS:IP
        self._resume_at = seg * 0x10 + off
        if self.m.fc is not None:
            self.m.fc.state.resume_at = self._resume_at

    def install(self, fast=False):
        """fast=True: the C glue layer (fastcore) takes over the port, INT and IVT hooks."""
        uc = self.uc
        self._resume_at = None

        # The IVT is written once by the installer loop at boot and then left
        # alone, so caching the vectors is safe as long as any write into that
        # first kilobyte drops the cache.
        def ivt_written(uc_, access, address, size, value, _):
            self._ivt_cache.clear()

        uc.hook_add(UC_HOOK_MEM_WRITE, ivt_written, None, 0x0000, 0x03FF)

        def brkxa(uc_, addr, size, _):
            if bytes(uc_.mem_read(addr, 3)) == BRKXA_BYTES:
                self._xa(True, addr)

        def retxa(uc_, addr, size, _):
            if bytes(uc_.mem_read(addr, 3)) == RETXA_BYTES:
                self._xa(False, addr)

        uc.hook_add(UC_HOOK_CODE, brkxa, None, BRKXA_ADDR, BRKXA_ADDR)
        uc.hook_add(UC_HOOK_CODE, retxa, None, RETXA_ADDR, RETXA_ADDR)

        # ---- fail loudly ----------------------------------------------
        def unmapped(uc_, access, addr, size, value, _):
            pc = uc_.reg_read(UC_X86_REG_CS) * 16 + uc_.reg_read(UC_X86_REG_IP)
            kind = {UC_MEM_READ_UNMAPPED: "unmapped read",
                    UC_MEM_WRITE_UNMAPPED: "unmapped write",
                    UC_MEM_FETCH_UNMAPPED: "unmapped fetch"}.get(access, "unmapped access")
            self.m.faults.append(bus.fault_msg(kind, addr, size, pc))
            return False        # stop; do not invent data

        def prot(uc_, access, addr, size, value, _):
            pc = uc_.reg_read(UC_X86_REG_CS) * 16 + uc_.reg_read(UC_X86_REG_IP)
            self.m.faults.append(bus.fault_msg(
                "write into ROM", addr, size, pc, "value %04x" % value))
            return False

        uc.hook_add(UC_HOOK_MEM_UNMAPPED, unmapped)
        uc.hook_add(UC_HOOK_MEM_WRITE_PROT, prot)
        uc.hook_add(UC_HOOK_MEM_FETCH_PROT, prot)
        if fast:
            return

        # ---- ports ------------------------------------------------------
        def hook_in(uc_, port, size, _):
            pc = uc_.reg_read(UC_X86_REG_CS) * 16 + uc_.reg_read(UC_X86_REG_IP)
            return self.m.port_read(port, size, pc)

        def hook_out(uc_, port, size, value, _):
            pc = uc_.reg_read(UC_X86_REG_CS) * 16 + uc_.reg_read(UC_X86_REG_IP)
            self.m.port_write(port, size, value, pc)

        uc.hook_add(UC_HOOK_INSN, hook_in, None, 1, 0, UC_X86_INS_IN)
        uc.hook_add(UC_HOOK_INSN, hook_out, None, 1, 0, UC_X86_INS_OUT)

        # ---- interrupts -------------------------------------------------
        # Unicorn does not dispatch real-mode interrupts; the hook replaces the
        # CPU's behaviour entirely. Service verbs are implemented here; anything
        # else is routed through the IVT the way the hardware would, so a wrong
        # vector shows up as a fault instead of silently doing nothing.
        def hook_intr(uc_, intno, _):
            # CORRECTED at stage E: INT 40h..47h are NOT external. They are the
            # system calls of a multitasking kernel inside this ROM, installed
            # by a block copy at 9d60:0020 that copies 8 far pointers from
            # cs:[0] into IVT 0x100..0x11f. Dispatch them like any other
            # interrupt; the stub layer stays only as a fallback for a vector
            # the firmware has not installed, and logs if it ever fires.
            import struct as _s
            vec = _s.unpack("<HH", bytes(uc_.mem_read(intno * 4, 4)))
            if vec != (0, 0):
                self._dispatch_ivt(uc_, intno, from_hook=True)
                return
            if 0x40 <= intno <= 0x47:
                self.m.service.handle(intno, uc_)
                return
            self._dispatch_ivt(uc_, intno, from_hook=True)

        uc.hook_add(UC_HOOK_INTR, hook_intr)

    def _dispatch_ivt(self, uc, intno, from_hook=False):
        """Vector to a handler.

        from_hook=True means we are inside UC_HOOK_INTR, i.e. a SOFTWARE INT.
        There the engine need not be stopped at all: changing CS:IP in the hook
        is enough and execution continues from the new address. Stopping was
        costing an emu_start round trip on every INT - and the firmware issues
        INT 43h hundreds of thousands of times - and it also broke instruction
        accounting, because a run that stops early has executed fewer
        instructions than the budget it was given."""
        # The dispatcher runs about once every 85 instructions - the kernel
        # yields with INT 43h constantly - so everything avoidable here is
        # avoided. The vector is cached and invalidated by any write into the
        # IVT (see install()); the three stack pushes are one 6-byte write; and
        # struct is imported at module scope.
        seg_off = self._ivt_cache.get(intno)
        if seg_off is None:
            off, seg = struct.unpack("<HH", bytes(uc.mem_read(intno * 4, 4)))
            self._ivt_cache[intno] = (seg, off)
        else:
            seg, off = seg_off
        if seg == 0 and off == 0:
            # An injected hardware interrupt before the firmware has installed
            # its vector is the emulator's mistake, not the firmware's: drop it
            # instead of dispatching into nothing. A SOFTWARE INT with a null
            # vector is still a fault, so those are counted separately.
            self.m._dropped_irqs.append(intno)
            return
        sp = uc.reg_read(UC_X86_REG_SP)
        ss = uc.reg_read(UC_X86_REG_SS)
        flags = uc.reg_read(UC_X86_REG_EFLAGS) & 0xFFFF
        cs = uc.reg_read(UC_X86_REG_CS)
        ip = uc.reg_read(UC_X86_REG_IP)
        sp = (sp - 6) & 0xFFFF
        uc.mem_write(ss * 16 + sp, struct.pack("<HHH", ip, cs, flags))
        uc.reg_write(UC_X86_REG_SP, sp)
        uc.reg_write(UC_X86_REG_EFLAGS, flags & ~0x300)      # IF and TF are cleared on entry (as the CPU does)
        uc.reg_write(UC_X86_REG_CS, seg)
        uc.reg_write(UC_X86_REG_IP, off)
        d = self.m._irq_dispatches
        if len(d) < 200000:            # bounded: this used to grow forever
            d.append((intno, seg, off))
        if not from_hook:
            uc.emu_stop()
            self._resume_at = seg * 0x10 + off
