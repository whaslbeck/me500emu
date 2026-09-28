"""The INT 40h..47h service layer - implemented, not emulated.

These handlers are not in the EPROM (firmware analysis), so
the emulator has to invent them. The point of stage D is NOT to get them right;
it is to answer "nothing available" everywhere, log every call with its full
register context, and read the firmware's reaction. That reaction is the only
evidence available for what the verbs mean.

Known convention, from the ROM side:
  - INT 44h is called with DI = a descriptor address (0x0524 / 0x0529 / 0x0533)
    and DX = an argument; it returns BX pointing at a buffer
  - [BX+2] == 0xffff means "nothing available"   (80a74)
  - [BX+6] = 0 is how the caller consumes a slot (80a9b)
  - returned payload words live at [BX+8], [BX+0a], [BX+0c], [BX+0e] (80a81..)
"""
import struct
from unicorn.x86_const import *

R = {"AX": UC_X86_REG_AX, "BX": UC_X86_REG_BX, "CX": UC_X86_REG_CX,
     "DX": UC_X86_REG_DX, "SI": UC_X86_REG_SI, "DI": UC_X86_REG_DI,
     "BP": UC_X86_REG_BP, "DS": UC_X86_REG_DS, "ES": UC_X86_REG_ES,
     "CS": UC_X86_REG_CS}

NOTHING_AVAILABLE = 0xFFFF


class Service(object):
    def __init__(self, machine):
        self.m = machine
        self.calls = []            # (intno, regs, note)
        self.by_verb = {}

    def _regs(self, uc):
        return dict((n, uc.reg_read(r)) for n, r in R.items())

    def handle(self, intno, uc):
        regs = self._regs(uc)
        pc = regs["CS"] * 16 + uc.reg_read(UC_X86_REG_IP)
        note = ""

        if intno == 0x44:
            # answer through the descriptor the caller passed in DI
            di = regs["DI"]
            try:
                uc.mem_write(di + 2, struct.pack("<H", NOTHING_AVAILABLE))
                uc.reg_write(UC_X86_REG_BX, di)
                note = "BX=DI=%04x, [DI+2]=ffff (nothing available)" % di
            except Exception as e:
                note = "could not answer: %s" % e
        elif intno in (0x40, 0x42, 0x43, 0x45, 0x47):
            note = "stub: registers left as-is"
        else:
            note = "UNEXPECTED service verb"

        self.calls.append((intno, pc, regs, note))
        self.by_verb.setdefault(intno, []).append((pc, regs, note))
