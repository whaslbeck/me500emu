"""The physical memory map, with the fail-loudly policy from the design.

The emulator's first job is to find errors in an analysis with 41.3% decode
coverage. A silent zero for an undecoded device would be worse than a crash, so
every access outside a known region raises, and every access to a device that is
not implemented yet raises by name.
"""
from . import symbols

RAM_BASE, RAM_SIZE = 0x00000, 0x10000      # 64 KiB: only segment 0 is ever loaded
ROM_BASE, ROM_SIZE = 0x80000, 0x80000

# name, base, size, why we believe it
REGIONS = [
    ("ram",         0x00000, 0x10000, "DS/SS/ES=0; SP=0 puts the stack at the top"),
    ("motion_regs", 0x20000, 0x00100, "motion request registers (firmware analysis)"),
    ("nvram",       0x24000, 0x00400, "NVRAM config path (firmware analysis)"),
    ("window_prod", 0x30000, 0x10000, "paged window, ports 0xff18..0xff1e"),
    ("window_cons", 0x40000, 0x10000, "paged window, ports 0xff20..0xff26"),
    ("window_probe", 0x50000, 0x10000, "paged window, port 0xff28"),
    ("window_font", 0x60000, 0x10000, "paged window, ports 0xff30/0xff32"),
    ("rom",         0x80000, 0x80000, "the EPROM image"),
]

DEVICE_REGIONS = [r for r in REGIONS if r[0] not in ("ram", "rom")]


class BusFault(Exception):
    pass


def region_of(addr):
    for name_, base, size, _ in REGIONS:
        if base <= addr < base + size:
            return name_
    return None


def fault_msg(kind, addr, size, pc, extra=""):
    reg = region_of(addr)
    msg = ("%s at %05x (size %d)\n"
           "    region : %s\n"
           "    from PC: %05x  %s\n" % (kind, addr, size, reg or "UNMAPPED",
                                        pc, symbols.name(pc)))
    if extra:
        msg += "    %s\n" % extra
    return msg


def fault(kind, addr, size, pc, extra=""):
    reg = region_of(addr)
    where = symbols.name(pc)
    msg = ("%s at %05x (size %d)\n"
           "    region : %s\n"
           "    from PC: %05x  %s\n" % (kind, addr, size, reg or "UNMAPPED", pc, where))
    if extra:
        msg += "    %s\n" % extra
    raise BusFault(msg)
