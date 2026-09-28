"""Stage A spike: does Unicorn's x86-16 real mode execute this ROM the way
Ghidra decoded it?  Throwaway - the only deliverable is the verdict.

Records the executed instruction trace from the reset vector and writes the
address list for an independent Ghidra cross-check.

Usage: python -m me500emu.boot_trace [max_instructions] [out.json] [--rom FILE]
"""
import os, sys, json
from unicorn import *
from unicorn.x86_const import *
import capstone

from me500emu import paths

ROM_BASE = 0x80000

_args = list(sys.argv[1:])
_rom_arg = None
if "--rom" in _args:
    _i = _args.index("--rom")
    _rom_arg = _args[_i + 1] if _i + 1 < len(_args) else None
    del _args[_i:_i + 2]
ROM_PATH = paths.rom_path("stock", _rom_arg)
MAXINS = int(_args[0]) if len(_args) > 0 else 400
OUT_PATH = _args[1] if len(_args) > 1 else os.path.join(paths.cache_dir(), "spike_trace.json")

rom = open(ROM_PATH, "rb").read()
assert len(rom) == 0x80000

uc = Uc(UC_ARCH_X86, UC_MODE_16)
uc.mem_map(0x00000, 0x100000)          # flat 1 MiB for the spike
uc.mem_write(ROM_BASE, rom)

md = capstone.Cs(capstone.CS_ARCH_X86, capstone.CS_MODE_16)

trace, ports, romwrites, notes = [], [], [], []


def hook_code(uc, addr, size, _):
    if len(trace) >= MAXINS:
        uc.emu_stop()
        return
    code = uc.mem_read(addr, min(size, 15))
    txt = "?"
    for i in md.disasm(bytes(code), addr):
        txt = "%s %s" % (i.mnemonic, i.op_str)
        break
    trace.append((addr, size, txt))


def hook_mem_write(uc, access, addr, size, value, _):
    if addr >= ROM_BASE:
        romwrites.append((uc.reg_read(UC_X86_REG_CS) * 16 + uc.reg_read(UC_X86_REG_IP),
                          addr, size, value))
    return True


def hook_in(uc, port, size, _):
    ip = uc.reg_read(UC_X86_REG_CS) * 16 + uc.reg_read(UC_X86_REG_IP)
    ports.append(("IN", port, size, ip))
    return 0                                   # spike only: log and continue


def hook_out(uc, port, size, value, _):
    ip = uc.reg_read(UC_X86_REG_CS) * 16 + uc.reg_read(UC_X86_REG_IP)
    ports.append(("OUT", port, size, ip, value))


uc.hook_add(UC_HOOK_CODE, hook_code)
uc.hook_add(UC_HOOK_MEM_WRITE, hook_mem_write)
uc.hook_add(UC_HOOK_INSN, hook_in, None, 1, 0, UC_X86_INS_IN)
uc.hook_add(UC_HOOK_INSN, hook_out, None, 1, 0, UC_X86_INS_OUT)

uc.reg_write(UC_X86_REG_CS, 0xFFFF)
uc.reg_write(UC_X86_REG_IP, 0x0000)

err = None
try:
    uc.emu_start(0xFFFF0, 0x100000, count=0)
except UcError as e:
    err = e

print("instructions executed : %d" % len(trace))
print("stop reason           : %s" % (("UcError: %s" % err) if err else "instruction budget"))
print("")
print("first 40 instructions:")
for a, s, t in trace[:40]:
    print("   %05x  (%d)  %s" % (a, s, t))
if len(trace) > 40:
    print("   ...")
    print("last 10:")
    for a, s, t in trace[-10:]:
        print("   %05x  (%d)  %s" % (a, s, t))

print("")
print("port accesses: %d" % len(ports))
for p in ports[:25]:
    print("   ", p)

print("")
print("writes into the ROM range: %d" % len(romwrites))
for w in romwrites[:10]:
    print("    from %05x -> %05x size %d value %04x" % w)

seen = {}
for a, s_, t in trace:
    seen[a] = (s_, t)
json.dump({"%05x" % a: [v[0], v[1]] for a, v in seen.items()},
          open(OUT_PATH, "w"))
