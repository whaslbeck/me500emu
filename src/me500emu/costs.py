# -*- coding: utf-8 -*-
"""Instruction cost per class: a static cost table over the running image.

The core counts instructions as time units (1 unit = 0.5 us = 2.4576 8253 clocks). The V33, however,
costs differently per instruction class (machine, `~B` measurement run 2026-09-24/25, clocks per
instruction of the seven mixes: register 1.36 / RAM read 3.18 / RAM write 3.08 / mul-div 4.62 /
RAM 2000: 6.33 / port 5.82 / rep movsw 14.56). This table holds, per INSTRUCTION START in the image, a
delta in eighth units against the base of 1; the core's instruction hook sums them, and they are folded
in at the end of each slice (like the old executor 1.4 calibration, which this replaces).

The core's instruction hook fires per instruction (for `rep`, per repetition at the same address) -
so the table holds, per address, the delta per FIRING in SIXTEENTHS (rep too: the core counts every
repetition in the budget, so there as well the delta is relative to the base of 1). Runtime share that
cannot be decided statically: accesses to the sub-CPU window 2000:00xx (surcharge at the motion MMIO
hook, PEER_SURCHARGE).

The class costs are PARAMETERS; they were fitted by matching the emulated `~B` values against the
machine measurement. The table is cached per image + parameters in the cache directory.
"""
import hashlib
import os
import struct

CYCLES_PER_UNIT = 2.4576              # 4.9152 MHz x 0.5 us

# Class costs in units (start values from the machine ~B values; the calibration adjusts them)
COSTS = {
    # calibrated 2026-09-25 against the seven ~B machine mixes (exact to a sixteenth, so the table
    # quantisation does not flip). "jump" == "reg": a separate jump class cannot be identified from
    # the ~B loops (only 1 jump per 10 instructions) - the jump-dense executor handlers carry the
    # measured range surcharges instead (HANDLER_SURCHARGE).
    "reg":     0.4375,                 # ~B0 1.36 clocks/instruction
    "read":    1.1875,                 # ~B1 3.18
    "write":   1.125,                  # ~B2 3.08
    "both":    1.9375,                 # (no ~B of its own)
    "muldiv":  3.1875,                 # ~B3 4.62
    "io":      2.4375,                 # ~B5 5.82
    "rep_word": 1.3125,                # ~B6 14.56
    "jump":    0.4375,                 # same as reg, see above
}

# Range surcharges in sixteenths per instruction: the tick anchors of the machine tick trace (chained
# 5be6 tick 677 us, plateau 587e tick 252 us) come out at 511/143 us with the pure class costs -
# the handlers are jump-dense and [bp/di+disp]-heavy. The surcharge closes the measured gap.
HANDLER_SURCHARGE = {
    # (time alignment) ONE surcharge for the whole executor range (Z lane 435a, handover 47c4,
    # launch 5644, plateau 587e, brake 5a74, chain 5be6 ...), as the old flat calibration covered it:
    # the tick handling is jump-dense and [bp/di+disp]-heavy. Calibrated against the firmware counter TK2
    # ("ticks over 0.5 ms") of the workshop run on 2026-09-25. KOST_EXEC overrides it (calibration).
    (0x84300, 0x86300): int(os.environ.get("KOST_EXEC", "8")),
}
PEER_SURCHARGE = 1.5                    # per access to 2000:00xx, on top of the memory class (~B4 6.33 clocks)

_READERS = {"pop", "popa", "popaw", "popf", "leave", "lds", "les", "lodsb", "lodsw",
          "cmpsb", "cmpsw", "scasb", "scasw", "xlatb"}
_WRITERS = {"push", "pusha", "pushaw", "pushf", "stosb", "stosw"}
_JUMPS = {"jmp", "ljmp", "call", "lcall", "ret", "retf", "iret", "int", "into", "int3",
           "loop", "loope", "loopne", "jcxz"}
_BOTH_STR = {"movsb", "movsw"}
_MULDIV = {"mul", "imul", "div", "idiv", "aam", "aad"}


def _instr_class(i):
    """Delta in eighth units per hook firing for a Capstone instruction."""
    mn = i.mnemonic
    if mn.startswith("rep"):
        # the core counts every repetition in the budget and the hook fires per repetition:
        # delta relative to the base of 1, as for all classes
        return int(round((COSTS["rep_word"] - 1.0) * 16))
    reads = writes = False
    try:
        ops = i.operands
    except Exception:
        ops = []
    for op in ops:
        if op.type == 3:               # X86_OP_MEM
            if op.access & 1: reads = True
            if op.access & 2: writes = True
    if mn in _JUMPS or mn.startswith("j"):    # includes all jcc
        k = COSTS["jump"]
    elif mn in _MULDIV:
        k = COSTS["muldiv"] + (COSTS["read"] - COSTS["reg"] if reads else 0.0)
    elif mn in ("in", "out", "insb", "insw", "outsb", "outsw"):
        k = COSTS["io"]
    elif mn in _BOTH_STR or (reads and writes):
        k = COSTS["both"]
    elif reads or mn in _READERS:
        k = COSTS["read"]
    elif writes or mn in _WRITERS:
        k = COSTS["write"]
    else:
        k = COSTS["reg"]
    return int(round((k - 1.0) * 16))


def build_table(rom_bytes, scratch=None):
    """Delta table (bytearray, signed int8 per file offset) for the image at 0x80000."""
    key = hashlib.md5(rom_bytes + (repr(sorted(COSTS.items())) + repr(sorted(HANDLER_SURCHARGE.items()))).encode() + b"v7").hexdigest()[:16]
    if scratch is None:
        from . import paths
        scratch = paths.cache_dir()
    path = os.path.join(scratch, "costs_%s.tab" % key)
    if os.path.exists(path):
        return bytearray(open(path, "rb").read())
    from capstone import Cs, CS_ARCH_X86, CS_MODE_16
    md = Cs(CS_ARCH_X86, CS_MODE_16)
    md.detail = True
    md.skipdata = True
    delta = bytearray(len(rom_bytes))
    for i in md.disasm(bytes(rom_bytes), 0):
        if i.mnemonic == ".byte":
            continue
        d = _instr_class(i)
        for (lo, hi), z in HANDLER_SURCHARGE.items():
            if lo <= 0x80000 + i.address < hi:
                d += z
        delta[i.address] = max(-128, min(127, d)) & 0xFF
    try:
        open(path, "wb").write(bytes(delta))
    except Exception:
        pass
    return delta
