"""Address -> Ghidra symbol, so a bus fault names the code that caused it."""
import os

_SYMS = {}
_SORTED = []


def load(path=None):
    global _SORTED
    if path is None:
        from . import paths
        path = paths.data_path("symbols.txt")
    with open(path) as fh:
        for line in fh:
            parts = line.split(None, 2)
            if len(parts) != 3 or parts[1] not in ("F", "L"):
                continue
            _SYMS.setdefault(int(parts[0], 16), parts[2].strip())
    _SORTED = sorted(_SYMS)
    return len(_SYMS)


def name(addr):
    """Exact symbol, or nearest preceding one with an offset."""
    if addr in _SYMS:
        return _SYMS[addr]
    if not _SORTED:
        return "?"
    lo, hi = 0, len(_SORTED) - 1
    best = None
    while lo <= hi:
        mid = (lo + hi) // 2
        if _SORTED[mid] <= addr:
            best = _SORTED[mid]
            lo = mid + 1
        else:
            hi = mid - 1
    if best is None or addr - best > 0x400:
        return "?"
    return "%s+0x%x" % (_SYMS[best], addr - best)
