"""Attribute an executed address to a functional region.

Every range below is an address this project established by analysis, not a
guess at a layout. The citation after each entry is where it was pinned down;
addresses without a citation are the contiguous body around a cited entry point
and are marked as such.

Regions deliberately do NOT tile the whole ROM. Anything unattributed comes back
as "unmapped", and a measurement in which "unmapped" is large is a measurement
that has not been understood yet - the first attempt at a cost profile spent
96.6% of its time there while reporting the remainder as "parser" and "planner".
"""

# (lo, hi_exclusive, region, what it is)
RANGES = [
    # ---- boot and low-level services -----------------------------------
    (0x80000, 0x80700, "boot",    "reset path, IVT fill, memory test 0596"),
    (0x80900, 0x80d00, "panel",   "key matrix scan 09f8, evaluator 0b26"),
    (0x80e00, 0x81000, "panel",   "LCD writers 0e94/0eab/0f2d"),
    (0x81a00, 0x81c00, "config",  "NVRAM transfer 1a2b..1b79"),
    (0x81c00, 0x81d00, "config",  "far memcpy 1c6f"),

    # ---- motion request builders and the homing block -------------------
    (0x810c0, 0x81500, "homing",  "the init/homing window, 10 submits"),
    (0x817ae, 0x81900, "executor", "request builders 17ae/181c, endstop wait 18dd"),

    # ---- the executor ---------------------------------------------------
    (0x82a00, 0x83100, "executor", "2a80 record builder, 2ef4, free-slot wait 2fa0"),
    (0x83100, 0x84900, "executor", "carryover/rate maths, DIV at 3cc8"),
    (0x84900, 0x85000, "executor", "4a06 launch wrapper, 498c, 49cd wait-empty"),

    # ---- the INT 20h step engine ----------------------------------------
    (0x85000, 0x87000, "step",    "states 4898/5644/587e/5a74/5be6/60a2, "
                                  "step core 844xx, tail advance 5ee5, guard 6ce2"),

    # ---- the HP-GL frontend: ISR, ring, tokeniser, number machine --------
    (0x8a400, 0x8a600, "parser",  "ring peek/classify a48e, advance a41f"),
    (0x8ac00, 0x8b600, "parser",  "serial ISR ac00, fetch aebc, commit b090, "
                                  "ingest dispatch b180"),
    (0x8c500, 0x8ca00, "parser",  "number state machine c60e, param copy c872"),
    (0x89600, 0x89800, "parser",  "command handler table 9670 (data + walker)"),

    # ---- the HP-GL command handlers -------------------------------------
    (0xaf400, 0xaf900, "handlers", "PA af400, PU af543, PR af495, PD af6d4"),
    (0xb4000, 0xbf000, "handlers", "LB/JK/CP/VZ/OE/OP/DI/... handler bodies"),

    # ---- the planner bridge ---------------------------------------------
    (0xb0000, 0xb1000, "planner",  "local planner b0a30, pre-d230 prepare blocks"),
    (0xa1000, 0xa1200, "planner",  "planner_bridge_root_10da"),
    (0xaf900, 0xb0000, "planner",  "f9b2 recompute/reset, fa38 post-d22f"),
    (0x8d100, 0x8e000, "planner",  "d181, rate mapper d5c2, orchestrator d527"),
    (0xad100, 0xad600, "planner",  "rate root loader d190, pen mode publisher d22f"),

    # ---- the HP-GL task shell -------------------------------------------
    (0x8a000, 0x8a400, "frontend", "task main loop a03a, mode gate a331"),
    (0xab500, 0xaca00, "frontend", "dispatcher entry ab510, interface mode "
                                   "dispatch ab53c, ready-gate wait ab554"),

    # ---- the cooperative kernel -----------------------------------------
    (0x9d600, 0x9d900, "kernel",   "scheduler 9d6a8, INT 23h tick 9d7d6"),
    (0x82600, 0x82800, "kernel",   "INT 4xh service stubs 263d/264a/269e"),
    (0x86600, 0x86800, "kernel",   "payload queue state init 6666"),

    # ---- maths ----------------------------------------------------------
    (0x9c000, 0x9d000, "math",     "software FP selection cb84, helpers"),
    (0x99000, 0x9a000, "math",     "transcendental dispatch 96c0"),
    (0xc0000, 0xd0000, "math",     "c000: numeric helpers (coarse)"),
]

_TABLE = None


def _build():
    global _TABLE
    _TABLE = sorted(RANGES)


def region(addr):
    if _TABLE is None:
        _build()
    for lo, hi, name, _ in _TABLE:
        if lo <= addr < hi:
            return name
    return "unmapped"


def describe():
    if _TABLE is None:
        _build()
    return [(lo, hi, name, what) for lo, hi, name, what in _TABLE]
