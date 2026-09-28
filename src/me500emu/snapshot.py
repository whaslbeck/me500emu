"""One call that returns everything a UI would draw.

Kept separate from Machine so the emulator core stays free of presentation
concerns, and so a future UI has a single, stable surface to poll rather than
reaching into device internals.

Every field names the cell or device it comes from, because a UI that shows a
number nobody can trace back to the firmware is worse than one that shows
nothing.
"""


def _w(m, addr, n=2):
    return int.from_bytes(m.cpu.uc.mem_read(addr, n), "little")


def snapshot(m):
    """A flat, JSON-serialisable view of the machine."""
    ring_free = _w(m, 0x2094, 4)
    return {
        "instructions": m.instr,
        "faults": list(m.faults),

        # --- panel -------------------------------------------------------
        "lcd": m.panel.text(),                       # 4 rows x 16, HD44780 DDRAM
        "ready_gate": _w(m, 0x0843, 1),              # 0 park, 1 idle, 2 run,
                                                     # 0x81/0x82 handshake
        "gate_text": {0: "LOCAL (parked)", 1: "idle", 2: "REMOTE (run)",
                      0x81: "->idle", 0x82: "->run"}.get(_w(m, 0x0843, 1), "?"),

        # --- axes --------------------------------------------------------
        "position_steps": dict(m.motion.pos),        # accumulated from the
                                                     # 0x2000 window strobes
        "strobes": m.motion.strobes,
        "path_len": len(m.motion.path),

        # --- HP-GL ring (the "1 [MB]" the panel advertises) ---------------
        "ring": {
            "write": _w(m, 0x2084, 4),               # 8b0a5 publishes to 208c
            "committed": _w(m, 0x208c, 4),
            "read": _w(m, 0x2090, 4),                # 8a43d advances, wraps at 1 MiB
            "free": ring_free,
            "capacity": 1 << 20,
            "used": (1 << 20) - ring_free,
        },

        # --- move queue ---------------------------------------------------
        "queue": {
            "head": _w(m, 0x1004, 1),                # 82ed6, producer
            "tail": _w(m, 0x1005, 1),                # 85ee5, consumer
            "slots": 16,                             # table at 0x850b8, stride 0x6c
            "depth": (_w(m, 0x1004, 1) - _w(m, 0x1005, 1)) & 0x0F,
            "seek_in_progress": _w(m, 0x1003, 1),    # 82ef4 sets, 861b2 clears
            "busy": _w(m, 0x1000, 1),
        },

        # --- step engine ---------------------------------------------------
        "int20_vector": _w(m, 0x0080, 2),            # the state machine's own
        "state_name": {0x4898: "idle service", 0x5644: "revector",
                       0x587e: "stepping", 0x5a74: "no continuation",
                       0x5be6: "chained segment", 0x60a2: "completion",
                       0x5f12: "5f12"}.get(_w(m, 0x0080, 2), "?"),

        # --- kernel -------------------------------------------------------
        "current_tcb": _w(m, 0x0500, 2),
        "tasks": [
            {"base": b, "state": _w(m, b, 1), "sp": _w(m, b + 1, 2),
             "waits_on": _w(m, b + 3, 2)}
            for b in (0x0504, 0x050c, 0x0514)
        ],

        # --- config that the machine actually uses -------------------------
        "travel_limits_mm": {"x": _w(m, 0x06aa), "y": _w(m, 0x06ac)},
        "rate_root": [_w(m, 0x09f6 + 2 * i) for i in range(10)],
        "end_threshold": _w(m, 0x1a2c),
    }


def render_text(s):
    """A compact text rendering - the shape a first UI would take."""
    out = []
    out.append("+----------------+")
    for row in s["lcd"]:
        out.append("|%-16s|" % row[:16])
    out.append("+----------------+")
    out.append("gate %s   state %s   vector %04x"
               % (s["gate_text"], s["state_name"], s["int20_vector"]))
    out.append("pos  X=%d Y=%d Z=%d   strobes %d"
               % (s["position_steps"]["X"], s["position_steps"]["Y"],
                  s["position_steps"]["Z"], s["strobes"]))
    out.append("ring %d/%d bytes used   queue %d/%d  (head %d tail %d)  seek=%d"
               % (s["ring"]["used"], s["ring"]["capacity"], s["queue"]["depth"],
                  s["queue"]["slots"], s["queue"]["head"], s["queue"]["tail"],
                  s["queue"]["seek_in_progress"]))
    return "\n".join(out)
