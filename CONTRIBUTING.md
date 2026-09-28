# Contributing

Contributions are welcome: device models, accuracy against the real machine, tooling for CAM testing, ports to other
platforms, documentation. A few rules keep the emulator trustworthy.

## Determinism is the contract

The emulator is deterministic: the same ROM, NVRAM and input give the same instruction count, the same strobe count
and the same RAM contents, run after run. Much of what the project knows was established by comparing such numbers, so
**a change to the core must keep them bit-identical unless the change is meant to alter behaviour**:

- C core (`fastcore.c`), run loop, interrupt delivery, device models, cost table: after the change, a reference job
  must give the same `m.instr`, `m.motion.strobes`, axis positions and a hash of RAM as before. Check `trace="fast"`
  and `trace="count"`.
- `tests/golden_run.py` runs a fixed HP-GL job on the stock firmware in a fresh process and prints the time
  counter, strobes, a SHA-1 of RAM and the job report; `tests/test_machine.py::test_golden_stock` compares it with
  `tests/golden_stock.json`. Regenerate that file (`python tests/golden_run.py > tests/golden_stock.json`) only for an
  intended behaviour change, and say why in the commit.
- If a change *does* alter timing or behaviour, say so in the pull request, explain why it is closer to the machine
  (measurement, firmware evidence, manual), and bump `snapshot_state.VERSION` or the cost-table key if old caches would
  otherwise be restored.
- Never "fix" a model constant to make one job's time match. Timing parameters are fitted against measurements on the
  real machine; see [docs/timing.md](docs/timing.md).

## Evidence

State where a model fact comes from: the firmware (address of the code that uses it), a manual, or a measurement on
the machine. Mark assumptions as assumptions, in code comments and in the docs. A device that silently returns a
plausible value for something unknown is worse than one that faults - keep the fail-loudly policy (unknown ports and
unmapped accesses stop the run).

## Tests

```bash
make test        # quick tests; parts that need a ROM image are skipped without one
make test-all    # includes the slow boot and job tests (first run: cold boot, 1-2 min)
```

Tests that need a ROM must skip cleanly when `paths.rom_path()` raises `RomNotFound`, so the suite runs in public CI.

## ROM images

Never commit a ROM image, a snapshot (`*.snap`, it contains the ROM) or a cost table, and never attach them to issues.
If you need to refer to firmware code, use addresses (`8000:366a`) and short descriptions.

## Style

- Python 3.8+, standard library plus the pinned `unicorn` and `capstone`; no new runtime dependencies without a good
  reason. The UI stays dependency-free (standard library HTTP server, bundled three.js).
- Keep the pinned versions: the timing calibration was done with `unicorn==2.1.4` and `capstone==5.0.9`.
- Comments explain *why* and name the evidence; they are in English.
- The C core must build with `gcc -O2 -Wall` without warnings.
- Documentation: facts, numbers and limits; no marketing.
