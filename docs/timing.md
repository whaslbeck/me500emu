# Timing

How emulated time relates to machine time, how that was calibrated, how well it agrees with the real machine, and
which numbers to trust.

## The time scale

The emulator's clock is `m.instr`. Despite the name it is a **time unit of 0.5 us**:

    2000 units = 1 ms         seconds = (m.instr_end - m.instr_start) / 2_000_000

`session.INSTR_PER_S = 2000000` is that constant; `Session.time_s()` and the runner's `machine_time_s` use it.

The anchor is the machine's motion tick. Measured on a real ME-500:

| Quantity | Machine | Emulator |
|---|---|---|
| Motion tick (INT 20h, the step engine) | 1000/s (measured 1001.6/s) | every 2000 units = 1.000 ms |
| 8253 input clock | 4.9152 MHz | (used to convert the counter values below) |
| Kernel timer INT 23h (counter 1, 50000 counts) | every 10.17 ms | every 20340 units |
| Service tick INT 24h (counter 2, 5000 counts) | ~941/s measured | every 2035 units (1.017 ms, the nominal counter value) |
| Cost of one V33 instruction | 0.25 ... 1.2 us, depending on the class | 1 unit (0.5 us) plus a class delta, see below |

At cold boot the two timer intervals can be overridden with the environment variables `INT23` and `INT24` (in
units); this changes the boot and therefore the snapshot, so only use it for experiments.

## Instruction costs

A flat "one instruction = 0.5 us" is wrong in both directions on a V33: a register instruction is much cheaper, a
port access or a `rep movsw` iteration much more expensive. The firmware's motion tick handler is dominated by memory
accesses and jumps, the frontend by other mixes, so a flat rate distorts the relative speed of the parts of the
firmware - which is exactly what decides how fast a job runs.

The cost model (`costs.py`), applied after the boot:

| Class | Cost in units (0.5 us) | Calibrated against |
|---|---|---|
| register-only, jumps | 0.4375 | mix `B0` |
| memory read | 1.1875 | `B1` |
| memory write | 1.125 | `B2` |
| read-modify-write, `movs` | 1.9375 | (no mix of its own) |
| multiply / divide | 3.1875 (+ read) | `B3` |
| port `in`/`out` | 2.4375 | `B5` |
| `rep` word string op, per iteration | 1.3125 | `B6` |
| access to the sub-CPU window `2000:00xx` | +1.5 | `B4` |
| any instruction in the executor range `0x84300..0x86300` | +0.5 | tick-trace anchors, workshop run |

**How it was calibrated.** A test build of the firmware ran seven fixed instruction mixes (`B0` ... `B6`: register,
RAM read, RAM write, multiply/divide, sub-CPU RAM, port I/O, `rep movsw`) on the real machine and timed them with the
8253 at 4.9152 MHz. The class costs were fitted until the emulator reproduced all seven, each within 4 %. The class
costs alone left the step engine's tick handler too fast (it is jump-dense and indexed-memory heavy); the executor
range surcharge was then fitted against a tick-by-tick timing trace of the same handler recorded on the machine
during a workshop run. `KOST_EXEC` (environment, sixteenths of a unit per instruction, default 8) overrides that
surcharge for calibration experiments.

The costs are **armed only after the boot**. The boot's homing run was tuned on flat time; arming the costs earlier
makes it fail with `ERR52` (Z sensor). This does not affect jobs: every job runs from a snapshot taken after the boot.

## Validation against the real machine

Workshop run on 2026-09-25, same jobs on the machine and in the emulator:

| Job type | Machine | Emulator | |
|---|---|---|---|
| Chained XY corner records | 21-24 ms per record (noise about +-10 ms) | 19-20 ms per record | within measurement accuracy |
| Block jobs, 9600 baud | reference | within +-1 % | |
| Block jobs, 19200 baud | reference | -3 % / -4 % | host-side scatter |
| Z records | reference | about 0.3-0.5 s **shorter per Z record** | open |

A later acceptance run with a 1.50MAX build found the remaining differences in Z moves, rapids and the AUTO VIEW
behaviour to be within the resolution of that measurement. The Z shortfall above is therefore not settled either way;
treat absolute Z-heavy times with caution.

Earlier versions of the model (before the tick was measured and before the class costs) were off by up to a factor of
two on short-segment jobs. Numbers from those versions are not comparable with the current ones.

## What to trust

In descending order of reliability:

1. **Whether a job runs, where it goes, where it ends.** The firmware computes every motion record itself; the path,
   the bounds and the final position are the firmware's, to the encoder pulse (subject to the axis model's hard stops).
2. **Relative time of two toolpaths** on the same emulator, ROM, NVRAM and baud rate - for example two
   post-processor variants, chords vs. arcs, two feed settings. Systematic errors largely cancel.
3. **Absolute XY machine time**: within a few percent of the machine at 9600 baud.
4. **Absolute time of Z-heavy jobs**: probably too short (see above).

Not covered at all: anything the servo loop decides on the machine (following-error and overcurrent aborts), and jobs
that hit the factory ramp fault (see [hardware-model.md](hardware-model.md#what-is-not-modelled)).

The serial line is part of the timing. A job of many short segments can be limited by the 9600-baud line rather than
by the motion; the emulator models the line at its byte rate, so that effect is included. A host that inserts its
own pauses (block-by-block acknowledgement, slow sender) is not: `run_job` streams as fast as flow control allows.

## Converting

```python
seconds = delta_units / 2_000_000
units   = int(seconds * 2_000_000)
ticks   = delta_units / 2000            # motion ticks (1 ms each)
```

- `run_job` reports `machine_time_s` from the start of streaming to the **last change of position**; the idle time the
  runner waits afterwards (about 1.2 s of machine time) is not included.
- `m.motion.strobes` counts motion commands to the sub CPU, about one per 1 ms tick while the machine moves. The UI's
  job time is derived from strobes (0.959 ms per strobe) and can
  therefore differ slightly from `machine_time_s`.
- **Wall clock:** with the C core the emulator runs about 2-5x faster than real time on a desktop CPU. A cold boot
  (~95 M units) takes 1-2 minutes; restoring a snapshot about a second. The bridge's `--realtime` option pins emulated
  time to the wall clock instead.
