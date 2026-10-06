"""The second V33 - as an explicit component, and the physics under it.

WHAT THIS IS, PLAINLY. The machine has two V33s and the service documentation
says the Sub CPU controls the X, Y and Z axis motors. Its program has not been
located: the single EPROM holds exactly one reset vector, there is no
independent stack prologue anywhere the main CPU does not execute, the image
holds 41 I/O instructions in total, and nothing is ever downloaded into the
shared windows. Five searches, five negatives (firmware analysis).

BOARD FINDINGS (2026-08-31). The service manual's block diagram settles the architecture and corrects the
central assumption of this file:

    Main CPU -- "RAM for interface" -- Sub CPU -- "Gate array for motor control"
                                                  PWM out, encoder processing

  * The Sub CPU IS the motor controller, and it has **no ROM of its own** in the
    block diagram. `Err 03 SERVO RAM` is the main CPU testing that interface RAM
    at boot, so it can address it - which is why "its program is copied there"
    is now the leading hypothesis (firmware analysis).
  * **The axes are brushed DC SERVOS with incremental encoders**, not steppers.
    Test points TP1..TP9 read "X/Y/Z axis servo motor encoder phase A/B input";
    the manual speaks of "servo off mode" and of a worn Z motor **brush**.
  * What this module calls a "step" is therefore an **encoder pulse**. The
    numbers below are confirmed - Err 40/41 put X and Y at 40 pulses = 0.02 mm
    and Err 42 puts Z at 50 pulses = 0.0125 mm, i.e. exactly the 2000 and 4000
    per mm modelled here - but their NAME was wrong.
  * **The real machine closes the loop and this model does not.** Err 40/41/42
    fire on a following error of 40 (X/Y) or 50 (Z) pulses, and Err 43/44/45 on
    an average current above 1.9 A over 4 s; the manual names "cutting speed is
    too fast" as a cause for both. This model pays owed motion out at a fixed
    rate and can never produce either. That is a NAMED gap: a feed profile that
    the real machine would refuse runs here without complaint.

So this module does NOT execute peer code. It is a **behavioural model** of the
second processor, in one place, with its assumptions named - which is strictly
better than the port stub it replaces, because that stub was answering page
register writes it had been mislabelled as a peer handshake.

The seam for real code is `SubCpu.step()`. When an entry point is found, that
method becomes "run N instructions of a second Unicorn instance sharing this
ROM and this window" and everything around it stays.

WHAT IS MODELLED, AND ON WHAT EVIDENCE
--------------------------------------
- the peer handshake over the window at segment 0x2000: the "MIMAKI" signature,
  the liveness byte, and the 0xAA / 0x55 exchange - decoded in the firmware
  analysis, so this part follows the firmware
- the axes: position, travel limits and endstops - **a physical model, not
  firmware evidence.** Every constant is marked below.
"""


# ---- The frame after the reference run -------------------------------------
#
# THE REST POSITION AFTER THE REFERENCE RUN IS THE FIRMWARE'S "LOW LEFT" ORIGIN.
# Measured in the emulator (2026-09-07): with ORIGIN LOW LEFT, `PA0,0` after
# boot commands not a single pulse; with ORIGIN CENTER exactly +483000/+305000
# (241.5 / 152.5 mm, half the 483 x 305 table area). The end of the reference
# run says the same: after finding the sensor the firmware moves Y by 590000
# pulses (295 mm) and X by 30000 to the rest position - so the Y sensor sits at
# the far end, the X sensor 22.4 mm beside the origin.
#
# Where this origin physically lies is stated by the service manual (4.5,
# REFERENCE POINT CORRECTIONS): pen at the origin, distance to the table edge
# **23 +- 1 mm in X, 8 +- 1 mm in Y**. So the axis can travel at least that
# far beyond the origin. The overtravel beyond the 483 x 305 mm is undocumented:
# ASSUMPTION same margins as at the origin. The axis position `pos` counts from
# the lower stop; `origin_steps` is the origin within it, `table_mm()` the
# table coordinate (0 = LOW LEFT, as HP-GL with ORIGIN LOW LEFT).
#
# Until 2026-09-07 the model's X rest position lay 22.6 mm before the +X stop
# (switch at the high end): almost the whole table lay beyond the stop, and
# every program with +X was cut off ("mutilated") at the stop.
ORIGIN_MARGIN_MM = {"X": 23.0, "Y": 8.0, "Z": 0.0}


class Axis(object):
    """One axis: where it is, how far it can go, and when its switch makes.

    The scale is now confirmed from three independent sides: measured here (50
    per HP-GL unit at 25 um), computed by the firmware itself (`!PZ-333` writes
    13320 = 333 x 40 for Z), and stated by the service manual (`Err 40` = 40
    pulses = 0.02 mm for X/Y, `Err 42` = 50 pulses = 0.0125 mm for Z).

    The unit is an **encoder pulse**, not a motor step - the axes are DC servos.
    The attribute keeps its name so the whole test corpus stays valid; see
    PULSES_PER_MM below for the honest alias.

    The TRAVEL LIMITS come from the model record (483 x 305 mm). The home switch
    position remains an **assumption**: the switch sits at the low end, which is
    where the homing seeks drive.
    """
    STEPS_PER_MM = 2000.0                      # encoder pulses per mm

    def __init__(self, name, travel_mm, home_at_low=True, steps_per_mm=None,
                 margin_steps=200, both_ends=False, sensor_band=None):
        self.name = name
        if steps_per_mm is not None:
            self.STEPS_PER_MM = float(steps_per_mm)
        self.travel_steps = int(travel_mm * self.STEPS_PER_MM)
        self.home_at_low = home_at_low
        self.margin_steps = margin_steps       # switch actuation band
        self.both_ends = both_ends             # one input, a switch at each end
        # An origin sensor with a DOG - the service manual's own word - reads
        # made while the flag is in the beam, i.e. over a BAND of travel, not at
        # a point. sensor_band = (lo, hi) in steps models that directly; it
        # replaces the both_ends hack, which was approximating the same thing
        # by pretending the axis was only as long as the band.
        self.sensor_band = sensor_band
        self.pos = 0                           # steps from the home switch
        self.pending = 0                       # steps still owed to the motor
        self.hit_low = 0
        self.hit_high = 0
        self.clipped = 0                       # commands that hit a hard stop
        self.max_command = 0                   # largest single per-tick command
        self.follow_error_max = 0              # largest backlog beyond that
        self.follow_trips = 0                  # how often above the manual's limit

    # Honest alias. The axes are servos, the unit is an encoder pulse;
    # STEPS_PER_MM keeps its name so that all existing measurements stay
    # valid.
    @property
    def PULSES_PER_MM(self):
        return self.STEPS_PER_MM

    # Following-error limits from the service manual, Err 40/41/42.
    FOLLOWING_ERROR_LIMIT = {"X": 40, "Y": 40, "Z": 50}      # pulses
    CURRENT_LIMIT_A = 1.9                                    # average over 4 s

    # ---- Following error -------------------------------------------------
    #
    # Retrofitting a following-error model revealed that this model already
    # has one. `pending` IS the following error.
    #
    #   owe()      lays down the commanded distance  -> commanded position
    #   advance()  pays it out at a bounded rate     -> actual position
    #   pending    = commanded minus actual          = following error
    #
    # A rate-limited follower, then, and the quantity was there all along - it
    # was just never evaluated. If the firmware demands more than the axis can
    # deliver, `pending` grows, and that is exactly the situation that triggers
    # Err 40/41/42 on the real machine.
    #
    # WHAT THIS NUMBER IS WORTH. The threshold - 40 or 50 pulses - comes from
    # the service manual and is established. The MAXIMUM RATE of the axis in
    # this model, however, is `SubCpu.INSTR_PER_STEP`, a calibration so that the
    # reference run finishes in reasonable time: 20 instructions per pulse at
    # 2000 instructions per tick is about 100 pulses/tick. The firmware caps at
    # 40 (XY-MS, 80 mm/s), so there is a good factor of two in between.
    #
    # CAUTION, and this was wrong in the first attempt: holding raw `pending`
    # against the 40 pulses compares two different things. Right after a
    # strobe it holds the WHOLE per-tick command, normally as much as the feed
    # per tick - up to 40. Even an ordinary approach move then reported
    # "170 % of the limit", which cannot be: the machine would never move. The
    # manual's threshold means the position deviation of the control loop,
    # not the size of a command increment.
    #
    # What is meaningful is therefore the BACKLOG BEYOND ONE COMMAND:
    #
    #     backlog = pending - largest single per-tick command
    #
    # It stays at zero as long as the axis keeps up, and grows monotonically
    # once the commanded profile is faster than the modelled axis. Exactly
    # this growth is the situation that triggers Err 40/41/42 for real.
    #
    # A deflection means: "the commanded profile exceeds what the MODELLED axis
    # can do" - and is only as good as the calibration `INSTR_PER_STEP`. It does
    # NOT mean the real machine aborts at the same point; its limit depends on
    # motor torque, mass and controller, all unknown. As a COMPARISON MEASURE
    # between two profiles the number is reliable, since both see the same
    # calibration.

    # THE actual detector, found while re-checking the trapezoid probes: not
    # the backlog but the COMMAND INCREMENT per tick. The backlog stays
    # unremarkable because the model catches up over the job - the increment,
    # however, jumped from 34 to 1880 when v_peak was raised. That is 0.94 mm
    # in ONE tick, 55 times what the firmware itself ever commands.
    #
    # The firmware's upper bound is known and derived: the long branch at
    # 83422 computes the time as (L*50)/[0x10c4] with [0x10c4] = 2000, i.e.
    # L/40 - at most 40 pulses per tick, corresponding to 80 mm/s at 2000
    # pulses/mm, the XY-MS value from the CONDITION menu.
    #
    # A job that commands more asks the machine for something it never asks
    # of itself. That is the most reliable warning this model can give - it
    # says nothing about the REAL limit of the drives, but it does say that a
    # profile lies outside the range the firmware is designed for.
    # Derived ONLY for X and Y. Z runs on its own profile with its own cap,
    # which is unknown - so Z is not measured against it.
    # (A first attempt did so anyway and reported the perfectly normal Z
    # command of 80 pulses as "2x over the limit".)
    FIRMWARE_MAX_COMMAND = {"X": 40, "Y": 40}

    @property
    def firmware_cap(self):
        return self.FIRMWARE_MAX_COMMAND.get(self.name)

    @property
    def over_firmware_cap(self):
        cap = self.firmware_cap
        return cap is not None and self.max_command > cap

    @property
    def follow_error(self):
        """Backlog beyond one per-tick command, in encoder pulses."""
        return max(0, abs(self.pending) - self.max_command)

    @property
    def follow_limit(self):
        return self.FOLLOWING_ERROR_LIMIT.get(self.name, 40)

    def follow_note(self):
        e = self.follow_error_max
        warn = ""
        if self.over_firmware_cap:
            warn = ("   <-- %dx above what the firmware ever commands (%d)"
                    % (self.max_command // self.firmware_cap, self.firmware_cap))
        elif self.firmware_cap is None:
            warn = "   (no derived cap for this axis)"
        return ("%s: largest per-tick command %d pulses%s\n"
                "        backlog max %d of %d (%.0f %%)%s"
                % (self.name, self.max_command, warn, e, self.follow_limit,
                   100.0 * e / self.follow_limit,
                   ", %dx over the limit" % self.follow_trips
                   if self.follow_trips else ""))

    def move(self, delta):
        """Advance by `delta` steps, clipping at the hard stops.

        A real machine cannot travel past its ends; the emulator used to,
        because position was a free-running accumulator. Clipping here is what
        makes an endstop mean something.
        """
        target = self.pos + delta
        if target < 0:
            self.clipped += 1
            target = 0
        elif target > self.travel_steps:
            self.clipped += 1
            target = self.travel_steps
        self.pos = target

    def owe(self, steps):
        """Queue a displacement. The strobe hands over a whole distance - the
        Z homing seek is one command of -34466 steps - and the real machine
        pays that out over time, which is what lets a switch interrupt it part
        way. Adding to `pending` instead of moving immediately is what makes
        the endstop meaningful at all."""
        self.pending += steps
        a = abs(steps)
        if a > self.max_command:
            self.max_command = a

    def advance(self, max_steps):
        """Pay out up to max_steps of what is owed, and stop at a hard stop.

        Hitting a stop **cancels the rest of the command** rather than clipping
        step by step: the axis is against metal, the remaining distance is not
        going to be travelled, and counting thousands of 'clips' against a stop
        was an artefact of paying the whole distance out anyway."""
        # Record the backlog BEFORE paying out: that is what the axis enters
        # this time slice with.
        e = self.follow_error
        if e > self.follow_error_max:
            self.follow_error_max = e
        if e >= self.follow_limit:
            self.follow_trips += 1
        if not self.pending or max_steps <= 0:
            return 0
        step = max(-max_steps, min(max_steps, self.pending))
        target = self.pos + step
        if target < 0:
            target = 0
        elif target > self.travel_steps:
            target = self.travel_steps
        moved = target - self.pos
        self.pos = target
        if moved != step or target in (0, self.travel_steps):
            self.pending = 0                   # against a stop: command is over
            if moved != step:
                self.clipped += 1
        else:
            self.pending -= moved
        return moved

    def at_home(self, margin_steps=None):
        """Is the home switch made?

        WHICH END the switch sits at is per axis and was MEASURED, not assumed:
        tracing the homing seeks shows X driving -30000 steps, Y driving +30000
        and Z driving -12000. So X and Z home toward zero and **Y homes toward
        its maximum** - which the owner suspected and the motion confirms. An
        earlier version placed all three at the low end, and Y then ran to the
        far stop and pressed against it 5595 times.

        The margin is an ASSUMPTION and matters more than it looks: a real
        limit switch has a wide actuation band and hysteresis, while a value
        that is too small reports "not made" for an axis that is physically
        against its stop.
        """
        if margin_steps is None:
            margin_steps = self.margin_steps
        if self.sensor_band is not None:
            lo, hi = self.sensor_band
            return lo <= self.pos <= hi
        if self.both_ends:
            return (self.pos <= margin_steps
                    or self.pos >= self.travel_steps - margin_steps)
        if self.home_at_low:
            return self.pos <= margin_steps
        return self.pos >= self.travel_steps - margin_steps

    def at_far_end(self, margin_steps=200):
        return self.pos >= self.travel_steps - margin_steps

    def mm(self):
        return self.pos / self.STEPS_PER_MM

    @property
    def origin_steps(self):
        """The LOW LEFT origin in axis steps (see ORIGIN_MARGIN_MM)."""
        return int(ORIGIN_MARGIN_MM.get(self.name, 0.0) * self.STEPS_PER_MM)

    def table_mm(self):
        """Table coordinate in mm: 0 = LOW LEFT origin, negative = beyond the origin."""
        return (self.pos - self.origin_steps) / self.STEPS_PER_MM


# Historical (until 2026-10-07): the fitted Z dog band and the start distance above it. No longer used by the model -
# the Z sensor follows the measured profile below - kept for old analysis scripts that import them.
Z_DOG_AT = 32880
Z_REFERENCE_TRAVEL = 87120

# Port 0x06 bit 6 (Z sensor) against the Z position after homing, MEASURED on a real machine (2026-10-07, firmware
# 1.50MAX build 000022, `me500_testlauf.py zprofil`, two runs with different Z0, 0.25 mm steps down and up, identical
# edges, no hysteresis at that resolution): set from the top position down to 2.00 mm, clear 2.25..3.75 mm, set
# 4.00..6.00 mm, clear from 6.25 mm down. Intervals in mm below the top position (table Z 0) where the bit is set; the
# edges sit halfway between the measured points (+-0.125 mm). Above the top position nothing is measured (the firmware
# never goes there); the bit is taken as set.
# The profile is used throughout, also during the homing run. The firmware's Z fine reference (8000:14f2) centres on
# the clear gap: down until clear (upper gap edge), 3 mm up, down until clear again -> [0x08a8], down until set (lower
# gap edge), to the centre, and declares the centre table [0x06cc] = model record 3 mm * 200; table 0 (the rest
# position) is then 3.0 mm above the gap centre - with this profile exactly the top of the model axis, where the
# measurement was taken. Before that, the coarse reference (8000:148d) finds bit 6 already set at the rest position and
# does nothing. (Firmware analysis: z_reference_table_zero_model.md.)
Z_PROFILE_SET_MM = ((-1.0e9, 2.125), (3.875, 6.125))

class SubCpu(object):
    """The second processor: handshake partner and motor driver.

    `step()` is called by the machine as it runs. Today it advances the
    behavioural model; when the peer's entry point is found it becomes a real
    instruction stepper over the same shared window.
    """

    # the window the two processors share, and the offsets the firmware uses
    WINDOW = 0x20000
    SIG = (0x1a, 0x1c, 0x1e)          # 'MI' 'MA' 'KI', checked at 86768
    LIVENESS = 0x10                   # cleared by the main CPU, peer must set
    HANDSHAKE = 0x00                  # 0xAA from the peer, 0x55 back

    def __init__(self, machine, enable_physics=True, start_distance_mm=10.0):
        self.m = machine
        self.enable_physics = enable_physics
        self._instr_credit = 0
        self.start_distance_mm = start_distance_mm
        # home_at_low per axis, measured from the seek directions (see Axis.at_home)
        # THE CONFIGURATION THAT REFERENCES AND PLOTS. Forced by the
        # firmware, by walking the error it raises forward - ERR50 X, ERR51 Y,
        # ERR52 Z - and by measuring the coordinate scale afterwards.
        #
        #  * 2000 steps/mm on X and Y. The coordinate scale measures exactly
        #    50 strobe steps per HP-GL unit, and 50 steps per 0.025 mm - the
        #    standard unit - is 2000/mm. An earlier 100/mm was compensating for
        #    an unrealistic power-on position, not a real resolution.
        #  * X's switch at the HIGH end, Y's at the LOW end.
        #  * Z travel 20240 steps with a switch at BOTH ends on one input line:
        #    two Z phases need bit 0x40 made 20000 steps apart.
        #  * 300 steps of actuation band. At 200 an axis sat 20 steps outside
        #    its own switch after backing off.
        #
        # start_distance_mm is a MACHINE FACT, not a convenience. The homing
        # seek is bounded at 30000-40000 steps, i.e. 15-20 mm, so the head has
        # to be parked near its switches at power-on. Measured window: the
        # reference completes at 5, 10 and 14 mm and fails at 2 mm (ERR50) and
        # at 20 and 40 mm (never finds the switch). The real machine therefore
        # does NOT reference from an arbitrary position.
        # MECA CORRECT (2026-09-25, pulse surplus explained): system parameters no. 24/25 (NVRAM 0x030 X,
        # 0x032 Y; machine 5998/5999, ROM default 6000) are the factory axis-scale correction of THIS
        # machine. The firmware scales position -> pulses by 6000/corr (82b83: fixed-point mul, +0x20,
        # >>6; then x10 at 82bf5) - so at 5998, +1 pulse per 3000 steps (staircase, edges measured at
        # 1500/4500/7500). The mechanics this correction compensates belong in the axis model: real
        # pulses per mm = 2000 * 6000 / corr. The bench seed (6000/6000) stays exactly 2000.
        def _meca(cell):
            try:
                nv = machine.nvram.cells
                v = nv[cell] | (nv[cell + 1] << 8)
            except Exception:
                return 6000
            return v if 3000 <= v <= 12000 else 6000
        spm_x = 2000.0 * 6000.0 / _meca(0x030)
        spm_y = 2000.0 * 6000.0 / _meca(0x032)
        self.axes = {
            # 483 x 305 mm table plus the margins beyond the origin (see
            # ORIGIN_MARGIN_MM): the reference run still runs against the
            # stops, afterwards rebase_to_origin() moves the rest position to
            # the origin.
            "X": Axis("X", 483.0 + 2 * ORIGIN_MARGIN_MM["X"], home_at_low=False, steps_per_mm=spm_x,
                      margin_steps=300),
            "Y": Axis("Y", 305.0 + 2 * ORIGIN_MARGIN_MM["Y"], home_at_low=True, steps_per_mm=spm_y,
                      margin_steps=300),
            # Z, at the resolution and stroke the service manual documents:
            # 4000 pulses/mm over a **62 mm** stroke.
            #
            # 62 instead of 60 since 2026-08-31: system parameter no. 23 "Maximum
            # valid area Z axis" is 62 (unit mm) in the EEPROM, and the value
            # comes from the same table that also supplies the 483 x 305 for X
            # and Y. The 60 was a rounding from the manual.
            # The earlier 2000/mm over 20240 steps with a sensor at both ends
            # was a fit that contradicted the manual, and it is gone.
            #
            # The origin sensor is a DOG - the manual's own word - 12000 steps
            # (3 mm) wide, and the reference is OPEN LOOP in distance: the
            # firmware commands -55120 steps blind and then -32000 in one
            # continuous move, 87120 in all, and expects the dog to be there.
            # It does not seek until found. So Z has to start about 87120 steps
            # (21.8 mm) above the dog, and the dog's 3 mm width is the whole
            # tolerance on that.
            #
            # Verified: the reference completes from starts of 100000, 120000,
            # 140000 and 180000 steps with the dog at start-87120, and with dog
            # widths of 8000 and 12000 steps. It fails at 1200 and 4000 - too
            # narrow to catch the move - and at 24000 - too wide, the sensor
            # then never releases for the phase that waits on that.
            # 2026-10-07: the fitted dog band (Z_DOG_AT +- 6000) is gone; the Z sensor follows the measured profile
            # Z_PROFILE_SET_MM (SubCpu.z_sensor). The model's top end of travel is the rest position (table 0). On the
            # machine there is some room above it - at least 0.875 mm (the reference's 3 mm hop from the upper gap
            # edge goes that far above the rest position without ERR42), less than 2 mm (G0 Z5 with Z0 3 mm below the
            # top gave ERR42) - which the model does not have; nothing but the homing hop goes there.
            "Z": Axis("Z", 62.0, home_at_low=True, steps_per_mm=4000,
                      margin_steps=600)}
        # Where the axes are at power-on. A real machine is wherever it was
        # switched off, which is why it homes at all - starting every axis on
        # its home switch would be the one position that never occurs in
        # practice and would make the homing escape move fire immediately.
        # ASSUMPTION: 40 % of travel.
        for k, a in self.axes.items():
            # a fixed distance from the axis's OWN switch - see
            # start_distance_mm above; the seek is bounded and cannot reach a
            # switch that is further away than it travels
            if k == "Z":
                # Z starts at its rest position (the top), as a machine is normally switched off and on: the
                # reference then runs the measured path (coarse seek skipped, gap centred 3 mm below).
                a.pos = 0
                continue
            d = int(self.start_distance_mm * a.STEPS_PER_MM)
            a.pos = (min(a.travel_steps, d) if a.home_at_low
                     else max(0, a.travel_steps - d))
        self.handshake_state = "idle"
        self.handshakes_completed = 0
        self.liveness_pulses = 0
        self.steps_consumed = 0
        self.rebased = False

    def rebase_to_origin(self):
        """After the reference run: the rest position IS the LOW LEFT origin (measurement
        and manual, see ORIGIN_MARGIN_MM). The model's reference run ends at the stops
        (X 22.6 mm before the high one, Y at the low one) because the sensors are there;
        here the axis position is set to the origin so that the 483 x 305 mm table lies
        physically in front of the axis. The reference-run counters (stop hits) are
        zeroed - they are model artefacts. Once per boot.
        Also Z since 2026-09-25: depending on the MECA ORIGIN corrections in NVRAM
        (block 0x024, on the machine 16/-16/3 instead of ROM 15/-10/5) the firmware's
        reference run ends at different model positions (machine seed: 7620 pulses =
        1.905 mm away from the bench seed), while the firmware afterwards carries its
        position as (0,0,0) - the anchoring puts the model on the same zero point. With
        the bench seed the Z step is a no-op (the run ends at the origin there anyway)."""
        if self.rebased:
            return False
        for k in ("X", "Y", "Z"):
            a = self.axes[k]
            a.pos = a.origin_steps
            a.pending = 0
            a.clipped = 0
            a.hit_low = 0
            a.hit_high = 0
        self.rebased = True
        return True

    # ---- the shared window -------------------------------------------
    def _peek(self, off):
        try:
            return self.m.cpu.uc.mem_read(self.WINDOW + off, 1)[0]
        except Exception:
            return 0

    def _poke(self, off, val):
        try:
            self.m.cpu.uc.mem_write(self.WINDOW + off, bytes([val & 0xFF]))
        except Exception:
            pass

    def sign_window(self):
        """Write the "MIMAKI" signature the main CPU looks for at 86768.

        Evidence-backed: the firmware tests es:0x1a/0x1c/0x1e for 'MI','MA','KI'
        and takes the short path when they are present.
        """
        for off, pair in zip(self.SIG, (b"MI", b"MA", b"KI")):
            self.m.cpu.uc.mem_write(self.WINDOW + off, pair)

    def step(self):
        """One service opportunity for the peer.

        THE SEAM. Real peer code would run here. Today: keep the handshake
        alive and let the liveness byte answer.
        """
        # the main CPU clears the liveness byte and expects it to come back
        if self._peek(self.LIVENESS) == 0:
            self._poke(self.LIVENESS, 1)
            self.liveness_pulses += 1
        # the 0xAA / 0x55 exchange
        h = self._peek(self.HANDSHAKE)
        if self.handshake_state == "idle" and h == 0:
            self._poke(self.HANDSHAKE, 0xAA)
            self.handshake_state = "posted"
        elif self.handshake_state == "posted" and h == 0x55:
            self._poke(self.HANDSHAKE, 0)
            self.handshake_state = "idle"
            self.handshakes_completed += 1

    # ---- the motor side ------------------------------------------------
    def consume_step(self, dx, dy, dz):
        """A strobe from the motion window: move the axes.

        This is where the Sub CPU's real job would happen. A strobe carries a
        whole DISTANCE, not one step - the Z homing seek arrives as a single
        command of -34466 steps - so the axes only take it on as owed motion
        and `advance()` pays it out over time. Moving immediately made every
        seek teleport past its switch before the firmware could read the
        sensor, which is exactly what stalled the boot with physics on.
        """
        self.steps_consumed += 1
        if not self.enable_physics:
            return
        self.axes["X"].owe(dx)
        self.axes["Y"].owe(dy)
        self.axes["Z"].owe(dz)

    # One step per this many instructions, per axis. CALIBRATION, not a
    # measurement - the CPU clock is unknown, so this cannot be converted to
    # mm/s. The order of magnitude does check out: the panel reports XY-ES 20,
    # and 20 mm/s at 2000 steps/mm is 40000 steps/s, which at a few million
    # instructions per second lands near 50. Lower values simply make homing
    # finish sooner in emulated instructions; the value does not change which
    # branches the firmware takes, only how long it waits.
    INSTR_PER_STEP = 20

    def advance(self, instr_delta):
        """Let time pass: pay out owed motion at the step rate."""
        if not self.enable_physics or instr_delta <= 0:
            return
        self._instr_credit += instr_delta
        steps = int(self._instr_credit // self.INSTR_PER_STEP)
        if steps <= 0:
            return
        self._instr_credit -= steps * self.INSTR_PER_STEP
        for a in self.axes.values():
            a.advance(steps)

    def follow_report(self):
        """Following error per axis - the measure a feed profile has to be
        judged by. See the block at Axis.FOLLOWING_ERROR_LIMIT for what the
        number says and what it does not."""
        return {k: {"max": a.follow_error_max, "limit": a.follow_limit,
                    "trips": a.follow_trips}
                for k, a in self.axes.items()}

    def follow_lines(self):
        return [a.follow_note() for a in self.axes.values()
                if a.max_command or a.follow_error_max or a.follow_trips]

    def over_cap(self):
        """Axes whose per-tick command exceeds what the firmware itself ever
        issues - the most reliable warning this model gives."""
        return {k: a.max_command for k, a in self.axes.items()
                if a.over_firmware_cap}

    def endstop_bits(self, idle=0x8F):
        """Port 0x06 as the axis positions imply it.

        Polarity is established: bit set = switch actuated.

        **The bit-to-axis assignment is open (unresolved).** It was briefly claimed to be
        derived - bit 0x20 = X, 0x10 = Y, 0x40 = Z - by pairing each back-off
        correction with "the axis that seeks in the opposite direction". That
        derivation is **withdrawn**: the +-30000 moves it used are not the
        homing seeks. Only two strobes precede the first sensor read, both
        open-loop (X +32809, Y -38768, Z -55120 at 805ed and 80656), and every
        +-30000 move happens afterwards - they are the back-offs and
        re-approaches, so the pairing had no basis.

        The assignment below is kept only because eight combinations of slot
        group and switch end were tested and **none** boots, so nothing here is
        currently better supported than anything else. Treat it as a placeholder.
        """
        v = idle & ~0x70 & 0xFF
        if self.axes["X"].at_home():
            v |= 0x20
        if self.axes["Y"].at_home():
            v |= 0x10
        if self.z_sensor():
            v |= 0x40
        return v

    def z_sensor(self):
        """Z sensor (port 0x06 bit 6) from the measured profile: depth below the top position in mm."""
        mm = self.axes["Z"].table_mm()
        return any(lo <= mm < hi for lo, hi in Z_PROFILE_SET_MM)

    def report(self):
        return {
            "handshakes": self.handshakes_completed,
            "liveness_pulses": self.liveness_pulses,
            "steps_consumed": self.steps_consumed,
            "axes": {n: {"steps": a.pos, "mm": round(a.mm(), 3),
                         "travel_mm": round(a.travel_steps / a.STEPS_PER_MM, 1),
                         "at_home": a.at_home(), "clipped": a.clipped}
                     for n, a in self.axes.items()},
        }
