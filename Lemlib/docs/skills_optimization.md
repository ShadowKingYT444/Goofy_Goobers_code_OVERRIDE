# Skills: less waiting, with the same drive speeds

This change removes avoidable waiting and exposes mechanism failures. The match-load loop
keeps its drive cap of 60, turn cap of 50, and loader line-up at field
**(-48, -59.25)**. The baseline is main commit `dcf9c7c`. Host tests cannot prove
faster field runs or reliable scoring.

## What changes

| Change | Reason and guardrail |
| --- | --- |
| One scoring readiness budget | Repeated checks share one allowance. Measured lift and arm positions must be ready before final approach/release; an expired deadline is not permission to score. |
| Arm deadline runs inside its worker | The 1.5-second limit applies during driving too. Finishing a job does not prove target position. Existing motor-power behavior is retained. |
| One low-load goal aim | Aim before raising, check rotation has stopped, and correct heading if needed. This replaces the second aim after raising began. |
| Pickup clearance recognizes a successful target | A successful lift within target tolerance should not exhaust the timer because it moved 59 degrees rather than 60. Failures and invalid readings do not count as clearance. |
| Stall detection uses the motion's start | Entering the wait helper late no longer grants another launch grace period. The sustained low-speed requirement remains. |
| Final arm-assisted G3 descent to carry height | On the final planned arm-assisted G3 trip with toggle/park enabled, avoid visiting the floor and raising again. Preserve release and goal clearance. |
| Wall samples end when complete | Keep all six successful samples and their spacing; omit the sleep after the final sample. Discard a partially collected batch if the wheels move. |
| Apply resets to the current pose | Preserve the latest heading and unaffected coordinate. After an accepted reset, recenter from remaining line-up error, rather than the size of the odometry correction. |

Lift commands need one owner: a replacement cannot start while an older worker
still commands the motor. Preserve loader contact/dwell, low-load turns, physical
release checks, and six-inch goal clearance. The
[motion journal](../journal/motion_and_pid.md) records missed alignment from
rolling handoffs and turning after raising.

## What can be claimed before field testing

The normal eight-load route makes **13 wall-reset attempts**. With six valid,
stationary samples every time, removing the final sleep changes the requested
sampling delay from 60 to 50 milliseconds per reset: **130 milliseconds across the
run**. Invalid readings, motion, refusals, and scheduling affect the real total.

Other gains need field measurements. Removing a duplicate turn
does not save its entire timeout. Removing false readiness waits helps only when
they occur. The final lift shortcut removes travel, not a known number of seconds.
Motion timeouts are maximum allowances, not fixed delays. Lift and arm work
already overlap some driving; do not count that work twice.

## Reading the timing report

A **trip** covers collecting a load, travelling to its goal, scoring, and leaving.
Its time includes wall resets and mechanism waits. The Brain's wait
totals explain portions of that time:

| Brain total | What it helps diagnose |
| --- | --- |
| `wall` | Time spent in the wall-reset helpers, including sampling and rejection checks. |
| `down` | Blocking at the loader while waiting for the lift and arm to be down. |
| `up` | Waiting for score readiness; `stack_ms` in serial. |
| `pickup` | Blocking for the newly gripped load to clear the floor. |

The per-trip `wall` columns show corrections/refusals, not seconds.
The wait totals measure blocked time. Do not add them to trip time. `SKILLS_PHASE` lines detail the loader,
aim, goal approach, release, and return. Read a phase duration as an enclosing
interval when it contains a wait; it is not automatically another independent
cost. Keep failures and reset refusal reasons alongside timings.
An aborted trip has `ok=0`. It is not a faster successful run; a completed route
counter does not prove every element landed.

## Field A/B check

1. Compare builds with the same route constants, caps, start, sensor offsets,
   loader setup, and field. The latest -59.25 lane
   also needs validation; do not compare it with an older lane and attribute the
   difference to scheduling.
2. First observe pickup, a low score, and an arm-assisted high score. Check that
   lift/arm failures stop the attempt, pickup clears the floor, and turning ends
   before raising. Test failure handling in a controlled setup, without people
   touching powered mechanisms.
3. Alternate baseline/changed full runs under similar battery conditions.
   Save the Brain report, serial output, and video of each run, including failures.
4. Compare completion time and successful placements together. Use wait totals
   to locate changes; use video to confirm release, loader contact,
   stack stability, and the final carry-height transition.
5. Accept improvements that survive repeated runs without
   worse alignment or missed placements. Retain the data before changing another
   route or speed setting.

## Later field experiment: curve on the G3 return

**This curve is a proposal, not enabled code in this change.** Initially test only
the first four low/empty G3 returns; the final arm-assisted return has different
clearance needs.

From the nominal released robot center **(-32.38, -47.62)**, go west 6.24 inches to
**(-38.62, -47.62)**. A quarter-circle of radius 9.38 inches then reaches
**(-48, -57)** facing south. Continue straight 2.25 inches to **(-48, -59.25)**;
stop there for the wall reset. Require the lift low and arm clear before curving;
otherwise retain the existing straight-and-pivot return.

This centerline is about **23.224 inches instead of 27.250**, saving **4.026 inches
per return** and replacing one pivot. Four returns save 16.104 inches of planned
distance, not a known number of seconds. A larger arc directly to the new line-up
would start turning before the required six-inch clearance. Even this proposal
leaves only about **3.38 inches** of nominal south-wall clearance for a 16-inch
body at the endpoint; extensions, field objects, tracking error, and actual curve
following require separate field checks.

## Reference behavior

Official references: [LemLib 0.5.6 motion chaining](https://lemlib.readthedocs.io/en/v0.5.6/tutorials/8_motion_chaining.html)
and [PROS distance-sensor API](https://pros.cs.purdue.edu/v5/pros-4/group__cpp-distance.html).

## Code verification (2026-10-18)

- All **42 tests passed** with `python3 -m unittest discover -s tests -v`,
  including 29 new control, scoring, reset, and contact regressions. The six
  firmware-image checks ran against the freshly linked default packages.
- Clean hot/cold builds passed for both default three-pin and Skills with
  Arm GNU Toolchain 15.2.1, PROS 4.2.2, LemLib 0.5.6, and liblvgl 9.2.0.
  The ARM builds reported no warnings.
- Skills was selected with `make -j4 quick EXTRA_CXXFLAGS=-DAUTON_ROUTINE=skills`
  after `make clean`; its binary contained `AUTON_RESULT,skills`. A final clean
  default build restored `AUTON_RESULT,three_pin_auton`.
- Independent flow review checked motor ownership, unchanged route/speed
  settings, failure handling, and recovery on a subsequent attempt. Host
  scoring tests compile the production function bodies with hardware fakes.

The checks establish code behavior and successful linking. The field A/B work
above is still needed to establish elapsed-time gains and scoring reliability.
