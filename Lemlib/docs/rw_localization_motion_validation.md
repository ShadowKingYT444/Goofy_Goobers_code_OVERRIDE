# Verification for RW-based Skills improvements

This is the acceptance plan for [the implementation proposal](rw_localization_motion_plan.md).
All implementation and field checks below are **pending** in this documentation
PR. Keep evidence from the exact tested commit and configuration.

## Baseline checks

- [ ] Record the current main SHA and compare it with the plan's baseline.
- [ ] Record actual command limits for every modified motion. Keep the existing
  limit at each call; do not raise a default or override indirectly.
- [ ] Record route coordinates, load count, lift/arm policy, and feature modes.
- [ ] Run the existing host suite on the unchanged base. Record failures and
  missing dependencies before editing. Do not weaken assertions to hide a new
  failure. If a test describes an obsolete policy, document the source/history
  conflict separately before changing it.
- [ ] Save the current Brain report and physical endpoint measurements when
  hardware is available. A missing field baseline remains a pending check.

The old optimization document's 42-test result belongs to its earlier commit.
It is not a current test result for this branch or the future implementation.

## Host tests for production logic

Keep the math, progress accounting, observation checks, and outcome state machine
testable without hardware. Use the actual production helpers. Tests that repeat
their implementation in a fake helper do not validate the production behavior.

| Area | Case | Required result |
| --- | --- | --- |
| Progress | Pose changes by several inches while encoders do not move | Zero added travel; no clearance event. |
| Progress | Equal/opposite wheel movement with an in-place IMU turn | Approximately zero center translation within numeric tolerance. |
| Progress | Known straight forward and reverse travel | Correct signed increments; accumulated travel advances by the known magnitude. |
| Progress | Heading crosses 0/360 degrees | No false full-turn jump or false travel. |
| Progress | Encoder reset, disconnected motor, nonfinite sensor, or invalid time delta | Invalid/reseeded sample; no fabricated progress. |
| Motion identity | Motion A ends; motion B starts while an A waiter remains | A cannot consume B's progress or cancel B. |
| Outcomes | Library ends before the requested event | Event remains unreached; unknown internal reason is retained. |
| Outcomes | Wrapper deadline or cancellation before the event | Explicit cause; no reached result. |
| Outcomes | Event reaches its predicate before completion | Reached state belongs to the correct motion and run. |
| Outcomes | Starts inside an early-exit region | No false claim that a crossing occurred. |
| Timing | Clock rollover and variable task intervals | Bounded waits and correct elapsed-time accounting. |
| Ownership | Queued LemLib motion, controller replacement, abort, and driver takeover | One writer; old and queued writers cannot resume after handoff/brake. |
| Contact | Early obstruction, timeout short of target, or wheel spin | No unsupported physical-contact assertion or stronger reference classification. |
| Geometry | Field/route transform round trip and south-wall correction sign | Correct units, axes, heading convention, and sign. |
| Geometry | Different sensor offsets at a small valid angle | The two corrected wall coordinates agree for a consistent synthetic pose. |
| Rejection | Range sentinel, disagreement, scatter, excessive angle, corner ambiguity, or full correction over 4 inches | Explicit reason; no pose mutation. |
| Timing | Known delay under constant motion and changing heading | Historical observation is propagated to the correct current coordinate. |
| Timing | Missing history, excessive latency uncertainty, or equal repeated values | Missing/uncertain history rejects; equality alone is not proof of staleness. |
| Anchors | Contact/start anchor after a range was acquired | Old-generation observation cannot alter the new pose. |
| Correction | Odometry update and axis correction occur together | Unobserved axis/heading preserved; movement neither lost nor counted twice. |
| GPS | Many small wall corrections below the teleport threshold | GPS estimator does not count them as extra travel or consume a correction twice. |
| Modes | OFF, OBSERVE, stopped fast path, and unavailable moving APPLY | New observer makes no pose writes in OFF/OBSERVE; existing resets remain active; fast path has bounded fallback; unsupported APPLY cannot activate. |
| Caps | Equal positive/negative demands, opposite-sign turns, reverse curves, and large catch-up | Final wheel commands stay within the configured limit. |
| Profiles | Saturation, sensor error, mode change, and timeout | Explicit result; controlled termination; no continued writer after brake. |
| Route | G3 return index 1..4 versus final index 5 | South-arrival experiment applies only where another pickup follows. |
| Clearance | Six inches of curved travel but insufficient straight separation | Does not substitute for required straight clearance. |
| Fallback | Rejected goal reference, unknown pose, or arm/lift envelope not ready | Existing route remains available; no assumption of a perfect start pose. |

Choose numeric tolerances from the units, calibration, and physical clearance
requirements. Record them before comparing candidate field results.

## Build checks

From `Lemlib/`, capture the exact toolchain and commands used. Preserve the
repository's hot/cold package configuration. For a PROS-equipped environment:

```bash
pros make clean
pros make
python3 -m unittest discover -s tests -v
pros make clean
pros make EXTRA_CXXFLAGS=-DAUTON_ROUTINE=skills
strings bin/hot.package.bin | rg '^AUTON_RESULT,'
pros make clean
pros make
strings bin/hot.package.bin | rg '^AUTON_RESULT,'
python3 -m unittest discover -s tests -v
```

Each inspected image must contain exactly one `AUTON_RESULT,` format string,
with the selected routine name: `skills` for the Skills build and the configured
three-pin routine for the final default build. Additional format fields after
the routine name are expected. Reject a mixed or stale routine image. Check the
firmware-image tests against the fresh default images. If the supported
environment uses direct `make` instead, record the equivalent commands and
dependencies. Do not count an old binary as a fresh build, or a skipped image
check as a pass.

If LemLib is patched, also build the pinned dependency from its patch and verify
the final application links that artifact. Save the patch, dependency version,
and build command. Do not upload a build as part of this documentation task.

## Field comparison

### Keep the comparison controlled

Use the same field, starting pose, payload, mechanism settings, and command
limits. Alternate baseline and candidate runs under similar battery conditions.
Start with at least five baseline/candidate pairs for the isolated movement.
This is an initial engineering screen, not a statistical reliability guarantee.

Use tape marks, measured field references, or calibrated video to measure final
position and heading. The same odometry that drives the robot cannot serve as
independent proof that localization improved.

Store commit, build flags, run ID, battery condition, route/load index, configured
caps, and measured sensor mounts with each result. Save a Brain-screen summary;
serial or SD logs can hold the detailed samples.

### Required comparisons

| Candidate | Compare the complete interval | Check physical behavior |
| --- | --- | --- |
| South-wall observer | G2 departure through the next stopped reset | Proposed versus stopped correction, actual endpoint, coverage, and rejection reasons. |
| Stopped fast reset | Arrival/alignment through start of the next move | Same correction quality, no stale observation after a turn/contact, and working fallback. |
| Moving correction | Entire enabled G2-return window plus next pickup | No premature clearance event, GPS estimator feedback, worse loader alignment, or extra recovery. |
| G3 return facing south | Goal departure through the next reset/alignment | Straight clearance, actual arm/lift envelope, wall/body clearance, and arrival heading. |
| Loader path | Start of entry through clamp and usable pickup clearance | Square contact, correct lateral placement, preserved dwell, and successful pickup. |
| Park handoff | Departure from toggle through final stopped park | Corridor clearance and final position; no corner cutting. |
| Profiled move | Motion start through any final correction/settling | Fixed final output limits, saturation time, slip, overshoot, and actual arrival. |

For complete Skills runs, record successful placements and actual score along
with completion time. Include failed and aborted runs. A run that skips or
misses a placement is not a faster successful run.

### Suggested report fields

Extend existing timing data without changing the meaning of `SKILLS_PHASE`:

```text
run_id, commit, mode, trip, goal, load_state, motion_id
drive_cap, turn_cap, total_ms, reset_sample_ms, reset_align_ms
reset_recenter_ms, touchup_ms, motion_stop_cause, event_state
sensor_a, sensor_b, read_time, assumed_latency, latency_uncertainty
observation_generation, observation_age, wall_axis, rejection_reason
proposed_correction, applied_correction, saturation_ms, fallback_count
physical_position_error, physical_heading_error, pickup_ok, score_ok
```

These are a proposed schema, not an implemented API. Distinguish missing values
from zero. Label enclosing and nested timing intervals; do not add overlapping
waits to trip time twice. Report mechanism outcomes under the current policy.

## Acceptance and rollback

- [ ] Host regressions pass, with any pre-existing failures identified separately.
- [ ] Fresh default and Skills packages build. The default is restored.
- [ ] Final wheel commands respect each tested motion's existing limits in all
  normal, catch-up, reverse, saturation, cancellation, and fallback cases.
- [ ] Required physical clearances and placement tolerances are met on all
  observed accepted trials. Explain any failures; do not exclude them silently.
- [ ] Report median and worst observed elapsed time with success counts.
  Compare complete intervals and show the raw trial data.
- [ ] The candidate improves the chosen time/alignment measure without an
  observed reduction in pickup/score success or required clearance. Repeat
  across the payload states that will use it before enabling it generally.
- [ ] Remove a scheduled reset only if a qualified observation remains valid
  after the latest relevant turn/contact. Demonstrate the fallback when it is not.
- [ ] Verify each experimental switch can restore the baseline path. Keep full
  MCL and RAMSETE disabled unless separately implemented and evaluated.

If a candidate fails, retain its diagnostic results and restore the baseline
behavior. Do not compensate by increasing drivetrain speed or quietly relaxing
clearance, sensor, or success criteria.

## Final agent report

State what was implemented, what is enabled, what is still a proposal, and which
checks actually ran. Include source commit, build configuration, test output,
field evidence, and unresolved measurements. Desktop simulation results must be
labeled as simulation. Do not report field time savings from geometry alone.
