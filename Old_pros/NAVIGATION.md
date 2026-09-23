# Navigation library

> **2026-08-25 safety hold:** do not treat a successful `navigation::init()` as
> sufficient motion authority yet. One API-ready P6 calibration stayed quiet
> initially, then crossed 0.50 degrees of false rotation after 183.4 seconds
> and drifted at 1.074 degrees/min while all drive encoders and P7 heading said
> the robot was stopped. A warm recalibration was stable, but the production
> API does not yet implement the validated session-scoped calibration-quality
> gate. Keep autonomous motion disabled until that gate passes live stopped and
> turning tests. See
> `reports/sensor_campaign_2026-08-25/live_resume_addendum.md`.

Include `main.h` (which includes `navigation.hpp`) and use field coordinates in
inches. The robot center is the pose origin. Heading uses normal field math:
0 degrees is +X, 90 degrees is +Y, and positive rotation is counterclockwise.

```cpp
void autonomous() {
if (!navigation::init(30.0, 35.0, 150.0)) {
    navigation::stop();
    return;
  }

  const auto first = navigation::go_straight_to(20.0, 45.0);
  if (first != navigation::Result::kSuccess) {
    printf("navigation failed: %s\n", navigation::result_name(first));
    navigation::stop();
    return;
  }

  navigation::turn_to(90.0);
}
```

The three-coordinate `init()` uses a provisional 1-inch radial start-placement
error. If a starting fixture/tape procedure has a measured conservative bound,
pass it as the fourth argument, for example
`navigation::init(30.0, 35.0, 150.0, 0.5)`. Zero is accepted only when the
caller can actually justify a zero-error anchor; it is not the default.

`navigation::update()` should be called about every 20 ms whenever navigation
is idle. `turn_to()` and `go_straight_to()` are blocking and update the fused
pose internally. `current_pose()` returns X, Y, heading, and a `valid` flag.
Both public pose and health include `estimator_age_ms`; pose validity expires
after 250 ms without an estimator update, so a dead owner task cannot leave a
frozen pose marked live. Blocking commands refresh the estimator before their
map preflight.
Use one task as the owner of `init()`/`update()`/blocking motion commands.
`current_pose()`, `sensor_health()`, and path reads may run in a separate
dashboard/logger task: they consume complete mutex-protected estimator and P8
frame snapshots rather than live structures that may be mid-update.
`navigation::stop()` is the intentional cross-task exception. It brakes
immediately and latches a cancellation that an active public turn/drive checks
on its next control iteration. Brake and physical-output calls share a short
mutex: if an output is already in progress, stop brakes after it; otherwise the
latched request prevents it. No stale motor write can occur after the brake.
A connected field-control disable produces the same fail-closed abort. A later
explicitly requested motion command clears an
older cancellation before refreshing pose and running map preflight; any
`stop()` arriving after that command linearization point remains latched and
cannot be overwritten by the later motor phase.
The go-to timeout is one end-to-end deadline shared by its initial turn and
straight leg; it is not restarted between phases.
The motion functions require an explicit successful `navigation::init()`;
background dashboard telemetry cannot silently substitute its default pose or
authorize movement. Changing/resetting the entered start pose brakes and
cancels any active public motion, then revokes initialization until
`navigation::init()` succeeds again.
Initialization first applies a true drive brake and requires both encoder sides
to move no more than 0.10 inch and P6 to change no more than 1 degree across a
250-ms settle window. Calling `init()` while rolling, being pushed, missing a
drive side, or while the IMU is unavailable therefore fails without granting
motion authority; retry only from a stationary, independently justified pose.
This 250-ms gate rejects motion during initialization; it cannot detect the
delayed P6 drift observed on 2026-08-25 and must not be described as an IMU
calibration-quality qualification.
`sensor_health()` separately reports encoder/IMU readiness, the current GPS and
AI acceptance/rejection states, P1 range/confidence, and whether an AprilTag is
visible. It also exposes the GPS sensor's raw Z-rate as a diagnostic; this is
paired with `gps_gyro_valid` so PROS's infinity error sentinel is never
mistaken for a measurement. It is not fused until its sign/scale pass a live
turn sweep. `sensor_health()` can
return raw preflight health
before `init()` without resetting P6, constructing a fused pose, or granting
motion authority. After initialization, both pose and health snapshots include
distance traveled since the
last accepted absolute position, its age, and a provisional error envelope.
A valid dead-reckoned pose does not imply that GPS is currently being
accepted. The envelope includes the caller-supplied/default start-position
bound, but excludes any placement error beyond that bound, collision, slip,
pushing, and endpoint overshoot. Treat it as a minimum engineering budget, not
a statistical guarantee. Its 2.155% scale-variation term is variation around a
P7-referenced fit and does not include a common systematic error in that fit.
Its 2-degree heading/controller term is a conservative provisional allowance,
not measured P6 accuracy; no independent angular truth was available.

The library also retains a fixed 512-point fused trail. Use `path_size()` and
`copy_path(buffer, capacity)` to read it oldest-first without heap allocation;
if the buffer is smaller than the retained trail, the newest suffix is copied.
`clear_path()` starts a new trail at the current valid pose. Points are sampled
no faster than 100 ms after meaningful motion (0.10 in or 0.25 degrees), with a
one-second stationary heartbeat. This is estimator history, not independent
ground truth: during a GPS outage it continues from encoders+P6 and carries the
same reported error envelope and absolute-fix age as the live pose.

P1 API health is separate from target-range validity, preserving the difference
between 9999-mm healthy/no-target and `PROS_ERR`. P8 diagnostics include tag ID,
horizontal and 3D range, right-bearing, elevation, and unchanged-geometry age;
they do not imply an accepted field-position correction.

Loss of either trustworthy drive-encoder side or the IMU during a blocking
command stops the drivetrain and invalidates the pose. Recovery is deliberately
not automatic: call `navigation::init()` with a newly justified absolute pose,
because movement during the sensor gap is unknowable. P8 temporal consistency
requires numerically changed corner geometry; a Smart Port poll ID alone is
not treated as a new optical exposure. An exact repeat preserves but does not
advance or reapply a correction. A fresh
failed, no-tag, unmapped, or ambiguous poll resets the confirmation sequence.
If exact corners persist despite more than 0.5 in encoder travel or 2 degrees
of P6 rotation, the observation is rejected as cached `stale_geometry`.

The current estimator propagates with GPS-referenced provisional drive encoders and the port 6
IMU. GPS on port 7 is a lower-rate absolute correction only: it must pass
reported-quality, fixed temporal-cluster, innovation, and bounded-step gates.
It corrects only while stopped (encoder speed at most 0.5 in/s and angular rate
at most 12 deg/s); moving legs remain encoder+IMU dead reckoning.
Because P7 exposes no optical-frame timestamp, an exactly unchanged GPS tuple
is also rejected as `stale_geometry` after more than 0.5 inch encoder travel or
2 degrees P6 heading change. Even while stationary, an exact tuple preserves
but does not advance the temporal chain or apply another correction. Cached
pre-move data therefore cannot establish a post-move stopped cluster.
P7 is position-only in production. A measured stopped-orientation replay found
that its low-RMS headings inside the old 5-degree gate could cumulatively bias
the P6-referenced heading by 2.96 degrees, even though each update was bounded.
P6 therefore owns heading until surveyed long-duration/temperature testing
shows a GPS correction improves it. See
`reports/sensor_campaign_2026-08-23/gps_heading_policy_replay_dashboard.png`.
Override v1.1 guarantees Field Code strips for Autonomous Coding Skills but not
for Head-to-Head fields, where they are an event-selected modification. Do not
make P7 availability a prerequisite for normal match autonomous.
The explicit `init()` pose is always the first absolute anchor, so GPS cannot
teleport the estimate on its first accepted cluster. Port 1 provides an
8-inch forward obstacle stop that fails closed on a missing P1, API error, or
malformed range;
a healthy 9999-mm/no-target return means clear, not failed. Any physical return
at or inside 8 inches stops even with low confidence, and a non-9999 value
outside the specified 20-2000 mm range is treated as a sensor fault. AI Vision port 8 reports tags and
horizontal/3D range plus bearing diagnostics. Field matching uses horizontal
range, but cannot correct position until its camera mount translation and
official goal-face transforms are measured.

That is API-level fail-closed behavior, not guaranteed obstacle perception.
The live campaign did not test controlled targets inside 8 inches, and a real
object that the sensor reports as the documented 9999-mm/no-target result is
indistinguishable from clear space. P1 therefore supplements the map and
supervision; it is not a certified collision envelope.

Before turning or driving, `go_straight_to()` also checks the whole requested
center-line segment against every mapped Goal and requires the endpoint to be
at least 14 inches inside the physical wall. Production then consumes another
one inch in the hazardous direction for the official T5 Field Element
tolerance, plus the current `position_error_envelope_in`. Normal Goal-center
clearance is similarly `16 + 1 + pose envelope` inches; a robot legally
starting close to a Goal may move outward using a bounded escape rule with a
`12 + 1 + pose envelope`-inch floor. A rejected segment returns
`kUnsafePath` without commanding the motors. The
straight-segment precheck projects the provisional 4.104%-of-travel error growth
through the full requested leg and reserves that end-of-leg envelope for the
entire segment; the initial in-place-turn check uses the current envelope.
These are deliberately
conservative center-point exclusions, not complete collision geometry: the
current robot's front/back/left/right extents from its rotation center and any
expansion envelope still need physical measurement. P1 sees only forward, so
an in-place turn is not protected on the sides or rear. Meaningful public turns
now fail closed unless the center satisfies the provisional 14-in wall inset
and 16-in mapped-Goal clearance. This applies to `turn_to()` and a go-to's
initial turn. An already-aligned robot may drive an outward escape segment
without rotating. These center checks are not a substitute for measuring the
actual swept footprint.
The static map currently includes walls and the nine Goal centers only. It does
not yet include the official Pins, Loaders, or Toggles, nor movable Blocks or
other robots. P1's single forward ray cannot replace those missing geometries,
especially during turns. Restrict calls to independently surveyed/cleared
segments until those field elements and the robot footprint are modeled.
The current Brain calculation is an opposite-edge/focal approximation; full
planar PnP and tag-plane-normal values in the campaign report are offline
reanalysis of the saved corners. A two-solution IPPE reanalysis found the same
six frames differ by at most 0.043 inch in horizontal range between solutions,
but the inferred 3D face normals differ by 41.7-68.6 degrees. Do not use the
small-tag normal to choose a Goal face without additional validation.

Current qualification status: the API, static safety checks, firmware build,
GPS fault replays, and turn controller have passed. The revised straight-line
finish-plane controller still needs a live out/return acceptance run before
this should be called competition-ready. The port 5 tracking wheel is disabled
because its mechanical coupling currently produces no useful motion.

`go_to_pose()` follows a forward continuous curve for targets up to 100 degrees
off the current heading. A target farther behind first uses the tuned,
map-checked in-place turn; all phases share the caller's one deadline. Its
map preflight reserves a six-inch corridor around the direct centerline, plus
the projected localization envelope. The live controller enforces the same
six-inch fused cross-track limit and brakes with `kDriveFailed` if the curve
leaves it. It also prevents inner-wheel reversal during the curved phase; the
dedicated tuned turn controller settles the requested final heading afterward.

For short GPS outages, P6 is currently the strongest relative heading source.
The live 120-second stationary capture varied only 0.010 degrees peak-to-peak
(0.00463-degree standard deviation); its fitted two-minute trend was
0.000442 degrees/min. Those figures do not establish long-term drift or
absolute heading accuracy because no external angle reference, temperature,
vibration, or power-cycle sweep was present. See
`reports/sensor_campaign_2026-08-23/imu_stability_dashboard.png`.

The provisional encoder model uses a 2.433055-in effective wheel and 10.624582-in
effective track. The latter is not a tape-measured chassis dimension: the live
12.0086-in angular calibration was originally expressed with a 2.75-in wheel,
so both linear terms are multiplied by the same 0.884748 straight-distance
scale to preserve their turn ratio. See
`reports/sensor_campaign_2026-08-23/drive_geometry_scale_consistency_dashboard.png`.
The scale reference was P7 GPS rather than tape/laser truth; because P7 later
showed strong orientation-dependent failures, independently survey several
straight distances before treating this as competition calibration.

`go_straight_to()` brakes immediately after entering its finish window while
the 80-ms settle timer confirms the stop. Safety, arrival, timeout, and explicit
`stop()` paths configure the drive for brake mode and call the PROS brake
operation; they do not rely on a zero-voltage `move(0)` coast command. The
translation minimum is 28/127. A live incremental test measured breakaway at
24/127 (0.430 inch in 300 ms), while 18/127 moved only 0.096 inch; 28 retains a
small battery/carpet margin and still requires live endpoint/overshoot
qualification.
2026-08-23 offline robustness
sweep is in
`reports/sensor_campaign_2026-08-23/navigation_robustness_dashboard.png`.
Within the provisional scale-plus-heading envelope, a 48-in path at power 40
had 1.94-in 95th-percentile physical endpoint error. A simulated unobserved
push/slip could create 7.14-in 95th-percentile error (8.57-in maximum) while the estimator still
claimed about 0.67 in. Keep
tagless legs short, inspect `position_error_envelope_in`, and do not interpret
an internally successful result as independent physical ground truth.

The outage envelope is distance-based, not time-based. Measured P7-referenced
scale variation plus a provisional 2-degree heading/controller allowance grows
by 4.104% of encoder-reported travel. This is not measured IMU accuracy. If the
last accepted absolute fix carries a 0.39-in base error, the 1/2/3-in total
error budgets permit about 14.9/39.2/63.6 in of subsequent dead-reckoned travel.
If the base is already 0.75 in, only 6.1 in remains before the 1-in budget is
exhausted. At 10 in/s those two 1-in budgets last only about 1.49 s and 0.61 s,
respectively; at 20 in/s they last 0.74 s and 0.30 s. See
`gps_outage_distance_budget_dashboard.png` and
`gps_outage_time_budgets.csv`; these are minimum
engineering budgets and still exclude slip, pushing, collision, start-pose
placement beyond the supplied base, common encoder-scale reference bias,
surveyed start-heading error, and long-duration/temperature IMU drift.
Operationally, keeping the provisional envelope below 1 in from a 0.39-in
base at 10 in/s means traveling no more than about 15 in, stopping, and waiting
for `gps_fix_accepted` rather than sleeping for a fixed interval. The measured-
repeat Monte Carlo's normal-return p95 is 1.47 s, but correlated/frozen optical
frames leave no finite guaranteed wait time.

Straight motion also has a raw-encoder stall watchdog. Once meaningful forward
power is commanded, failure to advance 0.10 in in 1 s stops the drivetrain and
returns `kDriveFailed`; fused absolute corrections cannot count as progress.
`turn_to()` likewise brakes throughout its 100-ms heading settle window and
resumes correction only if inertia carries it back outside tolerance.

`gps_recovery_gate_dashboard.png` replays GPS return through the production
gates. Exact tuples now preserve but do not advance confirmation. Using the
8.08% exact-repeat rate measured after resampling the live stationary run to
10 Hz, a 2.05-in disagreement begins bounded correction after 1.2 s
(12 changed observations); a 4.10-in disagreement waits 3.1 s (30 changed
observations). An 8.21-in provisional disagreement and a 7.14-in
hidden-slip disagreement remains rejected because it exceeds the 6 in recovery
gate. That is deliberate: after a large unobservable displacement, provide a
new justified `init()` pose or a separately validated absolute observation.
