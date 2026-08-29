# VEX Path 1 Agent Implementation Spec

This file is the handoff spec for an agent that needs to build the **improved Path 1 autonomous simulator end to end**.

## 1) Goal of the deliverable

Build a **Path 1-only** VEX autonomous simulator with a clearer and more implementation-oriented visualization than the earlier dual-path simulator.

The new simulator should:
- focus only on **Path 1**
- show the path on the real field map
- animate the robot’s position and heading over time
- expose the key coordinates and actions clearly
- show the mechanism intent for each phase (intake / lift / drop / reverse claw)
- stop after the robot turns to face the next stack at `(24, -24)`

## 2) Source images available

Use the existing field map plus the user-provided reference sketches.

### Required visual references
- Original field map: use the unchanged map image already used in the prior simulator.
- Real robot start photo: confirms qualitative starting orientation and that the robot starts near the wall / toggle.
- Annotated movement-1 sketch: shows the first move from the starting zone toward the first scoring goal.
- Annotated movement-2 sketch: shows the next move from the first goal toward the stack at `(24, 0)` and then to center `(0, 0)`.

## 3) Coordinate system (must stay consistent)

The simulator should continue using the same coordinate system as before:
- field is centered at `(0, 0)`
- units are **inches**
- **first coordinate**: positive goes **toward the top of the field image**
- **second coordinate**: positive goes **toward the left of the field image**

So the renderer transform is:
- `worldX = -coordY`
- `worldY = coordX`

## 4) Important route objects

Treat these as ground-truth object coordinates for Path 1:

- First scoring goal (goal 3): `(48, -24)`
- Red/blue stack pickup: `(24, 0)`
- Center scoring location: `(0, 0)`
- Final facing target (yellow pin+cup stack): `(24, -24)`

## 5) Ambiguity that must be handled explicitly

The user described the **starting pose** partly with edge distances:
- closer to the red goal than before
- `24 in` from the black goal to the robot edge
- `7 in` from the red goal to the robot edge
- `7 in` away from the toggle

That is **not enough by itself** to uniquely solve the exact robot-center coordinate unless the code also commits to:
- robot footprint dimensions
- which robot edge is being measured
- which exact field objects define those “black goal” and “red goal” distances

### Therefore:
The implementation should use an **editable approximate start pose constant** for the visualization.

Recommended default for the improved visualizer:
```ts
const START_POSE = {
  coord: [63, -8],   // approximate center in field coordinates
  headingDeg: 0      // facing toward +x (toward the top wall / toggle)
};
```

The key point is not that this number is perfectly solved. The key point is that the code should make the start pose **easy to tweak later**.

## 6) Path 1 route interpretation

There is one wording inconsistency in the user description: the user says “go the red goal” once.

However, the annotated field screenshots and the earlier route context strongly suggest that the **first post-toggle scoring action should still be on the upper-right goal 3 at `(48, -24)`**.

So the agent should implement the improved simulator using this interpretation:
- **first goal after the toggle = goal 3 at `(48, -24)`**

## 7) Detailed Path 1 phase breakdown

Implement the route as the following explicit phases.

| Step | Phase | Type | Coordinates | Drive behavior | Mechanism behavior | Intended result |
|---|---|---|---|---|---|---|
| 1 | Ram toggle | move | approx start → toggle hit point | drive forward `9 in` at full speed | no intake action | force the toggle hit |
| 2 | Back out from toggle | move | toggle hit point → original start lane | reverse `9 in` at about `80/127` speed | no intake action | return to starting lane after toggle flip |
| 3 | Approach first goal | move / curved | start lane → `(48, -24)` | smooth boomerang-like curved move | still carrying preload | reach goal 3 |
| 4 | Score on first goal | dwell | at `(48, -24)` | hold | place / drop the preload | preload scored on goal 3 |
| 5 | Back off from goal 3 | move | `(48, -24)` → small backoff point | short reverse | disengage from goal | clear room to turn |
| 6 | Turn toward stack A | rotate | at backoff point | turn in place | line up with `(24, 0)` | face the stack |
| 7 | Drive to stack A | move | backoff point → `(24, 0)` | drive in and slow down near the stack | intake with claw | acquire the red/blue stack |
| 8 | Carry stack to center | move | `(24, 0)` → `(0, 0)` | steady drive | raise lift to stage 2 while moving | arrive centered and ready to place |
| 9 | Place stack at center | dwell | at `(0, 0)` | hold | set stack down | stack placed at center |
| 10 | Reverse claw and back out | move | `(0, 0)` → `(24, 0)` | reverse away | reverse-spin claw to detach cleanly | robot detaches completely |
| 11 | Final turn | rotate | at `(24, 0)` | turn in place | face `(24, -24)` | end Path 1 staged for the next pickup |

## 8) Suggested geometry constants for the visualizer

These are visualization constants, not physics truth:

```ts
const TARGETS = {
  toggle: [72, -8],
  goal3: [48, -24],
  stackA: [24, 0],
  center: [0, 0],
  finalFace: [24, -24],
  backoff: [52, -20]
};
```

Notes:
- `toggle` is a convenient hit point for the start-zone ram.
- `backoff` is an implementation helper point, not a field object.
- The boomerang move should be rendered as a **small curve**, not a perfectly straight line.

## 9) How the simulator should visualize the path

The improved simulator should show:
- the field image as the ground plane / main board
- a robot rectangle with a clear **front direction arrow**
- a visible payload marker on the robot only when the robot is carrying something
- persistent route path (dashed or glowing)
- step markers for key score / pickup / end locations
- a phase list or timeline
- a details panel describing the active phase

## 10) Strong recommendation for code structure

Use a **data-driven phase list** instead of hardcoding the animation behavior directly.

Example shape:

```ts
type Phase = {
  id: string;
  title: string;
  kind: 'move' | 'rotate' | 'dwell';
  duration: number;
  payload: boolean;
  speedLabel: string;
  mechanism: string;
  outcome: string;

  path?: 'line' | 'quad';
  from?: [number, number];
  to?: [number, number];
  ctrl?: [number, number];

  at?: [number, number];
  fromHeadingTarget?: [number, number];
  toHeadingTarget?: [number, number];
};
```

This makes the route easy to tweak without rewriting animation logic.

## 11) Recommended animation behavior

### Move phases
- For straight moves: linear interpolation with easing.
- For the boomerang move: quadratic Bézier curve using `from`, `ctrl`, and `to`.
- Heading should follow the direction of travel.

### Rotate phases
- Robot stays at one coordinate.
- Heading interpolates between the from-target direction and the to-target direction.

### Dwell phases
- Robot stays fixed.
- Show the scoring or placement action in the UI / mechanism text.

## 12) What the UI should say during each key action

Use implementation-friendly wording like this:
- Ram toggle
- Back out from toggle
- Boomerang into Goal 3
- Score on Goal 3
- Back off from Goal 3
- Rotate toward Stack A
- Drive to Stack A and intake
- Carry stack to center while raising to stage 2
- Place stack at center
- Reverse claw and back out
- Final turn toward diagonal stack

## 13) Acceptance checklist

The simulator is done when all of the following are true:
- [ ] it only shows **Path 1**
- [ ] the route clearly goes through `(48, -24)`, `(24, 0)`, `(0, 0)`, back to `(24, 0)`
- [ ] the final heading clearly faces `(24, -24)`
- [ ] the robot visibly carries a preload before the first score
- [ ] the robot visibly carries a stack from `(24, 0)` to `(0, 0)`
- [ ] the UI communicates lift/claw actions, not just drive motion
- [ ] the start pose is stored as an editable constant
- [ ] the first post-toggle goal is implemented as goal 3 at `(48, -24)`
- [ ] the route stops after the final turn and does **not** proceed to `(24, -24)` yet

## 14) Deliverable names

Recommended output files:
- `vex_path1_detailed_simulator.html`
- `VEX_PATH1_AGENT_SPEC.md`
- optional bundle zip containing both files

## 15) Short implementation summary for the agent

If you need a condensed instruction block:

> Build a new Three.js HTML simulator focused only on Path 1. Keep the same field image and same coordinate convention (`+x` = top of image, `+y` = left of image). Use a data-driven list of phases. Show robot pose, heading, payload state, a dashed route, and a phase timeline. Implement the route as: start near the top wall, ram the toggle, back out, curve into goal 3 at `(48, -24)`, score, back off, rotate toward the stack at `(24, 0)`, intake it, drive to center `(0, 0)` while raising to stage 2, place the stack, reverse the claw while backing out to `(24, 0)`, then turn to face `(24, -24)` and stop.
