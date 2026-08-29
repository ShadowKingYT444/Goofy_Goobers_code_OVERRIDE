# VEX Path 2 Agent Implementation Spec

This file is the handoff spec for an agent that needs to build the **improved Path 2 autonomous simulator end to end**.

## 1) Goal of the deliverable

Build a **Path 2-only** autonomous simulator with clearer route visualization and clearer mechanism-state communication than the earlier combined simulator.

The simulator should:
- focus only on **Path 2**
- visualize the route on the real field map
- animate the robot pose over time
- show the current heading, payload state, and lift state
- expose the exact sequence for the two stack pickups and scores
- end at the **blue matchloading station**

## 2) Strong recommendation: do not use the earlier WebGL approach for this version

The previous Path 1 simulator produced a black screen for the user.

So for this deliverable, prefer a **robust 2D HTML + SVG overlay** on top of the field image rather than a WebGL-heavy implementation. The route logic matters more than 3D rendering.

## 3) Coordinate system

Keep the same coordinate convention as before:
- field center is `(0, 0)`
- units are **inches**
- first coordinate: positive goes **toward the top of the field image**
- second coordinate: positive goes **toward the left of the field image**

So the map projection is:
- `svgX = 72 - coordY`
- `svgY = 72 - coordX`

assuming the SVG viewBox is `0 0 144 144`.

## 4) Important ambiguity to capture explicitly

The user described the starting position as roughly **`(0, 63)`**.

However, that conflicts with:
- the earlier coordinate convention
- the right-side placement in the uploaded sketch
- the previously corrected Path 2 geometry

### Recommended assumption
Use:
```ts
const START_POSE = {
  coord: [0, -63],
  // heading chosen to face the nearby blue goal 2 lane
};
```

Important: make this a clearly editable constant and note in the UI/spec that the sign was corrected as an interpretation choice.

## 5) Ground-truth Path 2 objects

Use these route anchors:
- Blue goal 2: `(24, -48)`
- First stack pickup: `(24, -24)`
- Second stack pickup: `(48, -48)`
- Blue matchloading station: `(66, -66)`
- Toggle contact point: approximately `(0, -72)`

## 6) Path 2 route interpretation

Implement the route as:
1. Start near the right side.
2. Reverse into the toggle once.
3. Return to the start lane.
4. Drive to blue goal 2 and drop the preload.
5. Back off.
6. Reorient to the first stack at `(24, -24)`.
7. Intake that stack carefully.
8. Raise the lift to **level 1** while traveling back to blue goal 2.
9. Score on blue goal 2.
10. Back off again.
11. Reorient to the second stack at `(48, -48)`.
12. Intake it carefully.
13. Raise the lift to **level 2** asynchronously while traveling back to blue goal 2.
14. Score on blue goal 2.
15. Back off and rotate so the rear/claw is oriented toward the blue matchloading station.
16. Lower the lift while traveling to the matchloader and end there.

## 7) Important interpretation detail about final orientation

The user said two things:
- reorient the **back/claw** of the robot to the matchloading station
- end in an **exactly 180° flipped** orientation relative to the start

These are not automatically the same unless the heading convention and travel direction are modeled carefully.

### Recommended resolution
Represent the robot’s front arrow as the **main intake/front**.
Then model the final travel as the robot **backing into** the matchloading station while the front arrow remains 180° flipped from the start orientation.

That satisfies:
- rear/claw faces the station
- front/main intake faces away from it
- end orientation is 180° from the start

## 8) Suggested explicit phase breakdown

| Step | Phase | Type | Coordinates | Drive behavior | Mechanism behavior | Result |
|---|---|---|---|---|---|---|
| 1 | Reverse into toggle | move | `(0, -63)` → `(0, -72)` | reverse 9 in at full speed | no intake | toggle flips |
| 2 | Return to start lane | move | `(0, -72)` → `(0, -63)` | forward 9 in moderate speed | no intake | reset to start lane |
| 3 | Drive to goal 2 | move / curved | start → `(24, -48)` | short smooth curve | carry preload | arrive to score |
| 4 | Score preload | dwell | at `(24, -48)` | hold | drop preload | preload scored |
| 5 | Back off goal 2 | move | goal 2 → backoff point | short disengage | none | create turning space |
| 6 | Rotate to stack A | rotate | at backoff | turn in place | face `(24, -24)` | line up for intake |
| 7 | Collect stack A | move | backoff → `(24, -24)` | slow down near target | intake stack | first stack secured |
| 8 | Carry stack A to goal 2 | move | `(24, -24)` → `(24, -48)` | steady return | raise lift to level 1 asynchronously | ready to score at level 1 |
| 9 | Score stack A | dwell | at goal 2 | hold | score stack | first extra stack scored |
| 10 | Back off again | move | goal 2 → backoff | short disengage | none | create space for second run |
| 11 | Rotate to stack B | rotate | at backoff | turn in place | face `(48, -48)` | line up for second intake |
| 12 | Collect stack B | move | backoff → `(48, -48)` | controlled approach | intake stack | second stack secured |
| 13 | Carry stack B to goal 2 | move | `(48, -48)` → `(24, -48)` | steady return | raise lift to level 2 asynchronously | ready to score at level 2 |
| 14 | Score stack B | dwell | at goal 2 | hold | score stack | second extra stack scored |
| 15 | Flip orientation for finish | rotate | near goal 2 backoff | rotate in place | set rear/claw toward matchloader | finish orientation established |
| 16 | Travel to matchloader | move | backoff → `(66, -66)` | travel while lowering lift | lower lift; back into station | autonomous ends |

## 9) Suggested constants

```ts
const TARGETS = {
  toggle: [0, -72],
  goal2: [24, -48],
  stack1: [24, -24],
  stack2: [48, -48],
  backoff1: [20, -56],
  backoff2: [20, -56],
  backoff3: [18, -56],
  matchloader: [66, -66]
};
```

These are visualization helper constants, not physical truth.

## 10) Recommended simulator data structure

Use a data-driven phase array.

Example shape:

```ts
type Phase = {
  id: string;
  title: string;
  kind: 'move' | 'rotate' | 'dwell';
  duration: number;
  speedLabel: string;
  mechanism: string;
  outcome: string;
  payload: 'none' | 'preload' | 'stack A' | 'stack B';
  lift: '0' | '1' | '2' | 'raising to 1' | 'raising to 2' | 'lowering';

  // move
  path?: 'line' | 'quad';
  from?: [number, number];
  to?: [number, number];
  ctrl?: [number, number];
  headingMode?: 'fixed' | 'follow';
  headingDeg?: number;

  // rotate / dwell
  at?: [number, number];
  fromHeadingDeg?: number;
  toHeadingDeg?: number;
};
```

## 11) Visual requirements

The simulator should show:
- real field image in the background
- persistent path overlay
- visible robot body
- clear front arrow / heading indicator
- visible payload disk only when carrying something
- visible lift state indicator
- active phase panel
- scrubber / timeline

## 12) Acceptance checklist

The deliverable is done when:
- [ ] it is **Path 2 only**
- [ ] it starts from an editable right-side start pose
- [ ] it scores the preload on **blue goal 2 at `(24, -48)`**
- [ ] it collects the first stack at `(24, -24)` and scores it on goal 2
- [ ] it collects the second stack at `(48, -48)` and scores it on goal 2
- [ ] it visibly shows lift state changes: level 1, level 2, lowering
- [ ] it ends at the **blue matchloading station `(66, -66)`**
- [ ] the final pose is represented as a 180° flip relative to the start orientation
- [ ] the implementation explicitly documents the sign ambiguity in the user’s stated start coordinate

## 13) Deliverable names

Recommended output names:
- `vex_path2_detailed_simulator.html`
- `VEX_PATH2_AGENT_SPEC.md`
- optional zip bundle containing both

## 14) Short condensed build brief for the agent

> Build a Path 2-only autonomous simulator using robust HTML + SVG over the field image rather than WebGL. Preserve the existing coordinate convention. Use an editable start pose near the right side, preferably `(0, -63)` as the sign-corrected interpretation of the user sketch. Animate the sequence: reverse into the toggle, come back out, score the preload on blue goal 2 `(24, -48)`, collect the first stack at `(24, -24)` and score it while raising to lift level 1, collect the second stack at `(48, -48)` and score it while raising to lift level 2, then rotate for the finish and travel to the blue matchloading station `(66, -66)` while lowering the lift. Show heading, payload, lift state, and a phase-by-phase explanation.
