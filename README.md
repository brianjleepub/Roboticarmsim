# RX-6 Robot Cell

A mobile-first 3D simulator of a two-robot work cell. Two identical 6-axis industrial arms face each other across one table. Robot 1 feeds parts into a shared transfer tray, robot 2 takes them out and sets them on a belt conveyor, and the conveyor drops them into a bin. You can jog either arm axis by axis or in Cartesian space, drive it by dragging an inverse-kinematics target, pick and place parts with its parallel gripper, and teach and replay a program for each arm. It is built for a phone held in portrait (tuned for an iPhone 17 Pro Max), and it also works in landscape and on desktop.

**Live:** https://brianjleepub.github.io/Roboticarmsim/ (GitHub Pages, deployed from `main`)
**Mirror:** https://claude.ai/artifact/6ehqDvQJhHBG3ymPusD3Wz (claude.ai Artifact, private to the owner unless shared)

## Using it

| Where | What |
|---|---|
| 3D view | One finger orbits, two fingers pinch-zoom and pan. **Fit / Top / Tool** set the camera. **Tool** makes the camera follow the selected arm's gripper. On the Jog and Program tabs, tapping an arm selects it. |
| **R1 / R2** | Chooses which arm the Jog, Target and Program tabs and the HUD act on. An orange ring on the floor marks the selected arm. |
| **E-STOP** (top right) | Halts both arms and the line and clears their queues. Tap **RESET** to release it. The stack light on the controller cabinet shows the cell state: amber is manual, green is running, red is E-stop. |
| **Speed** slider | Override from 5 % to 100 % for both arms. It scales jog, PTP and LIN speeds. |
| **Jog** tab | Hold **−** / **+**. *Axes* moves A1–A6 directly. *World* moves X/Y/Z and A/B/C (rotation about Z/Y/X) in the shared cell frame, so +X means the same direction for both arms. *Tool* does the same in the gripper frame. The bars show where each axis sits within its software limits. |
| **Target** tab | Drag the orange ball, either *across the table* or *up / down*. With *Follow live* the arm solves IK every frame and chases the ball within its joint speed limits. With *Move on command* you place the ball and then press **Move PTP** (joint interpolated) or **Move LIN** (straight line). Tapping the table drops the target there. The ball turns red when the pose is out of reach. |
| **Pick** tab | **Run line** starts the two-robot transfer: robot 1 moves each table part into the green transfer tray, robot 2 moves it from the tray onto the conveyor, and the belt carries it off its end into the bin. The counters show how many parts are on the table, in the transfer tray, on the belt and in the bin. You can also tap a part, then tap a tray, the conveyor or a free spot on the table. If one arm can reach both ends it does the whole move. If not, the arm that can pick it hands it to the other through the transfer tray. **Sort by colour** has robot 1 clear its side of the table into the matching trays. **Reset parts** restores the starting layout. |
| **Program** tab | Teach pendant for the selected arm, with one program per arm. Move the arm, then record **+ PTP** / **+ LIN** points, **+ Close** / **+ Open** for the gripper and **+ Wait**. **Run** replays the program; **Loop** repeats it. Each arm has an example path loaded so you can press Run straight away. |

The HUD shows the selected arm's TCP position in cell coordinates (millimetres, with the origin on robot 1's mounting plate) and its A/B/C orientation in degrees (KUKA-style Z-Y′-X″ Euler angles).

### The cell

| Item | Position (cell frame, mm) | Notes |
|---|---|---|
| Robot 1 | base at 0, 0 | parts start on its side of the table |
| Robot 2 | base at 1700, 0, turned 180° | faces robot 1 |
| Transfer tray | 880, 0 | shared and interlocked, 2 × 2 slots, reachable by both arms |
| Colour trays | red, yellow and blue on robot 1's side | used by **Sort by colour** |
| Conveyor | x 1200, from y −300 to −1200, belt top 70 mm above the table | 140 mm/s; robot 2 loads it at 1200, −400 |
| Bin | under the conveyor's discharge end | parts carry the belt speed as they fall in |

## How it works

```
src/
  kinematics.js   FK + closed-form IK (spherical wrist), limits, rotation helpers. Pure math, unit-tested.
  robot.js        three.js model. Each link group matches the kinematic chain exactly.
  scene.js        renderer, lighting, procedural textures, cell layout (risers, table, trays, conveyor, bin, fence)
  parts.js        workpieces, drop physics, conveyor transport, grasp detection, tray slots
  motion.js       one controller per arm: PTP/LIN/grip/wait/zone queue, jogging, live IK follow,
                  collision guard, pick & place, cell <-> base frame conversion
  line.js         two-robot transfer line: feeder, unloader, conveyor, finish detection
  main.js         camera, touch interaction, pendant UI wiring, hand-over planning, render loop
  page.html       markup and CSS
scripts/build.mjs esbuild bundle → dist/index.html (one self-contained file) + dist/artifact.html/app.js
test/             node:test checks of the kinematics, frame conversion, cell reach and the interlock
```

### Kinematics

- The geometry is a typical 10 kg / 1.1 m class arm with a shoulder offset (A1 = 150 mm), a 560 mm upper arm, a small elbow offset (35 mm), a 515 mm forearm, and a spherical wrist with 80 mm to the flange. The gripper TCP is 165 mm past the flange.
- IK is **analytic**. The wrist centre gives A1–A3 (front/back × elbow up/down), and A4–A6 come from the X-Y-X wrist decomposition (flip/no flip). That is up to 8 solutions. Out-of-limit solutions are dropped, angles are unwrapped by 2π where an axis allows more than ±180°, and the solution closest to the current joints wins. That keeps jogging and LIN moves continuous.
- The wrist singularity (A5 = 0) is handled by holding A4 and putting the remaining rotation on A6.
- Tests check that IK reproduces FK for 2000 random in-limit poses and that the nearest branch is the original configuration.

### Motion

- **PTP** is synchronised joint interpolation with a minimum-jerk (quintic) profile. The slowest axis at its limit speed sets the duration.
- **LIN** interpolates position linearly and orientation by slerp, solving IK each frame. If any axis would exceed its speed limit, the move aborts with "passes a singularity", which is what a real controller reports.
- **Cartesian jog** scales its step down near singularities (with a "speed reduced" warning) and stops if it would need a configuration flip.
- **Workspace guard:** the fingertips, flange and wrist are checked against the table, floor, conveyor, bin and both robot bases on every step. Jogging is blocked and programs stop with a protective-stop message.
- **Two arms, one frame:** each controller solves kinematics in its own base frame and converts at its edges, so poses, part positions, jog directions and the HUD all use the shared cell frame.
- **Transfer-tray interlock:** the tray is a zone only one arm may work in. An arm acquires it before moving over the tray and releases it only after it has moved clear (back to home after placing, or on to its place position after picking). The other arm waits, and the status bar says what it is waiting for. A stop, fault or E-stop releases the zone. In the headless line tests the two tools were never near the tray at the same time.
- **Line logic:** robot 1 only feeds a part when a transfer slot is free, counting slots promised to moves already under way. Robot 2 only unloads when the conveyor infeed is clear. The belt runs while the line runs and keeps going until it is empty. A fault on either arm halts the whole line with the reason shown.

### Rendering and performance

- three.js r180, bundled. There are no runtime CDN or asset downloads apart from the Google Fonts stylesheet, which has system fallbacks.
- Lighting is ACES tone mapping, RoomEnvironment PMREM reflections, one 2048² soft shadow map tuned tight around the cell, plus hemisphere and rim lights.
- All textures (concrete floor, hazard tape, fence mesh, conveyor belt, model badge) are drawn procedurally on canvas, so the app is about 630 KB. The belt moves by scrolling its texture.
- **Render on demand:** frames are only drawn when something moves (arms, parts, belt, camera), which saves battery.
- Pixel ratio is capped at 2 on a 3× display. If frames average over 22 ms, resolution steps down automatically.
- The camera view offset keeps the robot centred in the part of the screen the pendant does not cover.

## Decisions and assumptions

Choices made without asking, as requested:

1. **Generic robot, not a brand.** The arm is styled like a modern industrial arm (orange paint, dark castings, cable dress) but is a fictional "RX-6" to avoid using anyone's trademark.
2. **Analytic IK over a numeric solver.** It is exact, fast enough to run every frame on a phone, and the nearest-solution selection gives controller-like behaviour. A numeric solver can drift or get stuck near singularities.
3. **Lightweight custom physics instead of a physics engine.** Released parts fall under gravity, settle upright keeping their heading, and stack on surfaces, trays and other parts. This avoids roughly 1 MB of WASM, which was the right trade for mobile load time. Parts are not pushed around by the arm.
4. **Grasping is geometric.** When the jaws close, any part sitting between the fingers is detected, the fingers stop at the part's width, and the part is attached to the TCP. A missed grasp stops the pick sequence with an error.
5. **Collision checking is simplified.** It covers tool-versus-environment (table, floor, conveyor, bin, both bases) but not link-versus-link collision, within one arm or between the two arms. Arm-to-arm safety comes from the transfer-tray interlock, which is how real cells share a hand-over station. Jogging both arms into each other by hand is not prevented.
6. **Manual mode feel.** Jog speed is a fraction of the maximum axis speeds, like T1 mode. Programs and pick-and-place run at the override speed ("Auto").
7. **One dark theme.** It reads like a teach pendant and stays legible over the bright 3D scene. It does not follow the system light/dark setting.
8. **Not saved.** Taught programs live in memory only and are not persisted between visits.
9. **Hand-over through a tray, not gripper to gripper.** The arms pass parts through a shared tray rather than directly between grippers. It is the common industrial pattern and needs no synchronised motion.
10. **One feeder, one unloader.** On the line, robot 1 always feeds and robot 2 always unloads. Manual tap moves pick whichever arm can do the job, preferring the selected one.
11. **The conveyor ends in a bin.** Parts that reach the bin stay there until **Reset parts**.

## Development

```bash
npm install
npm test          # kinematics, frame conversion, reach and interlock tests
npm run build     # → dist/index.html (open directly in a browser)
npm run dev       # rebuild on change
```

## Deploying

- **claude.ai Artifact** (the live link above): publish `dist/artifact.html` with `dist/app.js` alongside it.
- **GitHub Pages:** `.github/workflows/pages.yml` runs the tests, builds and deploys `dist/` on every push to `main` (Pages source is set to *GitHub Actions*). It can also be started by hand from the Actions tab (*Run workflow*).
- **Any static host:** `dist/index.html` is a single self-contained file.
