# RX-6 Robot Cell

A mobile-first 3D simulator of a 6-axis industrial robot arm. You can jog it axis by axis or in Cartesian space, drive it by dragging an inverse-kinematics target, pick and place parts with a parallel gripper, and teach and replay programs. It is built for a phone held in portrait (tuned for an iPhone 17 Pro Max), and it also works in landscape and on desktop.

**Live:** https://claude.ai/artifact/6ehqDvQJhHBG3ymPusD3Wz
(GitHub Pages also works once Pages is enabled for the repo; see [Deploying](#deploying).)

## Using it

| Where | What |
|---|---|
| 3D view | One finger orbits, two fingers pinch-zoom and pan. **Fit / Top / Tool** set the camera. **Tool** makes the camera follow the gripper. |
| **E-STOP** (top right) | Halts all motion and clears the queue. Tap **RESET** to release it. The stack light on the controller cabinet shows the state: amber is manual, green is running, red is E-stop. |
| **Speed** slider | Override from 5 % to 100 %. It scales jog, PTP and LIN speeds. |
| **Jog** tab | Hold **−** / **+**. *Axes* moves A1–A6 directly. *World* moves X/Y/Z and A/B/C (rotation about Z/Y/X) in the base frame. *Tool* does the same in the gripper frame. The bars show where each axis sits within its software limits. |
| **Target** tab | Drag the orange ball, either *across the table* or *up / down*. With *Follow live* the arm solves IK every frame and chases the ball within its joint speed limits. With *Move on command* you place the ball and then press **Move PTP** (joint interpolated) or **Move LIN** (straight line). Tapping the table drops the target there. The ball turns red when the pose is out of reach. |
| **Pick** tab | Tap a part, then tap a tray or a free spot on the table. The robot runs approach → LIN down → grip → retract → PTP → LIN place → release → depart → home. **Sort all by colour** clears the table into the matching trays. **Reset parts** restores the starting layout. |
| **Program** tab | Teach pendant. Move the arm, then record **+ PTP** / **+ LIN** points, **+ Close** / **+ Open** for the gripper and **+ Wait**. **Run** replays the program; **Loop** repeats it. An example path is loaded so you can press Run straight away. |

The HUD shows the TCP position in millimetres and the A/B/C orientation in degrees (KUKA-style Z-Y′-X″ Euler angles).

## How it works

```
src/
  kinematics.js   FK + closed-form IK (spherical wrist), limits, rotation helpers. Pure math, unit-tested.
  robot.js        three.js model. Each link group matches the kinematic chain exactly.
  scene.js        renderer, lighting, procedural textures, cell (riser, table, trays, fence, cabinet, stack light)
  parts.js        workpieces, drop physics, grasp detection, tray slots
  motion.js       controller: PTP/LIN/grip/wait queue, jogging, live IK follow, collision guard, pick & place
  main.js         camera, touch interaction, pendant UI wiring, render loop
  page.html       markup and CSS
scripts/build.mjs esbuild bundle → dist/index.html (one self-contained file) + dist/artifact.html/app.js
test/             node:test checks of the kinematics
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
- **Workspace guard:** the fingertips, flange and wrist are checked against the table, floor and robot base on every step. Jogging is blocked and programs stop with a protective-stop message.

### Rendering and performance

- three.js r180, bundled. There are no runtime CDN or asset downloads apart from the Google Fonts stylesheet, which has system fallbacks.
- Lighting is ACES tone mapping, RoomEnvironment PMREM reflections, one 2048² soft shadow map tuned tight around the cell, plus hemisphere and rim lights.
- All textures (concrete floor, hazard tape, fence mesh, model badge) are drawn procedurally on canvas, so the app is about 600 KB.
- **Render on demand:** frames are only drawn when something moves (robot, parts, camera), which saves battery.
- Pixel ratio is capped at 2 on a 3× display. If frames average over 22 ms, resolution steps down automatically.
- The camera view offset keeps the robot centred in the part of the screen the pendant does not cover.

## Decisions and assumptions

Choices made without asking, as requested:

1. **Generic robot, not a brand.** The arm is styled like a modern industrial arm (orange paint, dark castings, cable dress) but is a fictional "RX-6" to avoid using anyone's trademark.
2. **Analytic IK over a numeric solver.** It is exact, fast enough to run every frame on a phone, and the nearest-solution selection gives controller-like behaviour. A numeric solver can drift or get stuck near singularities.
3. **Lightweight custom physics instead of a physics engine.** Released parts fall under gravity, settle upright keeping their heading, and stack on surfaces, trays and other parts. This avoids roughly 1 MB of WASM, which was the right trade for mobile load time. Parts are not pushed around by the arm.
4. **Grasping is geometric.** When the jaws close, any part sitting between the fingers is detected, the fingers stop at the part's width, and the part is attached to the TCP. A missed grasp stops the pick sequence with an error.
5. **Collision checking is simplified.** It covers tool-versus-environment (table, floor, base) but not link-versus-link self-collision. Joint limits are set so the arm does not fold through itself in normal use.
6. **Manual mode feel.** Jog speed is a fraction of the maximum axis speeds, like T1 mode. Programs and pick-and-place run at the override speed ("Auto").
7. **One dark theme.** It reads like a teach pendant and stays legible over the bright 3D scene. It does not follow the system light/dark setting.
8. **Not saved.** Taught programs live in memory only and are not persisted between visits.

## Development

```bash
npm install
npm test          # kinematics tests
npm run build     # → dist/index.html (open directly in a browser)
npm run dev       # rebuild on change
```

## Deploying

- **claude.ai Artifact** (the live link above): publish `dist/artifact.html` with `dist/app.js` alongside it.
- **GitHub Pages:** `.github/workflows/pages.yml` builds and deploys `dist/` on every push to `main`. Enable it once under *Settings → Pages → Source: GitHub Actions*.
- **Any static host:** `dist/index.html` is a single self-contained file.
