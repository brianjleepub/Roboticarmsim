// Robot controller: motion queue (PTP / LIN / gripper / wait), jogging, live IK follow,
// workspace guarding and the high-level pick-and-place routine.
import {
  fk, ik, HOME, LIMITS, VMAX, DEG, GEO, toolDown, rotX, rotY, rotZ, mul, rotAxis,
  matToQuat, quatToMat, slerp, rotAngle,
} from './kinematics.js';
import { GRIPPER_MAX_GAP } from './robot.js';
import { TABLE, FLOOR_Z } from './scene.js';

const LIN_V = 0.9; // m/s at 100 %
const LIN_W = 200 * DEG; // rad/s at 100 %
const JOG_JOINT = 0.3; // fraction of VMAX when jogging at 100 %
const JOG_LIN = 0.25; // m/s
const JOG_ROT = 50 * DEG; // rad/s
const GRIP_SPEED = 0.22; // m/s finger travel

// quintic minimum-jerk profile, peak velocity 1.875x average
const ease = (s) => s * s * s * (10 + s * (-15 + 6 * s));
const PEAK = 1.875;

export class Controller {
  constructor({ robot, parts }) {
    this.robot = robot;
    this.parts = parts;
    this.q = HOME.slice();
    this.gap = GRIPPER_MAX_GAP;
    this.held = null;
    this.queue = [];
    this.active = null;
    this.override = 0.6;
    this.jog = null; // { frame: 'joint'|'world'|'tool', axis: 0..5, dir: ±1 }
    this.follow = null; // { p, R } live IK target
    this.followOk = true;
    this.estop = false;
    this.listeners = {};
    this.status = { text: 'Ready', level: 'ok' };
    this.robot.setJoints(this.q);
    this.robot.setGap(this.gap);
  }

  on(ev, fn) { (this.listeners[ev] ||= []).push(fn); }
  emit(ev, ...a) { for (const fn of this.listeners[ev] || []) fn(...a); }
  setStatus(text, level = 'info') {
    if (this.status.text === text && this.status.level === level) return;
    this.status = { text, level };
    this.emit('status', this.status);
  }

  get busy() { return !!this.active || this.queue.length > 0; }
  pose() { return fk(this.q); }

  // ---------- workspace guard ----------
  // Returns null if the configuration is clear of the table, floor and robot base.
  collision(q) {
    const f = fk(q);
    const x = [f.R[0], f.R[3], f.R[6]];
    const tip = [f.p[0] + x[0] * 0.015, f.p[1] + x[1] * 0.015, f.p[2] + x[2] * 0.015];
    // finger-tip corners spread along the finger travel axis
    const y = [f.R[1], f.R[4], f.R[7]];
    const half = this.gap / 2 + 0.012;
    const pts = [tip, f.flange, f.wrist,
      [tip[0] + y[0] * half, tip[1] + y[1] * half, tip[2] + y[2] * half],
      [tip[0] - y[0] * half, tip[1] - y[1] * half, tip[2] - y[2] * half]];
    for (const p of pts) {
      const overTable = p[0] > TABLE.x0 - 0.01 && p[0] < TABLE.x1 + 0.01 && p[1] > TABLE.y0 - 0.01 && p[1] < TABLE.y1 + 0.01;
      if (overTable && p[2] < -0.001) return 'table';
      if (p[2] < FLOOR_Z + 0.01) return 'floor';
      if (Math.hypot(p[0], p[1]) < 0.24 && p[2] < 0.3) return 'robot base';
    }
    return null;
  }

  // ---------- commands ----------
  enqueue(...cmds) {
    if (this.estop) return;
    this.queue.push(...cmds);
  }

  stop(reason) {
    this.queue = [];
    this.active = null;
    this.jog = null;
    if (reason) this.setStatus(reason, 'warn');
    this.emit('stopped');
  }

  setEstop(on) {
    this.estop = on;
    if (on) {
      this.queue = [];
      this.active = null;
      this.jog = null;
      this.follow = null;
      this.setStatus('Emergency stop. Release E-stop to continue.', 'error');
      this.emit('stopped');
    } else {
      this.setStatus('E-stop released. Ready.', 'ok');
    }
    this.emit('estop', on);
  }

  fault(text) {
    this.queue = [];
    this.active = null;
    this.setStatus(text, 'error');
    this.emit('stopped');
  }

  startCommand(c) {
    const q0 = this.q.slice();
    c.t = 0;
    if (c.type === 'ptp') {
      let qT = c.q;
      if (!qT) {
        qT = ik(c.pose.p, c.pose.R, q0);
        if (!qT) return this.fault(`${c.label || 'Target'} is out of reach.`);
      }
      c.q0 = q0;
      c.q1 = qT;
      const sp = this.override * (c.speed ?? 1);
      let T = 0.25;
      for (let i = 0; i < 6; i++) T = Math.max(T, (PEAK * Math.abs(qT[i] - q0[i])) / (VMAX[i] * sp));
      c.T = T;
    } else if (c.type === 'lin') {
      const f = fk(q0);
      c.p0 = f.p;
      c.R0 = f.R;
      c.q0r = matToQuat(f.R);
      c.q1r = matToQuat(c.pose.R);
      const d = Math.hypot(c.pose.p[0] - f.p[0], c.pose.p[1] - f.p[1], c.pose.p[2] - f.p[2]);
      const a = rotAngle(f.R, c.pose.R);
      const sp = this.override * (c.speed ?? 1);
      c.T = Math.max(0.2, (PEAK * d) / (LIN_V * sp), (PEAK * a) / (LIN_W * sp));
      if (!ik(c.pose.p, c.pose.R, q0)) return this.fault(`${c.label || 'LIN target'} is out of reach.`);
    } else if (c.type === 'grip') {
      this.robot.setJoints(this.q); // make sure the TCP transform is current for the part check
      if (c.close) {
        const part = this.held ? null : this.parts.findGraspable(this.robot.tcp, this.gap);
        c.part = part;
        c.goal = part ? part.w : 0;
      } else {
        if (this.held) {
          this.parts.release(this.held);
          this.held = null;
          this.emit('grip', false);
        }
        c.goal = GRIPPER_MAX_GAP;
      }
    }
    this.active = c;
    if (c.onStart) c.onStart();
  }

  // Advance the active command; returns true while motion continues.
  stepCommand(dt) {
    const c = this.active;
    c.t += dt;
    if (c.type === 'ptp') {
      const s = ease(Math.min(1, c.t / c.T));
      const q = c.q0.map((v, i) => v + (c.q1[i] - v) * s);
      const hit = this.collision(q);
      if (hit) return this.fault(`Protective stop: tool would hit the ${hit}.`);
      this.q = q;
      return c.t >= c.T;
    }
    if (c.type === 'lin') {
      const s = ease(Math.min(1, c.t / c.T));
      const p = c.p0.map((v, i) => v + (c.pose.p[i] - v) * s);
      const R = quatToMat(slerp(c.q0r, c.q1r, s));
      const q = ik(p, R, this.q);
      if (!q) return this.fault('LIN path leaves the workspace.');
      for (let i = 0; i < 6; i++) {
        if (Math.abs(q[i] - this.q[i]) > VMAX[i] * Math.max(dt, 1 / 60) * 1.5) {
          return this.fault('LIN path passes a singularity. Use a PTP move.');
        }
      }
      const hit = this.collision(q);
      if (hit) return this.fault(`Protective stop: tool would hit the ${hit}.`);
      this.q = q;
      return c.t >= c.T;
    }
    if (c.type === 'grip') {
      const dir = Math.sign(c.goal - this.gap);
      this.gap += dir * GRIP_SPEED * dt;
      if ((dir > 0 && this.gap >= c.goal) || (dir < 0 && this.gap <= c.goal) || dir === 0) {
        this.gap = c.goal;
        if (c.close && c.part) {
          this.parts.grasp(c.part, this.robot.tcp);
          this.held = c.part;
          this.emit('grip', true, c.part);
        }
        return c.t > 0.12; // brief dwell for the "part present" sensor
      }
      return false;
    }
    if (c.type === 'wait') return c.t >= c.T;
    if (c.type === 'call') { c.fn(); return true; }
    return true;
  }

  // ---------- jogging ----------
  jogStep(dt) {
    const { frame, axis, dir } = this.jog;
    const ov = this.override;
    if (frame === 'joint') {
      const v = VMAX[axis] * JOG_JOINT * ov * dir;
      const q = this.q.slice();
      q[axis] += v * dt;
      const [lo, hi] = LIMITS[axis];
      if (q[axis] <= lo || q[axis] >= hi) {
        q[axis] = Math.min(hi, Math.max(lo, q[axis]));
        this.setStatus(`A${axis + 1} at its software limit.`, 'warn');
      }
      const hit = this.collision(q);
      if (hit) return this.setStatus(`Blocked: tool would hit the ${hit}.`, 'warn');
      this.q = q;
      return;
    }
    // Cartesian jog in world (base) or tool frame
    const f = fk(this.q);
    let p = f.p.slice();
    let R = f.R;
    const step = (axis < 3 ? JOG_LIN : JOG_ROT) * ov * dir * dt;
    const solve = (scale) => {
      let pp = p.slice();
      let RR = R;
      if (axis < 3) {
        const e = [0, 0, 0];
        e[axis] = 1;
        const d = frame === 'tool' ? [R[axis], R[3 + axis], R[6 + axis]] : e;
        pp = [p[0] + d[0] * step * scale, p[1] + d[1] * step * scale, p[2] + d[2] * step * scale];
      } else {
        const rot = [rotZ, rotY, rotX][axis - 3]; // A about Z, B about Y, C about X
        RR = frame === 'tool' ? mul(R, rot(step * scale)) : mul(rot(step * scale), R);
      }
      return [pp, RR, ik(pp, RR, this.q)];
    };
    let [pp, RR, q] = solve(1);
    if (!q) return this.setStatus('Workspace limit reached.', 'warn');
    let ratio = 0;
    for (let i = 0; i < 6; i++) ratio = Math.max(ratio, Math.abs(q[i] - this.q[i]) / (VMAX[i] * dt));
    if (ratio > 1) {
      if (ratio > 40) return this.setStatus('Singularity: switch to axis jog to move through it.', 'warn');
      [pp, RR, q] = solve(1 / ratio);
      if (!q) return;
      this.setStatus('Near singularity: speed reduced.', 'warn');
    }
    const hit = this.collision(q);
    if (hit) return this.setStatus(`Blocked: tool would hit the ${hit}.`, 'warn');
    this.q = q;
  }

  // ---------- live IK follow ----------
  followStep(dt) {
    const { p, R } = this.follow;
    const goal = ik(p, R, this.q);
    if (!goal) {
      if (this.followOk) this.setStatus('Target out of reach.', 'warn');
      this.followOk = false;
      return false;
    }
    if (!this.followOk) this.setStatus('Tracking target.', 'ok');
    this.followOk = true;
    let ratio = 0;
    for (let i = 0; i < 6; i++) ratio = Math.max(ratio, Math.abs(goal[i] - this.q[i]) / (VMAX[i] * this.override * dt));
    const k = ratio > 1 ? 1 / ratio : 1;
    const q = this.q.map((v, i) => v + (goal[i] - v) * k);
    const hit = this.collision(q);
    if (hit) {
      this.setStatus(`Blocked: tool would hit the ${hit}.`, 'warn');
      return false;
    }
    const moved = q.some((v, i) => Math.abs(v - this.q[i]) > 1e-7);
    this.q = q;
    return moved;
  }

  // ---------- main update; returns true if anything moved ----------
  update(dt) {
    if (this.estop) return false;
    const before = this.q.slice();
    const gapBefore = this.gap;
    let guard = 0;
    while (!this.active && this.queue.length && guard++ < 20) {
      this.startCommand(this.queue.shift());
    }
    if (this.active) {
      const done = this.stepCommand(dt);
      if (done && this.active) {
        const c = this.active;
        this.active = null;
        if (c.onDone) c.onDone();
        if (!this.queue.length) this.emit('idle');
      }
    } else if (this.jog) {
      this.jogStep(dt);
    } else if (this.follow) {
      this.followStep(dt);
    }
    const moved = this.q.some((v, i) => v !== before[i]) || this.gap !== gapBefore;
    if (moved) {
      this.robot.setJoints(this.q);
      this.robot.setGap(this.gap);
    }
    return moved;
  }

  // Manual gripper toggle (works for jog and teach)
  gripper(close) {
    if (this.estop) return;
    this.enqueue({ type: 'grip', close });
  }

  // ---------- high-level pick & place ----------
  graspPose(part) {
    const pos = part.mesh.position;
    let yaw = this.parts.yawOf(part);
    if (part.kind === 'cyl') yaw = Math.atan2(pos.y, pos.x);
    // fold to the symmetric equivalent closest to the radial direction (keeps A6 calm)
    const radial = Math.atan2(pos.y, pos.x);
    const period = Math.PI / 2;
    yaw = radial + (((yaw - radial) % period) + period * 1.5) % period - period / 2;
    const top = pos.z + part.h / 2;
    return { x: pos.x, y: pos.y, zGrip: top - 0.024, yaw, top };
  }

  pickAndPlace(part, dest, { home = false } = {}) {
    if (this.estop || part.state !== 'rest') return false;
    const g = this.graspPose(part);
    const clear = Math.max(g.top, 0.05) + 0.1;
    const Rg = toolDown(g.yaw);
    const placeYaw = dest.yaw ?? g.yaw;
    const Rp = toolDown(placeYaw);
    const support = this.parts.support(dest.x, dest.y, part.r, part);
    const zPlace = support + part.h - 0.024 + 0.004;
    const placeClear = Math.max(support + part.h + 0.1, clear);
    const cmds = [];
    if (this.held) cmds.push({ type: 'grip', close: false });
    else if (this.gap < GRIPPER_MAX_GAP - 1e-4) cmds.push({ type: 'grip', close: false });
    cmds.push(
      { type: 'ptp', pose: { p: [g.x, g.y, clear], R: Rg }, label: 'Approach', onStart: () => this.setStatus(`Picking ${part.name.toLowerCase()}`, 'info') },
      { type: 'lin', pose: { p: [g.x, g.y, g.zGrip], R: Rg }, speed: 0.45, label: 'Grasp' },
      { type: 'grip', close: true },
      {
        type: 'call',
        fn: () => {
          if (this.held !== part) this.fault(`Missed the ${part.name.toLowerCase()}.`);
        },
      },
      { type: 'lin', pose: { p: [g.x, g.y, clear], R: Rg }, label: 'Retract' },
      { type: 'ptp', pose: { p: [dest.x, dest.y, placeClear], R: Rp }, label: 'Place approach', onStart: () => this.setStatus(`Placing ${part.name.toLowerCase()}`, 'info') },
      { type: 'lin', pose: { p: [dest.x, dest.y, zPlace], R: Rp }, speed: 0.4, label: 'Place' },
      { type: 'grip', close: false },
      { type: 'lin', pose: { p: [dest.x, dest.y, placeClear], R: Rp }, label: 'Depart' },
    );
    if (home) cmds.push({ type: 'ptp', q: HOME.slice(), label: 'Home' });
    this.enqueue(...cmds);
    return true;
  }

  goHome() {
    this.enqueue({ type: 'ptp', q: HOME.slice(), label: 'Home', onStart: () => this.setStatus('Moving home', 'info'), onDone: () => this.setStatus('Home position', 'ok') });
  }
}

export { HOME, GEO, toolDown, rotAxis };
