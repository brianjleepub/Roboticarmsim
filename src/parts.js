// Workpieces: creation, simple rigid-body drop physics, conveyor transport, grasp detection
// and tray slots. All positions are in the cell frame (Z-up, robot 1 base at the origin).
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { TABLE, FLOOR_Z, TRAY_DEFS, TRAY_FLOOR, CONVEYOR, BIN } from './scene.js';

const COLORS = { red: 0xd23b33, yellow: 0xf0b21b, blue: 0x2f6fd6 };

// Example layout, deterministic so "Reset parts" always gives the same scene.
// All parts start on robot 1's side of the table, out of robot 2's reach.
const START = [
  { color: 'red', kind: 'cube', x: 0.5, y: -0.27, yaw: 0.35 },
  { color: 'blue', kind: 'cyl', x: 0.64, y: -0.17, yaw: 0 },
  { color: 'yellow', kind: 'cube', x: 0.52, y: 0.24, yaw: -0.5 },
  { color: 'red', kind: 'cyl', x: 0.65, y: 0.19, yaw: 0 },
  { color: 'blue', kind: 'cube', x: 0.45, y: -0.02, yaw: 0.9 },
  { color: 'yellow', kind: 'cyl', x: 0.62, y: 0.02, yaw: 0 },
];

const insideRect = (x, y, cx, cy, hx, hy) => Math.abs(x - cx) < hx && Math.abs(y - cy) < hy;
const onBeltXY = (x, y) => Math.abs(x - CONVEYOR.x) < CONVEYOR.width / 2 && y <= CONVEYOR.y0 && y >= CONVEYOR.y1;

export class Parts {
  constructor(cell) {
    this.cell = cell;
    this.list = [];
    this.tmpM = new THREE.Matrix4();
    this.tmpV = new THREE.Vector3();
    START.forEach((s, i) => this.list.push(this.make(s, i)));
    this.reset();
  }

  make(s, i) {
    const w = 0.05;
    const h = s.kind === 'cube' ? 0.05 : 0.065;
    const geo = s.kind === 'cube'
      ? new RoundedBoxGeometry(w, w, h, 3, 0.004)
      : new THREE.CylinderGeometry(w / 2, w / 2, h, 40, 1).rotateX(Math.PI / 2);
    const mat = new THREE.MeshPhysicalMaterial({
      color: COLORS[s.color], roughness: 0.32, metalness: 0, clearcoat: 0.3, clearcoatRoughness: 0.4,
      emissive: COLORS[s.color], emissiveIntensity: 0,
    });
    const mesh = new THREE.Mesh(geo, mat);
    mesh.castShadow = mesh.receiveShadow = true;
    const name = `${s.color[0].toUpperCase()}${s.color.slice(1)} ${s.kind === 'cube' ? 'block' : 'cylinder'}`;
    const part = {
      id: `p${i}`, name, color: s.color, kind: s.kind, w, h, r: w / 2, start: s, mesh,
      state: 'rest', vx: 0, vy: 0, vz: 0, settleQ: new THREE.Quaternion(),
    };
    mesh.userData.partId = part.id;
    this.cell.add(mesh);
    return part;
  }

  byId(id) { return this.list.find((p) => p.id === id); }

  reset() {
    for (const p of this.list) {
      if (p.mesh.parent !== this.cell) this.cell.attach(p.mesh);
      p.state = 'rest';
      p.vx = p.vy = p.vz = 0;
      p.mesh.position.set(p.start.x, p.start.y, p.h / 2);
      p.mesh.quaternion.setFromAxisAngle(new THREE.Vector3(0, 0, 1), p.start.yaw);
    }
  }

  // Highest surface under (x, y) for a part of radius r, ignoring `self`.
  support(x, y, r, self) {
    let z = FLOOR_Z;
    if (x > TABLE.x0 && x < TABLE.x1 && y > TABLE.y0 && y < TABLE.y1) z = 0;
    for (const t of TRAY_DEFS) {
      const dx = Math.abs(x - t.x), dy = Math.abs(y - t.y);
      const half = t.inner / 2;
      if (dx < half && dy < half) z = Math.max(z, TRAY_FLOOR);
      else if (dx < half + 0.012 + r * 0.6 && dy < half + 0.012 + r * 0.6) z = Math.max(z, 0.045);
    }
    if (onBeltXY(x, y)) z = Math.max(z, CONVEYOR.z);
    if (insideRect(x, y, BIN.x, BIN.y, BIN.sx / 2, BIN.sy / 2)) z = Math.max(z, BIN.floor);
    else if (insideRect(x, y, BIN.x, BIN.y, BIN.sx / 2 + BIN.wall + r * 0.6, BIN.sy / 2 + BIN.wall + r * 0.6)) z = Math.max(z, BIN.top);
    for (const o of this.list) {
      if (o === self || o.state === 'held') continue;
      const d = Math.hypot(o.mesh.position.x - x, o.mesh.position.y - y);
      if (d < (o.r + r) * 0.92) z = Math.max(z, o.mesh.position.z + o.h / 2);
    }
    return z;
  }

  // Yaw (about Z) of the part's own X axis, folded by its symmetry.
  yawOf(p) {
    const e = new THREE.Vector3(1, 0, 0).applyQuaternion(p.mesh.quaternion);
    return Math.atan2(e.y, e.x);
  }

  // Look for a part sitting between the open fingers. tcp is the robot's TCP Object3D.
  findGraspable(tcp, gap) {
    const inv = this.tmpM.copy(tcp.matrixWorld).invert();
    let best = null;
    let bestScore = Infinity;
    for (const p of this.list) {
      if (p.state === 'held') continue;
      p.mesh.updateWorldMatrix(true, false);
      const c = this.tmpV.setFromMatrixPosition(p.mesh.matrixWorld).applyMatrix4(inv);
      // c.x: along approach (+ is beyond the tips), c.y: finger travel, c.z: across the fingers
      const along = p.h / 2;
      const overlapsJaw = c.x - along < 0.012 && c.x + along > -0.07;
      const between = Math.abs(c.y) < gap / 2 - p.w / 2 + 0.012;
      const centred = Math.abs(c.z) < 0.022;
      if (overlapsJaw && between && centred) {
        const score = Math.abs(c.y) + Math.abs(c.z);
        if (score < bestScore) { bestScore = score; best = p; }
      }
    }
    return best;
  }

  grasp(p, tcp) {
    p.state = 'held';
    p.vx = p.vy = p.vz = 0;
    tcp.attach(p.mesh);
  }

  release(p) {
    this.cell.attach(p.mesh);
    p.state = 'falling';
    p.vx = p.vy = p.vz = 0;
    // settle upright, keeping the heading the gripper gave it
    const e = new THREE.Vector3(1, 0, 0).applyQuaternion(p.mesh.quaternion);
    const up = new THREE.Vector3(0, 0, 1).applyQuaternion(p.mesh.quaternion);
    let yaw = Math.atan2(e.y, e.x);
    if (Math.abs(up.z) < 0.5) {
      // part held sideways: use whichever local axis is horizontal-most
      const f = new THREE.Vector3(0, 0, 1).applyQuaternion(p.mesh.quaternion);
      yaw = Math.atan2(f.y, f.x);
    }
    p.settleQ.setFromAxisAngle(new THREE.Vector3(0, 0, 1), yaw);
  }

  // Free slot inside a tray (2x2 grid), or null when all four are taken.
  // `reserved` lists slots promised to moves that are still on their way.
  freeSlot(trayId, reserved = []) {
    const t = TRAY_DEFS.find((d) => d.id === trayId);
    const o = t.inner / 4 + 0.004;
    const slots = [[-o, -o], [o, -o], [-o, o], [o, o]].map(([dx, dy]) => [t.x + dx, t.y + dy]);
    for (const [x, y] of slots) {
      const near = (px, py) => Math.hypot(px - x, py - y) < 0.045;
      const busy = this.list.some((p) => p.state !== 'held' && near(p.mesh.position.x, p.mesh.position.y))
        || reserved.some(([rx, ry]) => near(rx, ry));
      if (!busy) return [x, y];
    }
    return null;
  }

  // Free slot, falling back to the tray centre (parts then stack).
  traySlot(trayId) {
    const t = TRAY_DEFS.find((d) => d.id === trayId);
    return this.freeSlot(trayId) || [t.x, t.y];
  }

  trayAt(x, y) {
    return TRAY_DEFS.find((t) => Math.abs(x - t.x) < t.inner / 2 + 0.012 && Math.abs(y - t.y) < t.inner / 2 + 0.012) || null;
  }

  onBelt(p) { return p.state === 'rest' && onBeltXY(p.mesh.position.x, p.mesh.position.y) && p.mesh.position.z > CONVEYOR.z; }

  // True when nothing sits within `clear` metres of the conveyor infeed spot.
  infeedClear(clear = 0.09) {
    const [x, y] = CONVEYOR.infeed;
    return !this.list.some((p) => p.state !== 'held' && Math.hypot(p.mesh.position.x - x, p.mesh.position.y - y) < clear);
  }

  // Where a part is, for the UI and the line logic.
  whereIs(p) {
    if (p.state === 'held') return { kind: 'held' };
    const { x, y, z } = p.mesh.position;
    if (insideRect(x, y, BIN.x, BIN.y, BIN.sx / 2 + BIN.wall, BIN.sy / 2 + BIN.wall) && z < BIN.top) return { kind: 'bin' };
    if (p.state === 'falling') return { kind: 'moving' };
    if (onBeltXY(x, y) && z > CONVEYOR.z) return { kind: 'conveyor' };
    const tray = this.trayAt(x, y);
    if (tray) return { kind: 'tray', tray };
    if (z < FLOOR_Z + 0.2) return { kind: 'floor' };
    return { kind: 'table' };
  }

  // Advance physics. Parts on a running belt travel with it and tip off its end into the bin.
  // Returns true while anything is still moving (so the renderer keeps drawing).
  step(dt, conveyorOn = false) {
    let moving = false;
    for (const p of this.list) {
      const pos = p.mesh.position;
      if (p.state === 'rest' && conveyorOn && onBeltXY(pos.x, pos.y) && pos.z > CONVEYOR.z) {
        moving = true;
        pos.y -= CONVEYOR.speed * dt;
        if (pos.y < CONVEYOR.y1) {
          p.state = 'falling';
          p.vx = 0;
          p.vy = -CONVEYOR.speed;
          p.vz = 0;
          p.settleQ.copy(p.mesh.quaternion);
        }
        continue;
      }
      if (p.state !== 'falling') continue;
      moving = true;
      p.vz -= 9.81 * dt;
      pos.x += p.vx * dt;
      pos.y += p.vy * dt;
      pos.z += p.vz * dt;
      p.mesh.quaternion.slerp(p.settleQ, Math.min(1, dt * 14));
      const floor = this.support(pos.x, pos.y, p.r, p) + p.h / 2;
      if (pos.z <= floor) {
        pos.z = floor;
        p.vx *= 0.3;
        p.vy *= 0.3;
        if (p.vz < -0.6) p.vz = -p.vz * 0.22;
        else {
          p.vx = p.vy = p.vz = 0;
          p.state = 'rest';
          p.mesh.quaternion.copy(p.settleQ);
        }
      }
    }
    return moving;
  }

  setHighlight(id) {
    for (const p of this.list) p.mesh.material.emissiveIntensity = p.id === id ? 0.35 : 0;
  }
}
