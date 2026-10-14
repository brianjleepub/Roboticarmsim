// Workpieces: creation, simple rigid-body drop physics, grasp detection and tray slots.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { TABLE, FLOOR_Z, TRAY_DEFS, TRAY_INNER, TRAY_FLOOR } from './scene.js';

const COLORS = { red: 0xd23b33, yellow: 0xf0b21b, blue: 0x2f6fd6 };

// Example layout, deterministic so "Reset parts" always gives the same scene.
const START = [
  { color: 'red', kind: 'cube', x: 0.56, y: -0.2, yaw: 0.35 },
  { color: 'blue', kind: 'cyl', x: 0.72, y: -0.04, yaw: 0 },
  { color: 'yellow', kind: 'cube', x: 0.62, y: 0.17, yaw: -0.5 },
  { color: 'red', kind: 'cyl', x: 0.84, y: 0.16, yaw: 0 },
  { color: 'blue', kind: 'cube', x: 0.47, y: 0.03, yaw: 0.9 },
  { color: 'yellow', kind: 'cyl', x: 0.83, y: -0.24, yaw: 0 },
];

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
      state: 'rest', vz: 0, settleQ: new THREE.Quaternion(),
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
      p.vz = 0;
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
      const half = TRAY_INNER / 2;
      if (dx < half && dy < half) z = Math.max(z, TRAY_FLOOR);
      else if (dx < half + 0.012 + r * 0.6 && dy < half + 0.012 + r * 0.6) z = Math.max(z, 0.045);
    }
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
    p.vz = 0;
    tcp.attach(p.mesh);
  }

  release(p) {
    this.cell.attach(p.mesh);
    p.state = 'falling';
    p.vz = 0;
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

  // Free slot inside a tray (2x2 grid), falling back to the centre.
  traySlot(trayId) {
    const t = TRAY_DEFS.find((d) => d.id === trayId);
    const o = TRAY_INNER / 4 + 0.004;
    const slots = [[-o, -o], [o, -o], [-o, o], [o, o]].map(([dx, dy]) => [t.x + dx, t.y + dy]);
    for (const [x, y] of slots) {
      const busy = this.list.some((p) => p.state !== 'held' && Math.hypot(p.mesh.position.x - x, p.mesh.position.y - y) < 0.045);
      if (!busy) return [x, y];
    }
    return [t.x, t.y];
  }

  trayAt(x, y) {
    return TRAY_DEFS.find((t) => Math.abs(x - t.x) < TRAY_INNER / 2 + 0.012 && Math.abs(y - t.y) < TRAY_INNER / 2 + 0.012) || null;
  }

  // Returns true while anything is still moving (so the renderer keeps drawing).
  step(dt) {
    let moving = false;
    for (const p of this.list) {
      if (p.state !== 'falling') continue;
      moving = true;
      const pos = p.mesh.position;
      p.vz -= 9.81 * dt;
      pos.z += p.vz * dt;
      p.mesh.quaternion.slerp(p.settleQ, Math.min(1, dt * 14));
      const floor = this.support(pos.x, pos.y, p.r, p) + p.h / 2;
      if (pos.z <= floor) {
        pos.z = floor;
        if (p.vz < -0.6) p.vz = -p.vz * 0.22;
        else {
          p.vz = 0;
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
