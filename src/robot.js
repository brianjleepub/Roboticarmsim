// Visual model of the arm. Every link lives in a group whose transform matches the
// kinematic chain in kinematics.js exactly, so the TCP drawn here is the TCP solved there.
import * as THREE from 'three';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { GEO } from './kinematics.js';

const { D1, A1, A2, A3, D4, D6, TOOL } = GEO;

export const GRIPPER_MAX_GAP = 0.09;

export function makeMaterials() {
  return {
    paint: new THREE.MeshPhysicalMaterial({
      color: 0xf07a12, roughness: 0.38, metalness: 0.0, clearcoat: 0.6, clearcoatRoughness: 0.22,
    }),
    cast: new THREE.MeshStandardMaterial({ color: 0x2b2f35, roughness: 0.55, metalness: 0.45 }),
    motor: new THREE.MeshStandardMaterial({ color: 0x17191c, roughness: 0.45, metalness: 0.5 }),
    alu: new THREE.MeshStandardMaterial({ color: 0xc8ccd2, roughness: 0.26, metalness: 1.0 }),
    anod: new THREE.MeshStandardMaterial({ color: 0x3a4250, roughness: 0.35, metalness: 0.8 }),
    rubber: new THREE.MeshStandardMaterial({ color: 0x0d0e10, roughness: 0.9, metalness: 0.0 }),
    cable: new THREE.MeshStandardMaterial({ color: 0x141517, roughness: 0.55, metalness: 0.1 }),
    label: new THREE.MeshStandardMaterial({ color: 0xf4f1e8, roughness: 0.6 }),
  };
}

// Cylinder along a given axis, centred at `at`.
function cyl(r1, r2, h, axis, at = [0, 0, 0], seg = 40) {
  const g = new THREE.CylinderGeometry(r2, r1, h, seg, 1);
  if (axis === 'x') g.rotateZ(-Math.PI / 2);
  else if (axis === 'z') g.rotateX(Math.PI / 2);
  g.translate(...at);
  return g;
}

// Convex hull of a list of circles, as a THREE.Shape (used for cast link profiles).
function hullShape(circles, seg = 48) {
  const pts = [];
  for (const [cx, cy, r] of circles) {
    for (let i = 0; i < seg; i++) {
      const a = (i / seg) * Math.PI * 2;
      pts.push([cx + Math.cos(a) * r, cy + Math.sin(a) * r]);
    }
  }
  pts.sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cross = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lower = [];
  for (const p of pts) {
    while (lower.length >= 2 && cross(lower[lower.length - 2], lower[lower.length - 1], p) <= 0) lower.pop();
    lower.push(p);
  }
  const upper = [];
  for (let i = pts.length - 1; i >= 0; i--) {
    const p = pts[i];
    while (upper.length >= 2 && cross(upper[upper.length - 2], upper[upper.length - 1], p) <= 0) upper.pop();
    upper.push(p);
  }
  const hull = lower.slice(0, -1).concat(upper.slice(0, -1));
  return new THREE.Shape(hull.map(([x, y]) => new THREE.Vector2(x, y)));
}

// A bevelled slab whose profile (hull of circles given in the robot X/Z plane) is
// extruded along Y, centred on y = yc.
function slab(circles, thickness, yc = 0, bevel = 0.012) {
  const shape = hullShape(circles);
  const depth = Math.max(0.001, thickness - 2 * bevel);
  const g = new THREE.ExtrudeGeometry(shape, {
    depth, bevelEnabled: true, bevelThickness: bevel, bevelSize: bevel, bevelSegments: 4, curveSegments: 12,
  });
  // shape (x, y) -> robot (x, z); extrusion axis -> robot -y
  g.rotateX(Math.PI / 2);
  g.translate(0, yc + depth / 2, 0);
  return g;
}

function rbox(sx, sy, sz, r, at) {
  const g = new RoundedBoxGeometry(sx, sy, sz, 4, r);
  g.translate(...at);
  return g;
}

// Merge geometries (keeping their own normals) into one shadowed mesh.
function mesh(geoms, mat) {
  const list = (Array.isArray(geoms) ? geoms : [geoms]).map((x) => (x.index ? x.toNonIndexed() : x));
  for (const g of list) { for (const k of Object.keys(g.attributes)) if (!['position', 'normal', 'uv'].includes(k)) g.deleteAttribute(k); }
  const m = new THREE.Mesh(mergeGeometries(list), mat);
  m.castShadow = true;
  m.receiveShadow = true;
  return m;
}

function boltCircle(r, n, z, boltR = 0.007, h = 0.008) {
  const out = [];
  for (let i = 0; i < n; i++) {
    const a = (i / n) * Math.PI * 2 + Math.PI / n;
    out.push(cyl(boltR, boltR, h, 'z', [Math.cos(a) * r, Math.sin(a) * r, z], 6));
  }
  return out;
}

export class RobotModel {
  constructor(mat) {
    this.mat = mat;
    this.root = new THREE.Group();
    this.root.name = 'robot';
    this.joints = [];
    this.gap = GRIPPER_MAX_GAP;
    this.build();
  }

  build() {
    const m = this.mat;
    const root = this.root;

    // ---------- base (fixed) ----------
    const plate = mesh([rbox(0.40, 0.40, 0.03, 0.008, [0, 0, 0.015])], m.cast);
    const bolts = mesh([
      ...[[1, 1], [1, -1], [-1, 1], [-1, -1]].map(([sx, sy]) => cyl(0.012, 0.012, 0.016, 'z', [sx * 0.16, sy * 0.16, 0.038], 6)),
    ], m.alu);
    const profile = [
      [0, 0.03], [0.19, 0.03], [0.195, 0.04], [0.19, 0.06], [0.175, 0.08], [0.168, 0.15], [0.172, 0.158], [0.172, 0.165], [0, 0.165],
    ].map(([r, z]) => new THREE.Vector2(r, z));
    const baseG = new THREE.LatheGeometry(profile, 64);
    baseG.rotateX(Math.PI / 2);
    const base = new THREE.Mesh(baseG, m.paint);
    base.castShadow = base.receiveShadow = true;
    const j1motor = mesh([cyl(0.055, 0.055, 0.13, 'z', [-0.17, 0, 0.11]), cyl(0.04, 0.04, 0.02, 'z', [-0.17, 0, 0.185])], m.motor);
    const conn = mesh([rbox(0.06, 0.09, 0.05, 0.01, [-0.2, 0, 0.06])], m.cast);
    root.add(plate, bolts, base, j1motor, conn);

    // ---------- J1 turret ----------
    const j1 = new THREE.Group();
    root.add(j1);
    const ring = mesh([cyl(0.165, 0.165, 0.03, 'z', [0, 0, 0.18], 64)], m.cast);
    const turret = mesh([
      rbox(0.36, 0.30, 0.10, 0.03, [0.02, 0, 0.245]),
    ], m.paint);
    const cheekCircles = [[0.0, 0.24, 0.13], [A1, D1, 0.115]];
    const cheeks = mesh([slab(cheekCircles, 0.05, 0.108), slab(cheekCircles, 0.05, -0.108)], m.paint);
    const j2motor = mesh([
      cyl(0.085, 0.085, 0.04, 'y', [A1, 0.152, D1]),
      cyl(0.06, 0.06, 0.13, 'y', [A1, 0.235, D1]),
    ], m.motor);
    const j2cap = mesh([cyl(0.045, 0.045, 0.012, 'y', [A1, 0.306, D1]), cyl(0.07, 0.07, 0.012, 'y', [A1, -0.139, D1])], m.alu);
    const j1motorTop = mesh([cyl(0.05, 0.05, 0.12, 'z', [-0.12, 0, 0.35]), cyl(0.035, 0.035, 0.02, 'z', [-0.12, 0, 0.42])], m.motor);
    j1.add(ring, turret, cheeks, j2motor, j2cap, j1motorTop);

    // ---------- J2 upper arm ----------
    const j2 = new THREE.Group();
    j2.position.set(A1, 0, D1);
    j1.add(j2);
    const upper = mesh([slab([[0, 0, 0.1], [0, A2, 0.082]], 0.15, 0, 0.018)], m.paint);
    const upperRecess = mesh([
      cyl(0.07, 0.07, 0.006, 'y', [0, 0.077, 0]), cyl(0.07, 0.07, 0.006, 'y', [0, -0.077, 0]),
    ], m.cast);
    j2.add(upper, upperRecess);
    for (const side of [1, -1]) j2.add(this.makeBadge(side));

    // ---------- J3 elbow / forearm housing ----------
    const j3 = new THREE.Group();
    j3.position.set(0, 0, A2);
    j2.add(j3);
    const elbowCircles = [[0, 0, 0.092], [0.2, A3, 0.075]];
    const elbowCheeks = mesh([slab(elbowCircles, 0.045, 0.1, 0.012), slab(elbowCircles, 0.045, -0.1, 0.012)], m.paint);
    const elbowTop = mesh([rbox(0.36, 0.245, 0.09, 0.03, [0.02, 0, 0.15])], m.paint);
    const j3motor = mesh([cyl(0.075, 0.075, 0.035, 'y', [0, 0.14, 0]), cyl(0.055, 0.055, 0.1, 'y', [0, 0.205, 0])], m.motor);
    const j3cap = mesh([cyl(0.05, 0.05, 0.01, 'y', [0, -0.128, 0]), cyl(0.04, 0.04, 0.01, 'y', [0, 0.258, 0])], m.alu);
    const j4motor = mesh([cyl(0.048, 0.048, 0.16, 'x', [-0.22, 0, 0.15]), cyl(0.034, 0.034, 0.02, 'x', [-0.31, 0, 0.15])], m.motor);
    const gearbox = mesh([cyl(0.075, 0.075, 0.08, 'x', [0.22, 0, A3])], m.cast);
    j3.add(elbowCheeks, elbowTop, j3motor, j3cap, j4motor, gearbox);

    // ---------- J4 forearm (roll) ----------
    const j4 = new THREE.Group();
    j4.position.set(0, 0, A3);
    j3.add(j4);
    const fStart = 0.26;
    const fEnd = D4 - 0.12;
    const forearm = mesh([
      cyl(0.068, 0.056, fEnd - fStart, 'x', [(fStart + fEnd) / 2, 0, 0], 48),
      cyl(0.072, 0.072, 0.02, 'x', [fStart + 0.01, 0, 0], 48),
    ], m.paint);
    const prongCircles = [[D4 - 0.15, 0, 0.056], [D4, 0, 0.056]];
    const prongs = mesh([slab(prongCircles, 0.028, 0.062, 0.008), slab(prongCircles, 0.028, -0.062, 0.008)], m.paint);
    const collar = mesh([cyl(0.062, 0.062, 0.03, 'x', [D4 - 0.13, 0, 0], 48)], m.cast);
    const j5caps = mesh([cyl(0.03, 0.03, 0.006, 'y', [D4, 0.079, 0]), cyl(0.03, 0.03, 0.006, 'y', [D4, -0.079, 0])], m.alu);
    j4.add(forearm, prongs, collar, j5caps);

    // ---------- J5 wrist (pitch) ----------
    const j5 = new THREE.Group();
    j5.position.set(D4, 0, 0);
    j4.add(j5);
    const wristBody = mesh([cyl(0.05, 0.05, 0.094, 'y', [0, 0, 0], 48)], m.cast);
    const nose = mesh([cyl(0.048, 0.042, D6 - 0.012, 'x', [(D6 - 0.012) / 2, 0, 0], 48)], m.paint);
    j5.add(wristBody, nose);

    // ---------- J6 flange (roll) + gripper ----------
    const j6 = new THREE.Group();
    j6.position.set(D6, 0, 0);
    j5.add(j6);
    const flange = mesh([cyl(0.04, 0.04, 0.012, 'x', [-0.006, 0, 0], 48), ...boltCircle(0.03, 6, 0).map((g) => { g.rotateY(Math.PI / 2); return g; })], m.alu);
    const adapter = mesh([cyl(0.034, 0.034, 0.018, 'x', [0.009, 0, 0], 32)], m.anod);
    const body = mesh([rbox(0.065, 0.15, 0.06, 0.008, [0.0505, 0, 0])], m.anod);
    const rail = mesh([cyl(0.006, 0.006, 0.15, 'y', [0.088, 0, 0.018], 12), cyl(0.006, 0.006, 0.15, 'y', [0.088, 0, -0.018], 12)], m.alu);
    const valve = mesh([rbox(0.03, 0.04, 0.02, 0.004, [0.04, 0, 0.04])], m.motor);
    j6.add(flange, adapter, body, rail, valve);

    this.fingers = [];
    for (const side of [1, -1]) {
      const f = new THREE.Group();
      const blade = mesh([rbox(0.105, 0.012, 0.032, 0.003, [0.1325, side * 0.006, 0])], m.alu);
      const carriage = mesh([rbox(0.024, 0.026, 0.05, 0.004, [0.09, side * 0.013, 0])], m.alu);
      const pad = mesh([rbox(0.045, 0.004, 0.028, 0.0015, [0.16, -side * 0.002, 0])], m.rubber);
      f.add(blade, carriage, pad);
      f.userData.side = side;
      j6.add(f);
      this.fingers.push(f);
    }

    const tcp = new THREE.Object3D();
    tcp.position.set(TOOL, 0, 0);
    j6.add(tcp);
    this.tcp = tcp;

    this.joints = [j1, j2, j3, j4, j5, j6];
    this.links = { j1, j2, j3 };

    // dress pack (cable) from turret to forearm housing, rebuilt as joints move
    this.cableMesh = new THREE.Mesh(new THREE.BufferGeometry(), m.cable);
    this.cableMesh.castShadow = true;
    root.add(this.cableMesh);
    this.cableAnchors = [
      [j1, new THREE.Vector3(-0.12, 0.17, 0.33)],
      [j1, new THREE.Vector3(-0.02, 0.2, 0.47)],
      [j2, new THREE.Vector3(-0.04, 0.115, 0.18)],
      [j2, new THREE.Vector3(-0.02, 0.115, 0.42)],
      [j3, new THREE.Vector3(-0.12, 0.135, 0.1)],
      [j3, new THREE.Vector3(-0.18, 0.06, 0.2)],
    ];
    this.setGap(GRIPPER_MAX_GAP);
  }

  // Model badge printed along the upper arm (canvas texture, no external assets).
  makeBadge(side) {
    if (!RobotModel.badgeTex) {
      const c = document.createElement('canvas');
      c.width = 512; c.height = 128;
      const ctx = c.getContext('2d');
      ctx.fillStyle = '#1a1b1e';
      ctx.font = '700 92px "Barlow Semi Condensed", "Arial Narrow", sans-serif';
      ctx.textBaseline = 'middle';
      ctx.fillText('RX-6', 10, 68);
      ctx.font = '600 30px "Barlow Semi Condensed", "Arial Narrow", sans-serif';
      ctx.fillText('10 kg · 1100', 250, 50);
      ctx.fillText('6-AXIS', 250, 88);
      const t = new THREE.CanvasTexture(c);
      t.colorSpace = THREE.SRGBColorSpace;
      t.anisotropy = 8;
      RobotModel.badgeTex = t;
    }
    const mat = new THREE.MeshStandardMaterial({ map: RobotModel.badgeTex, transparent: true, roughness: 0.4, polygonOffset: true, polygonOffsetFactor: -2 });
    const plane = new THREE.Mesh(new THREE.PlaneGeometry(0.3, 0.075), mat);
    // lay the text along the arm (robot Z), on the arm's side face
    const basis = new THREE.Matrix4().makeBasis(
      new THREE.Vector3(0, 0, 1), new THREE.Vector3(side, 0, 0), new THREE.Vector3(0, side, 0),
    );
    plane.quaternion.setFromRotationMatrix(basis);
    plane.position.set(0, side * 0.0765, 0.31);
    return plane;
  }

  setJoints(q) {
    const [j1, j2, j3, j4, j5, j6] = this.joints;
    j1.rotation.set(0, 0, q[0]);
    j2.rotation.set(0, q[1], 0);
    j3.rotation.set(0, q[2], 0);
    j4.rotation.set(q[3], 0, 0);
    j5.rotation.set(0, q[4], 0);
    j6.rotation.set(q[5], 0, 0);
    this.root.updateMatrixWorld(true);
    this.updateCable();
  }

  // gap = clear distance between the finger pads
  setGap(gap) {
    this.gap = gap;
    for (const f of this.fingers) f.position.y = f.userData.side * gap / 2;
  }

  updateCable() {
    const inv = new THREE.Matrix4().copy(this.root.matrixWorld).invert();
    const pts = this.cableAnchors.map(([obj, p]) => p.clone().applyMatrix4(obj.matrixWorld).applyMatrix4(inv));
    const curve = new THREE.CatmullRomCurve3(pts, false, 'centripetal');
    const g = new THREE.TubeGeometry(curve, 48, 0.017, 8, false);
    this.cableMesh.geometry.dispose();
    this.cableMesh.geometry = g;
  }
}
