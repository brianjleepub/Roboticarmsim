// Renderer, lighting and the static work cell (floor, risers, table, trays, conveyor, bin, fence).
// The cell group is Z-up with its origin on robot 1's mounting plate, same as kinematics.js.
import * as THREE from 'three';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { RoundedBoxGeometry } from 'three/addons/geometries/RoundedBoxGeometry.js';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';

export const RISER_H = 0.75; // robot mount and table top height above the floor
export const FLOOR_Z = -RISER_H;
// Two identical arms face each other across one table. Robot 2 is turned 180° about Z.
export const ROBOT_BASES = [
  { x: 0, y: 0, yaw: 0 },
  { x: 1.7, y: 0, yaw: Math.PI },
];
export const TABLE = { x0: 0.3, x1: 1.42, y0: -0.78, y1: 0.78 };

export const TRAY_INNER = 0.21;
export const TRAY_DEFS = [
  { id: 'red', label: 'Red', color: 0xc8352e, x: 0.42, y: 0.56, inner: TRAY_INNER },
  { id: 'yellow', label: 'Yellow', color: 0xe0a816, x: 0.74, y: 0.54, inner: TRAY_INNER },
  { id: 'blue', label: 'Blue', color: 0x2c64c8, x: 0.6, y: -0.56, inner: TRAY_INNER },
  // shared transfer tray in the middle, reachable by both arms
  { id: 'transfer', label: 'Transfer', color: 0x2f8a64, x: 0.88, y: 0, inner: 0.25, shared: true },
];

// Belt conveyor on robot 2's side. Parts travel toward -Y and drop into the bin.
export const CONVEYOR = {
  x: 1.2, width: 0.16, y0: -0.3, y1: -1.2, z: 0.07, speed: 0.14,
  infeed: [1.2, -0.4], // where robot 2 sets parts down
};
export const BIN = { x: 1.2, y: -1.39, sx: 0.42, sy: 0.44, wall: 0.015, floor: FLOOR_Z + 0.06, top: -0.22 };
export const TRAY_FLOOR = 0.012;
const TRAY_WALL_H = 0.045;

export function createRenderer(canvas) {
  const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, powerPreference: 'high-performance' });
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.05;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  return renderer;
}

function canvasTex(size, draw, repeat = 1, srgb = true) {
  const c = document.createElement('canvas');
  c.width = c.height = size;
  const ctx = c.getContext('2d');
  draw(ctx, size);
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.repeat.set(repeat, repeat);
  t.anisotropy = 8;
  if (srgb) t.colorSpace = THREE.SRGBColorSpace;
  return t;
}

// Deterministic PRNG so the concrete looks the same every load.
function rng(seed) {
  let a = seed;
  return () => {
    a |= 0; a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function concreteTextures() {
  const r = rng(7);
  const color = canvasTex(1024, (ctx, s) => {
    ctx.fillStyle = '#7d7b77';
    ctx.fillRect(0, 0, s, s);
    // blotchy trowel variation
    for (let i = 0; i < 900; i++) {
      const x = r() * s, y = r() * s, rad = 10 + r() * 90;
      const g = ctx.createRadialGradient(x, y, 0, x, y, rad);
      const v = 100 + Math.floor(r() * 40);
      g.addColorStop(0, `rgba(${v},${v - 2},${v - 6},0.10)`);
      g.addColorStop(1, 'rgba(0,0,0,0)');
      ctx.fillStyle = g;
      ctx.fillRect(x - rad, y - rad, rad * 2, rad * 2);
    }
    // aggregate speckle
    const img = ctx.getImageData(0, 0, s, s);
    for (let i = 0; i < img.data.length; i += 4) {
      const n = (r() - 0.5) * 26;
      img.data[i] += n; img.data[i + 1] += n; img.data[i + 2] += n;
    }
    ctx.putImageData(img, 0, 0);
    // saw-cut joints
    ctx.strokeStyle = 'rgba(40,38,36,0.55)';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(0, 1.5); ctx.lineTo(s, 1.5);
    ctx.moveTo(1.5, 0); ctx.lineTo(1.5, s);
    ctx.stroke();
  }, 6);
  const rough = canvasTex(512, (ctx, s) => {
    ctx.fillStyle = '#c8c8c8';
    ctx.fillRect(0, 0, s, s);
    for (let i = 0; i < 400; i++) {
      const x = r() * s, y = r() * s, rad = 10 + r() * 60;
      const g = ctx.createRadialGradient(x, y, 0, x, y, rad);
      g.addColorStop(0, `rgba(90,90,90,${0.15 + r() * 0.25})`);
      g.addColorStop(1, 'rgba(0,0,0,0)');
      ctx.fillStyle = g;
      ctx.fillRect(x - rad, y - rad, rad * 2, rad * 2);
    }
  }, 6, false);
  return { color, rough };
}

function hazardTexture() {
  const t = canvasTex(256, (ctx, s) => {
    ctx.fillStyle = '#f2c230';
    ctx.fillRect(0, 0, s, s);
    ctx.fillStyle = '#121212';
    for (let i = -s; i < s * 2; i += 64) {
      ctx.beginPath();
      ctx.moveTo(i, 0); ctx.lineTo(i + 32, 0); ctx.lineTo(i + 32 + s, s); ctx.lineTo(i + s, s);
      ctx.fill();
    }
  });
  return t;
}

function meshTexture() {
  return canvasTex(256, (ctx, s) => {
    ctx.clearRect(0, 0, s, s);
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 4;
    for (let i = 0; i <= s; i += 32) {
      ctx.beginPath(); ctx.moveTo(i, 0); ctx.lineTo(i, s); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0, i); ctx.lineTo(s, i); ctx.stroke();
    }
  }, 1, false);
}

function m(geo, mat, cast = true, recv = true) {
  const x = new THREE.Mesh(geo, mat);
  x.castShadow = cast;
  x.receiveShadow = recv;
  return x;
}
function box(sx, sy, sz, at, r = 0) {
  const g = r > 0 ? new RoundedBoxGeometry(sx, sy, sz, 3, r) : new THREE.BoxGeometry(sx, sy, sz);
  g.translate(...at);
  return g.index ? g.toNonIndexed() : g;
}

export function createWorld(renderer) {
  const scene = new THREE.Scene();
  const bg = new THREE.Color(0x1d2024);
  scene.background = bg;
  scene.fog = new THREE.Fog(bg, 7, 18);

  const pmrem = new THREE.PMREMGenerator(renderer);
  scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
  scene.environmentIntensity = 0.6;

  const hemi = new THREE.HemisphereLight(0xdde6f2, 0x3a342c, 0.55);
  scene.add(hemi);

  const key = new THREE.DirectionalLight(0xfff3e2, 2.6);
  key.position.set(3.0, 5.4, 3.2);
  key.target.position.set(0.85, 0.5, 0.3);
  key.castShadow = true;
  key.shadow.mapSize.set(2048, 2048);
  const sc = key.shadow.camera;
  sc.left = -2.6; sc.right = 2.6; sc.top = 2.6; sc.bottom = -2.6; sc.near = 1; sc.far = 13;
  key.shadow.bias = -0.0003;
  key.shadow.normalBias = 0.02;
  key.shadow.radius = 3;
  scene.add(key, key.target);

  const rim = new THREE.DirectionalLight(0xbfd4ff, 0.7);
  rim.position.set(-3, 3, -2.5);
  scene.add(rim);

  // Z-up cell frame on the robot mount
  const cell = new THREE.Group();
  cell.rotation.x = -Math.PI / 2;
  cell.position.y = RISER_H;
  scene.add(cell);

  const mats = {
    steel: new THREE.MeshStandardMaterial({ color: 0x3b4047, roughness: 0.5, metalness: 0.6 }),
    alu: new THREE.MeshStandardMaterial({ color: 0xb9bec5, roughness: 0.35, metalness: 0.9 }),
    top: new THREE.MeshStandardMaterial({ color: 0x5d646c, roughness: 0.62, metalness: 0.25 }),
    yellow: new THREE.MeshStandardMaterial({ color: 0xe8b81e, roughness: 0.45, metalness: 0.2 }),
    dark: new THREE.MeshStandardMaterial({ color: 0x1b1d20, roughness: 0.6, metalness: 0.3 }),
    cabinet: new THREE.MeshStandardMaterial({ color: 0xd8d9d6, roughness: 0.45, metalness: 0.15 }),
  };

  // ----- floor -----
  const { color, rough } = concreteTextures();
  const floorMat = new THREE.MeshStandardMaterial({ map: color, roughnessMap: rough, roughness: 0.92, metalness: 0 });
  const floor = m(new THREE.PlaneGeometry(24, 24), floorMat, false, true);
  floor.position.z = FLOOR_Z;
  cell.add(floor);
  // hazard border on the floor around the cell
  const hz = hazardTexture();
  const stripe = (w, h, x, y, rot) => {
    const t = hz.clone();
    t.needsUpdate = true;
    t.repeat.set(Math.max(w, h) / 0.18, 1);
    const s = m(new THREE.PlaneGeometry(Math.max(w, h), Math.min(w, h)), new THREE.MeshStandardMaterial({ map: t, roughness: 0.7 }), false, true);
    s.position.set(x, y, FLOOR_Z + 0.001);
    s.rotation.z = rot;
    cell.add(s);
  };
  const bx0 = -1.15, bx1 = 2.85, by0 = -1.92, by1 = 1.45, bw = 0.08;
  stripe(bx1 - bx0, bw, (bx0 + bx1) / 2, by0, 0);
  stripe(bx1 - bx0, bw, (bx0 + bx1) / 2, by1, 0);
  stripe(by1 - by0, bw, bx0, 0, Math.PI / 2);
  stripe(by1 - by0, bw, bx1, 0, Math.PI / 2);

  // ----- robot risers -----
  const riserGeo = [];
  for (const b of ROBOT_BASES) {
    riserGeo.push(
      box(0.46, 0.46, 0.03, [b.x, b.y, -0.015], 0.006),
      box(0.3, 0.3, RISER_H - 0.06, [b.x, b.y, -RISER_H / 2], 0.01),
      box(0.6, 0.6, 0.03, [b.x, b.y, FLOOR_Z + 0.015], 0.006),
    );
    for (let i = 0; i < 4; i++) {
      const g = box(0.012, 0.13, 0.2, [0, 0.21, FLOOR_Z + 0.13]);
      g.rotateZ((i * Math.PI) / 2);
      g.translate(b.x, b.y, 0);
      riserGeo.push(g);
    }
  }
  cell.add(m(mergeGeometries(riserGeo), mats.steel));

  // ----- work table -----
  const { x0, x1, y0, y1 } = TABLE;
  const tw = x1 - x0, td = y1 - y0;
  const top = m(box(tw, td, 0.025, [(x0 + x1) / 2, 0, -0.0125], 0.004), mats.top);
  top.name = 'table';
  cell.add(top);
  const legs = [];
  for (const lx of [x0 + 0.04, x1 - 0.04]) {
    for (const ly of [y0 + 0.04, y1 - 0.04]) legs.push(box(0.045, 0.045, RISER_H - 0.025, [lx, ly, -RISER_H / 2 - 0.0125]));
  }
  legs.push(box(tw - 0.08, 0.045, 0.045, [(x0 + x1) / 2, y0 + 0.04, -0.06]));
  legs.push(box(tw - 0.08, 0.045, 0.045, [(x0 + x1) / 2, y1 - 0.04, -0.06]));
  legs.push(box(0.045, td - 0.08, 0.045, [x1 - 0.04, 0, -0.06]));
  legs.push(box(0.045, td - 0.08, 0.045, [x0 + 0.04, 0, -0.06]));
  legs.push(box(tw - 0.08, td - 0.08, 0.02, [(x0 + x1) / 2, 0, FLOOR_Z + 0.16]));
  cell.add(m(mergeGeometries(legs), mats.alu));

  // ----- sorting trays and the shared transfer tray -----
  const trays = [];
  for (const def of TRAY_DEFS) {
    const g = new THREE.Group();
    g.position.set(def.x, def.y, 0);
    const inner = def.inner, wall = 0.012, outer = inner + 2 * wall;
    const trayMat = new THREE.MeshStandardMaterial({ color: def.color, roughness: 0.5, metalness: 0.05 });
    const parts = [
      box(outer, outer, TRAY_FLOOR, [0, 0, TRAY_FLOOR / 2]),
      box(outer, wall, TRAY_WALL_H, [0, (outer - wall) / 2, TRAY_WALL_H / 2]),
      box(outer, wall, TRAY_WALL_H, [0, -(outer - wall) / 2, TRAY_WALL_H / 2]),
      box(wall, inner, TRAY_WALL_H, [(outer - wall) / 2, 0, TRAY_WALL_H / 2]),
      box(wall, inner, TRAY_WALL_H, [-(outer - wall) / 2, 0, TRAY_WALL_H / 2]),
    ];
    const tm = m(mergeGeometries(parts), trayMat);
    tm.userData.trayId = def.id;
    g.add(tm);
    cell.add(g);
    trays.push({ ...def, group: g, mesh: tm, inner });
  }

  const conveyor = buildConveyor(cell, mats);
  const bin = buildBin(cell);

  // ----- safety fence (back and left side) -----
  const fenceTex = meshTexture();
  const panelMat = new THREE.MeshStandardMaterial({
    color: 0x15171a, alphaMap: fenceTex, transparent: true, opacity: 0.55, depthWrite: false, side: THREE.DoubleSide, roughness: 0.6, metalness: 0.5,
  });
  const postGeo = [];
  const fenceH = 2.0;
  const addPanel = (ax, ay, bx, by) => {
    const len = Math.hypot(bx - ax, by - ay);
    const t = fenceTex.clone();
    t.needsUpdate = true;
    t.repeat.set(len / 0.4, (fenceH - 0.2) / 0.4);
    const mat = panelMat.clone();
    mat.alphaMap = t;
    const p = m(new THREE.PlaneGeometry(len, fenceH - 0.2), mat, false, false);
    p.position.set((ax + bx) / 2, (ay + by) / 2, FLOOR_Z + 0.1 + (fenceH - 0.2) / 2);
    // stand the plane upright, then turn it to run along the segment
    p.quaternion.setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.atan2(by - ay, bx - ax))
      .multiply(new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI / 2));
    cell.add(p);
    postGeo.push(box(0.05, 0.05, fenceH, [ax, ay, FLOOR_Z + fenceH / 2]));
    postGeo.push(box(0.05, 0.05, fenceH, [bx, by, FLOOR_Z + fenceH / 2]));
    const cx = (ax + bx) / 2, cy = (ay + by) / 2;
    const rail = box(len, 0.035, 0.035, [0, 0, 0]);
    rail.rotateZ(Math.atan2(by - ay, bx - ax));
    rail.translate(cx, cy, FLOOR_Z + fenceH - 0.04);
    postGeo.push(rail);
  };
  const fx0 = -1.4, fy0 = -1.7, fy1 = 1.7, fx1 = 3.1;
  const step = (fy1 - fy0) / 3;
  for (let i = 0; i < 3; i++) {
    addPanel(fx0, fy0 + i * step, fx0, fy0 + (i + 1) * step);
    addPanel(fx1, fy0 + i * step, fx1, fy0 + (i + 1) * step);
  }
  const step2 = (fx1 - fx0) / 5;
  for (let i = 0; i < 5; i++) addPanel(fx0 + i * step2, fy1, fx0 + (i + 1) * step2, fy1);
  cell.add(m(mergeGeometries(postGeo), mats.yellow));

  // ----- one controller cabinet per robot + stack light -----
  for (const cx of [-0.95, 2.65]) {
    const cab = new THREE.Group();
    cab.position.set(cx, 1.05, FLOOR_Z);
    cab.rotation.z = -Math.PI / 2;
    cab.add(m(mergeGeometries([box(0.6, 0.5, 1.0, [0, 0, 0.55], 0.01), box(0.64, 0.54, 0.08, [0, 0, 0.04])]), mats.cabinet));
    const vents = [];
    for (let i = 0; i < 6; i++) vents.push(box(0.3, 0.006, 0.012, [0, -0.252, 0.25 + i * 0.03]));
    vents.push(box(0.12, 0.006, 0.12, [0.16, -0.252, 0.85]));
    cab.add(m(mergeGeometries(vents), mats.dark));
    const sw = m(new THREE.CylinderGeometry(0.035, 0.035, 0.03, 24), new THREE.MeshStandardMaterial({ color: 0xc23a2a, roughness: 0.4 }));
    sw.position.set(-0.16, -0.26, 0.85);
    cab.add(sw);
    cell.add(cab);
  }

  const stack = new THREE.Group();
  stack.position.set(-0.95, 0.85, FLOOR_Z + 1.1);
  stack.add(m(new THREE.CylinderGeometry(0.012, 0.012, 0.16, 12).rotateX(Math.PI / 2).translate(0, 0, 0.08), mats.alu));
  const lamps = {};
  const lampColors = { green: 0x23d06a, amber: 0xffa21a, red: 0xff3b30 };
  let z = 0.18;
  for (const k of ['green', 'amber', 'red']) {
    const mat = new THREE.MeshStandardMaterial({ color: lampColors[k], emissive: lampColors[k], emissiveIntensity: 0.05, roughness: 0.3, transparent: true, opacity: 0.92 });
    const l = m(new THREE.CylinderGeometry(0.035, 0.035, 0.06, 24).rotateX(Math.PI / 2), mat, false, false);
    l.position.z = z;
    z += 0.064;
    stack.add(l);
    lamps[k] = mat;
  }
  stack.add(m(new THREE.CylinderGeometry(0.036, 0.036, 0.015, 24).rotateX(Math.PI / 2).translate(0, 0, z - 0.025), mats.dark));
  cell.add(stack);

  const setStackLight = (state) => {
    lamps.green.emissiveIntensity = state === 'green' ? 2.6 : 0.04;
    lamps.amber.emissiveIntensity = state === 'amber' ? 2.6 : 0.04;
    lamps.red.emissiveIntensity = state === 'red' ? 2.8 : 0.04;
  };
  setStackLight('amber');

  return { scene, cell, key, trays, table: top, conveyor, bin, setStackLight };
}

function beltTexture() {
  const t = canvasTex(128, (ctx, s) => {
    ctx.fillStyle = '#1c1e21';
    ctx.fillRect(0, 0, s, s);
    const img = ctx.getImageData(0, 0, s, s);
    const r = rng(11);
    for (let i = 0; i < img.data.length; i += 4) {
      const n = (r() - 0.5) * 14;
      img.data[i] += n; img.data[i + 1] += n; img.data[i + 2] += n;
    }
    ctx.putImageData(img, 0, 0);
    // moulded cross cleats make the belt motion readable
    ctx.fillStyle = '#34373c';
    ctx.fillRect(0, 0, s, 10);
    ctx.fillStyle = '#0d0e10';
    ctx.fillRect(0, 10, s, 3);
  });
  return t;
}

// Flat belt conveyor: frame, rollers, drive motor, guide rails and an animated belt surface.
function buildConveyor(cell, mats) {
  const C = CONVEYOR;
  const len = C.y0 - C.y1;
  const cy = (C.y0 + C.y1) / 2;
  const g = new THREE.Group();
  const frame = [
    box(0.025, len + 0.06, 0.06, [C.x - C.width / 2 - 0.015, cy, C.z - 0.035]),
    box(0.025, len + 0.06, 0.06, [C.x + C.width / 2 + 0.015, cy, C.z - 0.035]),
    // feet resting on the table at the infeed end, full legs past the table edge
    box(0.03, 0.03, C.z - 0.065, [C.x - 0.09, C.y0 - 0.05, (C.z - 0.065) / 2]),
    box(0.03, 0.03, C.z - 0.065, [C.x + 0.09, C.y0 - 0.05, (C.z - 0.065) / 2]),
    box(0.035, 0.035, C.z - 0.065 - FLOOR_Z, [C.x - 0.09, C.y1 + 0.13, (C.z - 0.065 + FLOOR_Z) / 2]),
    box(0.035, 0.035, C.z - 0.065 - FLOOR_Z, [C.x + 0.09, C.y1 + 0.13, (C.z - 0.065 + FLOOR_Z) / 2]),
    box(0.21, 0.035, 0.035, [C.x, C.y1 + 0.13, FLOOR_Z + 0.25]),
  ];
  g.add(m(mergeGeometries(frame), mats.alu));
  const guides = [
    box(0.008, len - 0.04, 0.022, [C.x - C.width / 2 - 0.004, cy - 0.02, C.z + 0.016]),
    box(0.008, len - 0.04, 0.022, [C.x + C.width / 2 + 0.004, cy - 0.02, C.z + 0.016]),
  ];
  g.add(m(mergeGeometries(guides), mats.yellow));
  for (const y of [C.y0, C.y1]) {
    const roller = m(new THREE.CylinderGeometry(0.028, 0.028, C.width + 0.01, 24).rotateZ(Math.PI / 2), mats.alu);
    roller.position.set(C.x, y, C.z - 0.028);
    g.add(roller);
  }
  const motor = m(mergeGeometries([
    new THREE.CylinderGeometry(0.035, 0.035, 0.11, 24).rotateZ(Math.PI / 2).translate(C.x + C.width / 2 + 0.085, C.y0, C.z - 0.03).toNonIndexed(),
    box(0.05, 0.07, 0.07, [C.x + C.width / 2 + 0.04, C.y0, C.z - 0.03]),
  ]), mats.dark);
  g.add(motor);
  const tex = beltTexture();
  tex.repeat.set(1, len / 0.12);
  const beltMat = new THREE.MeshStandardMaterial({ map: tex, roughness: 0.85, metalness: 0.05 });
  const belt = m(new THREE.PlaneGeometry(C.width, len), beltMat, false, true);
  belt.position.set(C.x, cy, C.z);
  belt.name = 'conveyor';
  g.add(belt);
  // the belt's underside and the wrap around the rollers
  const under = m(box(C.width, len, 0.05, [C.x, cy, C.z - 0.028]), new THREE.MeshStandardMaterial({ color: 0x17181a, roughness: 0.9 }));
  g.add(under);
  cell.add(g);
  return {
    mesh: belt,
    // scroll the belt texture; one texture repeat is 0.12 m of belt
    update(dt) { tex.offset.y = (tex.offset.y + (C.speed * dt) / 0.12) % 1; },
  };
}

function buildBin(cell) {
  const B = BIN;
  const h = B.top - B.floor + 0.04;
  const mat = new THREE.MeshStandardMaterial({ color: 0x3b5a7a, roughness: 0.55, metalness: 0.3 });
  const z0 = B.floor - 0.04;
  const w = B.wall;
  const parts = [
    box(B.sx + 2 * w, B.sy + 2 * w, 0.04, [B.x, B.y, z0 + 0.02]),
    box(B.sx + 2 * w, w, h, [B.x, B.y - B.sy / 2 - w / 2, z0 + h / 2]),
    box(B.sx + 2 * w, w, h, [B.x, B.y + B.sy / 2 + w / 2, z0 + h / 2]),
    box(w, B.sy, h, [B.x - B.sx / 2 - w / 2, B.y, z0 + h / 2]),
    box(w, B.sy, h, [B.x + B.sx / 2 + w / 2, B.y, z0 + h / 2]),
  ];
  const binMesh = m(mergeGeometries(parts), mat);
  binMesh.name = 'bin';
  cell.add(binMesh);
  // casters
  const wheels = [];
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) {
    wheels.push(new THREE.CylinderGeometry(0.025, 0.025, 0.02, 16).rotateZ(Math.PI / 2).translate(B.x + sx * (B.sx / 2 - 0.03), B.y + sy * (B.sy / 2 - 0.03), FLOOR_Z + 0.025).toNonIndexed());
  }
  cell.add(m(mergeGeometries(wheels), new THREE.MeshStandardMaterial({ color: 0x111214, roughness: 0.8 })));
  return { mesh: binMesh };
}
