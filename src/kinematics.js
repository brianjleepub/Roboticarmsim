// Kinematics for a 6-axis industrial arm with a spherical wrist.
// Pure math, no three.js: robot frame is Z-up, X forward, units are metres and radians.
//
// Chain (all joints revolute):
//   J1 about Z at the base
//   J2 about Y at the shoulder, offset (A1, 0, D1) from the base
//   J3 about Y at the elbow, A2 above the shoulder along the upper arm
//   J4 about X (forearm roll), axis lifted A3 above the elbow axis
//   J5 about Y at the wrist centre, D4 along the forearm
//   J6 about X at the flange, D6 beyond the wrist centre
// Tool frame: +X is the approach direction, +Y is the finger travel direction.

export const GEO = {
  D1: 0.400, // base mount to shoulder axis height
  A1: 0.150, // shoulder axis forward offset
  A2: 0.560, // upper arm length
  A3: 0.035, // elbow-to-forearm-axis offset
  D4: 0.515, // forearm length to wrist centre
  D6: 0.080, // wrist centre to flange
  TOOL: 0.165, // flange to TCP (gripper)
};

const D = Math.PI / 180;
export const DEG = D;

// Joint limits [min, max] in radians and max speeds in rad/s (typical 10 kg class arm).
export const LIMITS = [
  [-185 * D, 185 * D],
  [-80 * D, 140 * D],
  [-120 * D, 80 * D],
  [-350 * D, 350 * D],
  [-125 * D, 125 * D],
  [-350 * D, 350 * D],
];
export const VMAX = [220 * D, 210 * D, 230 * D, 360 * D, 380 * D, 520 * D];

export const HOME = [0, 0, 0, 0, 90 * D, 0];

// ---------- small 3x3 / vector helpers (row-major arrays of 9) ----------
export const rotX = (t) => { const c = Math.cos(t), s = Math.sin(t); return [1, 0, 0, 0, c, -s, 0, s, c]; };
export const rotY = (t) => { const c = Math.cos(t), s = Math.sin(t); return [c, 0, s, 0, 1, 0, -s, 0, c]; };
export const rotZ = (t) => { const c = Math.cos(t), s = Math.sin(t); return [c, -s, 0, s, c, 0, 0, 0, 1]; };

export function mul(A, B) {
  const R = new Array(9);
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      R[i * 3 + j] = A[i * 3] * B[j] + A[i * 3 + 1] * B[3 + j] + A[i * 3 + 2] * B[6 + j];
    }
  }
  return R;
}
export const mulv = (A, v) => [
  A[0] * v[0] + A[1] * v[1] + A[2] * v[2],
  A[3] * v[0] + A[4] * v[1] + A[5] * v[2],
  A[6] * v[0] + A[7] * v[1] + A[8] * v[2],
];
export const transpose = (A) => [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];

// ---------- forward kinematics ----------
// Returns TCP position p, tool rotation R, and the wrist centre / elbow / shoulder points.
export function fk(q, tool = GEO.TOOL) {
  const { D1, A1, A2, A3, D4, D6 } = GEO;
  const R1 = rotZ(q[0]);
  const shoulder = mulv(R1, [A1, 0, D1]);
  const R2 = mul(R1, rotY(q[1]));
  const elbow = add(shoulder, mulv(R2, [0, 0, A2]));
  const R3 = mul(R2, rotY(q[2]));
  const wrist = add(elbow, mulv(R3, [D4, 0, A3]));
  const R5 = mul(mul(R3, rotX(q[3])), rotY(q[4]));
  const R6 = mul(R5, rotX(q[5]));
  const x = [R6[0], R6[3], R6[6]];
  const flange = add(wrist, scale(x, D6));
  const p = add(flange, scale(x, tool));
  return { p, R: R6, wrist, elbow, shoulder, flange };
}

// ---------- inverse kinematics ----------
const TWO_PI = Math.PI * 2;
const wrapPi = (a) => a - TWO_PI * Math.floor((a + Math.PI) / TWO_PI);

// Pick the 2πk-equivalent of `a` that lies within limits and is closest to `ref`.
function fitJoint(a, i, ref) {
  const [lo, hi] = LIMITS[i];
  let best = null;
  let bestD = Infinity;
  const base = wrapPi(a);
  for (let k = -2; k <= 2; k++) {
    const v = base + k * TWO_PI;
    if (v < lo - 1e-9 || v > hi + 1e-9) continue;
    const d = Math.abs(v - ref);
    if (d < bestD) { bestD = d; best = v; }
  }
  return best;
}

const WEIGHTS = [1.4, 1.2, 1, 0.6, 0.6, 0.35];

/**
 * Analytic IK. Returns every in-limit solution (up to 8) sorted by weighted distance to qRef.
 * p: TCP position, R: tool rotation (row-major 3x3), qRef: current joints (used to pick
 * the nearest branch and to resolve the J1 / wrist singularities).
 */
export function ikAll(p, R, qRef = HOME, tool = GEO.TOOL) {
  const { D1, A1, A2, A3, D4, D6 } = GEO;
  const x = [R[0], R[3], R[6]];
  const w = add(p, scale(x, -(D6 + tool)));
  const L3 = Math.hypot(D4, A3);
  const phi = Math.atan2(D4, A3);
  const rh = Math.hypot(w[0], w[1]);
  const sols = [];

  const q1Opts = rh < 1e-6
    ? [[qRef[0], 0], [qRef[0] + Math.PI, 0]]
    : [[Math.atan2(w[1], w[0]), rh], [Math.atan2(w[1], w[0]) + Math.PI, -rh]];

  for (const [q1, r] of q1Opts) {
    const u = r - A1;
    const v = w[2] - D1;
    const fz = (u * u + v * v - A2 * A2 - L3 * L3) / (2 * A2);
    const c = fz / L3;
    if (c < -1 - 1e-9 || c > 1 + 1e-9) continue;
    const ac = Math.acos(Math.max(-1, Math.min(1, c)));
    for (const sgn of [1, -1]) {
      const q3 = sgn * ac - phi;
      const fx = D4 * Math.cos(q3) + A3 * Math.sin(q3);
      const gz = A2 + fz;
      const q2 = Math.atan2(u, v) - Math.atan2(fx, gz);
      // Wrist: R36 = Rx(q4) Ry(q5) Rx(q6)
      const R03 = mul(rotZ(q1), rotY(q2 + q3));
      const M = mul(transpose(R03), R);
      const cb = Math.max(-1, Math.min(1, M[0]));
      const b = Math.acos(cb);
      const wristSets = [];
      if (Math.abs(Math.sin(b)) < 1e-6) {
        // Wrist singular (J5 ≈ 0): keep J4 where it is, put the rest on J6.
        const q4 = qRef[3];
        const sum = Math.atan2(M[7], M[4]);
        wristSets.push([q4, cb > 0 ? 0 : Math.PI, cb > 0 ? sum - q4 : -sum + q4]);
      } else {
        const a = Math.atan2(M[3], -M[6]);
        const cc = Math.atan2(M[1], M[2]);
        wristSets.push([a, b, cc]);
        wristSets.push([a + Math.PI, -b, cc + Math.PI]);
      }
      for (const [q4, q5, q6] of wristSets) {
        const raw = [q1, q2, q3, q4, q5, q6];
        const sol = [];
        let ok = true;
        for (let i = 0; i < 6; i++) {
          const f = fitJoint(raw[i], i, qRef[i]);
          if (f === null) { ok = false; break; }
          sol.push(f);
        }
        if (ok) sols.push(sol);
      }
    }
  }
  const cost = (s) => s.reduce((acc, v, i) => acc + WEIGHTS[i] * Math.abs(v - qRef[i]), 0);
  sols.sort((a, b) => cost(a) - cost(b));
  return sols;
}

export function ik(p, R, qRef = HOME, tool = GEO.TOOL) {
  const s = ikAll(p, R, qRef, tool);
  return s.length ? s[0] : null;
}

// ---------- orientation helpers ----------
// KUKA-style A/B/C (Z-Y'-X'' Euler) in radians.
export function toABC(R) {
  const A = Math.atan2(R[3], R[0]);
  const B = Math.asin(Math.max(-1, Math.min(1, -R[6])));
  const C = Math.atan2(R[7], R[8]);
  return [A, B, C];
}
export const fromABC = (A, B, C) => mul(mul(rotZ(A), rotY(B)), rotX(C));

// Tool pointing straight down, fingers rotated by yaw about the vertical.
export const toolDown = (yaw = 0) => mul(rotZ(yaw), rotY(Math.PI / 2));

// Rotation about an arbitrary unit axis.
export function rotAxis(axis, t) {
  const [x, y, z] = axis;
  const c = Math.cos(t), s = Math.sin(t), C = 1 - c;
  return [
    c + x * x * C, x * y * C - z * s, x * z * C + y * s,
    y * x * C + z * s, c + y * y * C, y * z * C - x * s,
    z * x * C - y * s, z * y * C + x * s, c + z * z * C,
  ];
}

// Quaternion (x,y,z,w) <-> rotation matrix, for LIN interpolation.
export function matToQuat(m) {
  const tr = m[0] + m[4] + m[8];
  let x, y, z, w;
  if (tr > 0) {
    const s = 0.5 / Math.sqrt(tr + 1);
    w = 0.25 / s; x = (m[7] - m[5]) * s; y = (m[2] - m[6]) * s; z = (m[3] - m[1]) * s;
  } else if (m[0] > m[4] && m[0] > m[8]) {
    const s = 2 * Math.sqrt(1 + m[0] - m[4] - m[8]);
    w = (m[7] - m[5]) / s; x = 0.25 * s; y = (m[1] + m[3]) / s; z = (m[2] + m[6]) / s;
  } else if (m[4] > m[8]) {
    const s = 2 * Math.sqrt(1 + m[4] - m[0] - m[8]);
    w = (m[2] - m[6]) / s; x = (m[1] + m[3]) / s; y = 0.25 * s; z = (m[5] + m[7]) / s;
  } else {
    const s = 2 * Math.sqrt(1 + m[8] - m[0] - m[4]);
    w = (m[3] - m[1]) / s; x = (m[2] + m[6]) / s; y = (m[5] + m[7]) / s; z = 0.25 * s;
  }
  return [x, y, z, w];
}
export function quatToMat([x, y, z, w]) {
  return [
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y),
  ];
}
export function slerp(a, b, t) {
  let [bx, by, bz, bw] = b;
  let d = a[0] * bx + a[1] * by + a[2] * bz + a[3] * bw;
  if (d < 0) { d = -d; bx = -bx; by = -by; bz = -bz; bw = -bw; }
  if (d > 0.9995) {
    const r = [a[0] + t * (bx - a[0]), a[1] + t * (by - a[1]), a[2] + t * (bz - a[2]), a[3] + t * (bw - a[3])];
    const n = Math.hypot(...r);
    return r.map((v) => v / n);
  }
  const th = Math.acos(d);
  const s = Math.sin(th);
  const wa = Math.sin((1 - t) * th) / s;
  const wb = Math.sin(t * th) / s;
  return [a[0] * wa + bx * wb, a[1] * wa + by * wb, a[2] * wa + bz * wb, a[3] * wa + bw * wb];
}
export function rotAngle(Ra, Rb) {
  const M = mul(transpose(Ra), Rb);
  return Math.acos(Math.max(-1, Math.min(1, (M[0] + M[4] + M[8] - 1) / 2)));
}
export const inLimits = (q) => q.every((v, i) => v >= LIMITS[i][0] - 1e-9 && v <= LIMITS[i][1] + 1e-9);
