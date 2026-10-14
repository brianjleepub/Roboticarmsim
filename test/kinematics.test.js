import { test } from 'node:test';
import assert from 'node:assert/strict';
import { fk, ik, ikAll, LIMITS, HOME, toolDown, toABC, fromABC, matToQuat, quatToMat, DEG } from '../src/kinematics.js';

const close = (a, b, eps = 1e-6) => a.every((v, i) => Math.abs(v - b[i]) < eps);

function randomQ(rng) {
  return LIMITS.map(([lo, hi]) => {
    // keep away from the extreme ends so wrap-around choices stay unambiguous
    const m = (hi - lo) * 0.05;
    return lo + m + rng() * (hi - lo - 2 * m);
  });
}

function mulberry32(a) {
  return () => {
    a |= 0; a = (a + 0x6d2b79f5) | 0;
    let t = Math.imul(a ^ (a >>> 15), 1 | a);
    t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

test('home pose puts the tool straight down in front of the robot', () => {
  const { p, R } = fk(HOME);
  assert.ok(close(p, [0.665, 0, 0.75], 1e-9), `p=${p}`);
  assert.ok(close(R, toolDown(0), 1e-9));
});

test('IK reproduces FK for random in-limit poses', () => {
  const rng = mulberry32(42);
  let checked = 0;
  for (let n = 0; n < 2000; n++) {
    const q = randomQ(rng);
    if (Math.abs(Math.sin(q[4])) < 0.02) continue; // skip exact wrist singularity
    const { p, R } = fk(q);
    const sols = ikAll(p, R, q);
    assert.ok(sols.length > 0, `no solution for q=${q.map((v) => (v / DEG).toFixed(1))}`);
    for (const s of sols) {
      const f = fk(s);
      assert.ok(close(f.p, p, 1e-7), 'position mismatch');
      assert.ok(close(f.R, R, 1e-7), 'rotation mismatch');
    }
    // the nearest solution must be the original configuration
    assert.ok(close(sols[0], q, 1e-6), `nearest branch differs: ${sols[0]} vs ${q}`);
    checked++;
  }
  assert.ok(checked > 1800);
});

test('IK handles the wrist singularity (J5 = 0)', () => {
  const q = [0.3, 0.2, -0.4, 0.5, 0, 0.7];
  const { p, R } = fk(q);
  const s = ik(p, R, q);
  assert.ok(s);
  const f = fk(s);
  assert.ok(close(f.p, p, 1e-7) && close(f.R, R, 1e-7));
});

test('unreachable targets return null', () => {
  assert.equal(ik([3, 0, 0.5], toolDown(0), HOME), null);
});

test('table pick poses are reachable with the tool pointing down', () => {
  for (const [x, y] of [[0.45, -0.5], [0.95, 0], [0.6, 0.55], [0.35, 0.3]]) {
    const s = ik([x, y, 0.02], toolDown(0.3), HOME);
    assert.ok(s, `unreachable ${x},${y}`);
  }
});

test('ABC and quaternion round trips', () => {
  const R = fromABC(0.4, -0.3, 1.2);
  assert.ok(close(toABC(R), [0.4, -0.3, 1.2], 1e-9));
  assert.ok(close(quatToMat(matToQuat(R)), R, 1e-9));
});
