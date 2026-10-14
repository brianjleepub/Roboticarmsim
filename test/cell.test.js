import { test } from 'node:test';
import assert from 'node:assert/strict';
import { Controller, Zone } from '../src/motion.js';
import { ROBOT_BASES, TRAY_DEFS, CONVEYOR } from '../src/scene.js';
import { toolDown, HOME } from '../src/kinematics.js';

// Minimal stand-in for the three.js robot model: the controller only needs these hooks.
const fakeRobot = () => ({
  root: { position: { set() {} }, rotation: { set() {} } },
  tcp: {},
  setJoints() {},
  setGap() {},
});
const make = (i) => new Controller({ robot: fakeRobot(), parts: {}, base: ROBOT_BASES[i], name: `Robot ${i + 1}` });
const close = (a, b, eps = 1e-9) => a.every((v, i) => Math.abs(v - b[i]) < eps);

test('cell <-> base frame round trip for the rotated second robot', () => {
  const c = make(1);
  const p = [0.9, -0.3, 0.2];
  assert.ok(close(c.toCellP(c.toLocalP(p)), p));
  // robot 2 home TCP sits in front of it, i.e. toward robot 1 (smaller x)
  const home = c.pose(HOME).p;
  assert.ok(close(home, [ROBOT_BASES[1].x - 0.665, 0, 0.75], 1e-9));
});

test('IK in cell coordinates reproduces the requested pose for both robots', () => {
  for (const i of [0, 1]) {
    const c = make(i);
    const R = toolDown(0.4);
    const q = c.ikCell([0.88, 0.05, 0.1], R);
    assert.ok(q, `robot ${i + 1} cannot reach`);
    const f = c.pose(q);
    assert.ok(close(f.p, [0.88, 0.05, 0.1], 1e-7) && close(f.R, R, 1e-7));
  }
});

test('layout: both arms reach the transfer tray, robot 2 reaches the conveyor infeed', () => {
  const [c1, c2] = [make(0), make(1)];
  const t = TRAY_DEFS.find((d) => d.id === 'transfer');
  for (const dx of [-0.07, 0.07]) for (const dy of [-0.07, 0.07]) {
    assert.ok(c1.canReach(t.x + dx, t.y + dy, 0.03), 'robot 1 -> transfer slot');
    assert.ok(c2.canReach(t.x + dx, t.y + dy, 0.03), 'robot 2 -> transfer slot');
  }
  const [ix, iy] = CONVEYOR.infeed;
  assert.ok(c2.canReach(ix, iy, CONVEYOR.z + 0.03, -Math.PI / 2));
});

test('workspace guard flags the conveyor belt', () => {
  const c2 = make(1);
  const [ix, iy] = CONVEYOR.infeed;
  const low = c2.ikCell([ix, iy, CONVEYOR.z - 0.03], toolDown(0));
  assert.ok(low);
  assert.equal(c2.collision(low), 'conveyor');
  const above = c2.ikCell([ix, iy, CONVEYOR.z + 0.05], toolDown(0));
  assert.equal(c2.collision(above), null);
});

test('zone interlock lets only one controller in at a time', () => {
  const [a, b] = [make(0), make(1)];
  const z = new Zone('transfer tray');
  a.active = { type: 'acquire', zone: z, t: 0 };
  assert.equal(a.stepCommand(0.016), true);
  b.active = { type: 'acquire', zone: z, t: 0 };
  assert.equal(b.stepCommand(0.016), false, 'b must wait');
  a.active = { type: 'release', zone: z, t: 0 };
  a.stepCommand(0.016);
  assert.equal(b.stepCommand(0.016), true, 'b gets in once a releases');
  b.stop();
  assert.equal(z.owner, null, 'stopping frees the zone');
});
