import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { createRenderer, createWorld, RISER_H, TABLE, ROBOT_BASES, CONVEYOR, FLOOR_Z } from './scene.js';
import { RobotModel, makeMaterials, GRIPPER_MAX_GAP } from './robot.js';
import { Parts } from './parts.js';
import { Controller, Zone } from './motion.js';
import { Line } from './line.js';
import { fk, LIMITS, DEG, HOME, toolDown, toABC } from './kinematics.js';

const $ = (id) => document.getElementById(id);

// ---------------------------------------------------------------- setup
const canvas = $('view');
const renderer = createRenderer(canvas);
const world = createWorld(renderer);
const { scene, cell } = world;

const parts = new Parts(cell);
const materials = makeMaterials();
const ctls = ROBOT_BASES.map((base, i) => {
  const robot = new RobotModel(materials);
  cell.add(robot.root);
  return new Controller({ robot, parts, base, name: `Robot ${i + 1}` });
});
const transferZone = new Zone('transfer tray');
const line = new Line({ feeder: ctls[0], unloader: ctls[1], parts, zone: transferZone });
let active = 0;
const ctl = () => ctls[active];

// marker ring on the floor around the selected robot's riser
const selRing = new THREE.Mesh(
  new THREE.RingGeometry(0.4, 0.44, 64),
  new THREE.MeshBasicMaterial({ color: 0xf07a12, transparent: true, opacity: 0.85, depthWrite: false }),
);
selRing.position.z = FLOOR_Z + 0.004;
cell.add(selRing);

const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 60);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.09;
controls.minDistance = 0.7;
controls.maxDistance = 9;
controls.maxPolarAngle = Math.PI * 0.495;
controls.rotateSpeed = 0.7;
controls.zoomSpeed = 0.9;
controls.touches = { ONE: THREE.TOUCH.ROTATE, TWO: THREE.TOUCH.DOLLY_PAN };

// cell (x, y, z) -> world (x, z + RISER_H, -y)
const HOME_TARGET = new THREE.Vector3(0.85, RISER_H + 0.1, 0.35);
function viewFit(animate = true) {
  const portrait = innerHeight > innerWidth;
  const dist = portrait ? 6.1 : 4.3;
  const dir = new THREE.Vector3(0.22, 0.7, 1.0).normalize();
  setView(HOME_TARGET.clone(), HOME_TARGET.clone().addScaledVector(dir, dist), animate);
}
function viewTop() {
  const t = new THREE.Vector3(0.85, RISER_H, 0.35);
  setView(t, t.clone().add(new THREE.Vector3(0.001, 3.6, 0.0)), true);
}
let camAnim = null;
function setView(target, pos, animate) {
  if (!animate) {
    controls.target.copy(target);
    camera.position.copy(pos);
    controls.update();
    return;
  }
  camAnim = { t: 0, t0: controls.target.clone(), p0: camera.position.clone(), t1: target, p1: pos };
}

// ---------------------------------------------------------------- IK target gizmo
const target = new THREE.Group();
target.visible = false;
cell.add(target);
const tgtMat = new THREE.MeshStandardMaterial({ color: 0xff8a1f, emissive: 0xff6a00, emissiveIntensity: 0.6, roughness: 0.3, transparent: true, opacity: 0.85, depthTest: false });
const tgtBall = new THREE.Mesh(new THREE.SphereGeometry(0.034, 32, 16), tgtMat);
tgtBall.renderOrder = 10; // always drawn on top so it can be found and grabbed
target.add(tgtBall);
const tgtHit = new THREE.Mesh(new THREE.SphereGeometry(0.11, 12, 8), new THREE.MeshBasicMaterial({ visible: false }));
target.add(tgtHit);
// approach arrow (tool +X)
const arrow = new THREE.ArrowHelper(new THREE.Vector3(1, 0, 0), new THREE.Vector3(), 0.13, 0xffd08a, 0.035, 0.02);
for (const o of [arrow.line, arrow.cone]) { o.material.depthTest = false; o.renderOrder = 11; }
target.add(arrow);
const ringMat = new THREE.MeshBasicMaterial({ color: 0xff8a1f, transparent: true, opacity: 0.55, depthWrite: false });
const tgtRing = new THREE.Mesh(new THREE.RingGeometry(0.035, 0.045, 40), ringMat);
cell.add(tgtRing);
tgtRing.visible = false;
const dropLineGeo = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]);
const dropLine = new THREE.Line(dropLineGeo, new THREE.LineDashedMaterial({ color: 0xff8a1f, dashSize: 0.02, gapSize: 0.015, transparent: true, opacity: 0.8 }));
cell.add(dropLine);
dropLine.visible = false;

// target pose in the cell frame
const tgt = { p: [0.665, 0, 0.75], R: toolDown(0), orient: 'down', plane: 'h', live: true };

function setTargetFromTool() {
  const f = ctl().pose();
  tgt.p = f.p.slice();
  if (tgt.orient === 'down') {
    // derive yaw from the current finger axis so the wrist does not spin
    tgt.R = toolDown(Math.atan2(f.R[4], f.R[1]) - Math.PI / 2);
  } else {
    tgt.R = f.R;
  }
  syncTarget();
}

function syncTarget() {
  target.position.set(...tgt.p);
  const m = new THREE.Matrix4().set(
    tgt.R[0], tgt.R[1], tgt.R[2], 0, tgt.R[3], tgt.R[4], tgt.R[5], 0, tgt.R[6], tgt.R[7], tgt.R[8], 0, 0, 0, 0, 1,
  );
  target.quaternion.setFromRotationMatrix(m);
  const ok = !!ctl().ikCell(tgt.p, tgt.R);
  tgtMat.color.set(ok ? 0xff8a1f : 0xff3b30);
  tgtMat.emissive.set(ok ? 0xff6a00 : 0xd01000);
  const onTable = tgt.p[0] > TABLE.x0 && tgt.p[0] < TABLE.x1 && tgt.p[1] > TABLE.y0 && tgt.p[1] < TABLE.y1;
  const groundZ = onTable ? 0.001 : -RISER_H + 0.002;
  tgtRing.position.set(tgt.p[0], tgt.p[1], groundZ + 0.001);
  const pos = dropLineGeo.attributes.position;
  pos.setXYZ(0, tgt.p[0], tgt.p[1], tgt.p[2]);
  pos.setXYZ(1, tgt.p[0], tgt.p[1], groundZ);
  pos.needsUpdate = true;
  dropLine.computeLineDistances();
  $('tgtXYZ').textContent = `X ${mm(tgt.p[0])}  Y ${mm(tgt.p[1])}  Z ${mm(tgt.p[2])} mm`;
  const reach = $('tgtReach');
  reach.textContent = ok ? `reachable by R${active + 1}` : `out of reach of R${active + 1}`;
  reach.className = ok ? '' : 'bad';
  if (tgt.live && ui.tab === 'p-ik') ctl().follow = { p: tgt.p, R: tgt.R };
  dirty = true;
}
const mm = (v) => (v * 1000).toFixed(0);

// ---------------------------------------------------------------- UI state
const ui = { tab: 'p-jog', frame: 'joint', selected: null, follow: false };
let dirty = true;

function toast(text) {
  const t = $('toast');
  t.textContent = text;
  t.classList.add('show');
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => t.classList.remove('show'), 2400);
}
function setStatus(text, level) {
  $('statusText').textContent = text;
  $('status').dataset.level = level;
}

const anyEstop = () => ctls.some((c) => c.estop);
const anyBusy = () => ctls.some((c) => c.busy);

ctls.forEach((c, i) => {
  c.on('status', ({ text, level }) => setStatus(text, level));
  c.on('grip', () => renderPartList());
  c.on('stopped', () => {
    if (progs[i].running) { progs[i].running = false; progs[i].cur = -1; }
    if (i === active) { updateProgButtons(); renderProgram(); }
  });
});
line.on('state', (running, reason) => {
  $('lineBtn').textContent = running ? 'Stop line' : 'Run line';
  $('lineBtn').classList.toggle('primary', !running);
  $('lineBtn').classList.toggle('danger', running);
  if (reason) setStatus(reason, /stopped/i.test(reason) ? 'error' : 'ok');
  else if (running) setStatus('Line running: robot 1 feeds, robot 2 unloads to the conveyor', 'info');
  renderPartList();
  updateMode();
});

// tabs
for (const b of document.querySelectorAll('.tabs button')) {
  b.addEventListener('click', () => selectTab(b.dataset.pane));
}
function selectTab(pane) {
  ui.tab = pane;
  for (const b of document.querySelectorAll('.tabs button')) b.setAttribute('aria-selected', String(b.dataset.pane === pane));
  for (const p of document.querySelectorAll('.pane')) p.hidden = p.id !== pane;
  const ikTab = pane === 'p-ik';
  target.visible = tgtRing.visible = dropLine.visible = ikTab;
  if (ikTab) {
    if (!ctl().busy) setTargetFromTool();
  } else {
    for (const c of ctls) c.follow = null;
  }
  if (pane !== 'p-pick') selectPart(null);
  $('sheet').classList.remove('min');
  layout();
  updateReadouts();
  dirty = true;
}

// robot selector
for (const b of $('robotSel').querySelectorAll('button')) {
  b.addEventListener('click', () => selectRobot(+b.dataset.r));
}
function selectRobot(i) {
  if (i === active) return;
  ctl().jog = null;
  ctl().follow = null;
  active = i;
  for (const x of $('robotSel').querySelectorAll('button')) x.setAttribute('aria-pressed', String(+x.dataset.r === i));
  const b = ROBOT_BASES[i];
  selRing.position.x = b.x;
  selRing.position.y = b.y;
  $('hRobot').textContent = `Robot ${i + 1}`;
  if (ui.tab === 'p-ik' && !ctl().busy) setTargetFromTool();
  if (ui.follow) setFollowCam(true);
  updateProgButtons();
  renderProgram();
  updateReadouts();
  dirty = true;
}

// sheet collapse
$('handle').addEventListener('click', () => { $('sheet').classList.toggle('min'); setTimeout(layout, 260); layout(); });

// speed override applies to both arms
$('override').addEventListener('input', (e) => {
  for (const c of ctls) c.override = e.target.value / 100;
  $('ovrOut').textContent = `${e.target.value}%`;
});

// E-stop halts both arms and the line
$('estop').addEventListener('click', () => {
  const on = !anyEstop();
  if (on) line.halt('Emergency stop');
  for (const c of ctls) c.setEstop(on);
  $('estop').setAttribute('aria-pressed', String(on));
  $('estop').textContent = on ? 'RESET' : 'E-STOP';
  updateMode();
});

// camera buttons
$('vFit').addEventListener('click', () => { setFollowCam(false); viewFit(true); });
$('vTop').addEventListener('click', () => { setFollowCam(false); viewTop(); });
$('vTool').addEventListener('click', () => setFollowCam(!ui.follow));
function setFollowCam(on) {
  ui.follow = on;
  $('vTool').setAttribute('aria-pressed', String(on));
  if (on) {
    const tcp = ctl().robot.tcp.getWorldPosition(new THREE.Vector3());
    const offset = camera.position.clone().sub(controls.target).setLength(1.1);
    setView(tcp, tcp.clone().add(offset), true);
  }
}

// ---------------------------------------------------------------- jog panel
const JOINT_NAMES = ['Base', 'Shoulder', 'Elbow', 'Wrist roll', 'Wrist bend', 'Flange'];
const CART_NAMES = ['X', 'Y', 'Z', 'A', 'B', 'C'];
const CART_SUB = ['mm', 'mm', 'mm', 'about Z', 'about Y', 'about X'];
const jogVals = [];
const jogBars = [];
function buildJogRows() {
  const rows = $('jogRows');
  rows.innerHTML = '';
  jogVals.length = 0;
  jogBars.length = 0;
  for (let i = 0; i < 6; i++) {
    const r = document.createElement('div');
    r.className = 'jr';
    const joint = ui.frame === 'joint';
    const name = joint ? `A${i + 1}` : CART_NAMES[i];
    const sub = joint ? JOINT_NAMES[i] : CART_SUB[i];
    r.innerHTML = `<div class="lab">${name}<small>${sub}</small></div>
      <div class="val"><span>0.0</span>${joint ? '<div class="bar"><i></i></div>' : ''}</div>
      <button class="hold" aria-label="${name} minus" data-axis="${i}" data-dir="-1">−</button>
      <button class="hold" aria-label="${name} plus" data-axis="${i}" data-dir="1">+</button>`;
    rows.appendChild(r);
    jogVals.push(r.querySelector('.val span'));
    jogBars.push(r.querySelector('.bar'));
  }
  for (const b of rows.querySelectorAll('.hold')) bindHold(b);
  updateReadouts();
}
function bindHold(b) {
  let jogger = null;
  const start = (e) => {
    e.preventDefault();
    const c = ctl();
    if (c.estop) { toast('Release the E-stop first.'); return; }
    if (line.running) { toast('Stop the line before jogging.'); return; }
    if (c.busy) { toast('Stop the running motion before jogging.'); return; }
    b.setPointerCapture?.(e.pointerId);
    b.classList.add('on');
    jogger = c;
    c.follow = null;
    c.jog = { frame: ui.frame, axis: +b.dataset.axis, dir: +b.dataset.dir };
    const axisName = ui.frame === 'joint' ? 'A' + (+b.dataset.axis + 1) : CART_NAMES[+b.dataset.axis];
    c.setStatus(`${c.name}: jogging ${axisName} (${ui.frame === 'joint' ? 'axes' : ui.frame})`, 'info');
  };
  const end = () => {
    if (!b.classList.contains('on')) return;
    b.classList.remove('on');
    if (jogger) { jogger.jog = null; jogger.setStatus('Ready', 'ok'); }
    jogger = null;
  };
  b.addEventListener('pointerdown', start);
  b.addEventListener('pointerup', end);
  b.addEventListener('pointercancel', end);
  b.addEventListener('lostpointercapture', end);
  b.addEventListener('contextmenu', (e) => e.preventDefault());
}
for (const b of $('jogFrame').querySelectorAll('button')) {
  b.addEventListener('click', () => {
    ui.frame = b.dataset.frame;
    for (const x of $('jogFrame').querySelectorAll('button')) x.setAttribute('aria-pressed', String(x === b));
    buildJogRows();
  });
}
$('homeBtn').addEventListener('click', () => {
  if (ctl().estop) return toast('Release the E-stop first.');
  ctl().follow = null;
  ctl().goHome();
});
const toggleGrip = () => {
  const c = ctl();
  if (c.estop) return toast('Release the E-stop first.');
  c.gripper(c.gap > GRIPPER_MAX_GAP / 2 && !c.held);
};
$('gripBtn').addEventListener('click', toggleGrip);
$('tgtGrip').addEventListener('click', toggleGrip);

// ---------------------------------------------------------------- target panel
function bindSeg(id, key, fn) {
  for (const b of $(id).querySelectorAll('button')) {
    b.addEventListener('click', () => {
      for (const x of $(id).querySelectorAll('button')) x.setAttribute('aria-pressed', String(x === b));
      fn(b.dataset[key]);
    });
  }
}
bindSeg('dragPlane', 'plane', (v) => { tgt.plane = v; });
bindSeg('tgtOrient', 'orient', (v) => {
  tgt.orient = v;
  setTargetFromTool();
});
bindSeg('followMode', 'follow', (v) => {
  tgt.live = v === 'live';
  $('moveBtns').hidden = tgt.live;
  ctl().follow = tgt.live ? { p: tgt.p, R: tgt.R } : null;
  syncTarget();
});
$('tgtSnap').addEventListener('click', () => { ctl().follow = null; setTargetFromTool(); });
for (const [id, type] of [['movePtp', 'ptp'], ['moveLin', 'lin']]) {
  $(id).addEventListener('click', () => {
    const c = ctl();
    if (c.estop) return toast('Release the E-stop first.');
    c.enqueue({
      type, pose: { p: tgt.p.slice(), R: tgt.R }, label: 'Target',
      onStart: () => c.setStatus(`${c.name}: ${type.toUpperCase()} to target`, 'info'),
      onDone: () => c.setStatus(`${c.name}: at target`, 'ok'),
    });
  });
}

// ---------------------------------------------------------------- pick panel
function partWhere(p) {
  const w = parts.whereIs(p);
  if (w.kind === 'held') return `in ${ctls.find((c) => c.held === p)?.name.toLowerCase() || 'gripper'}`;
  if (w.kind === 'tray') return w.tray.shared ? 'in transfer tray' : `in ${w.tray.label.toLowerCase()} tray`;
  return { bin: 'in the bin', conveyor: 'on the conveyor', moving: 'moving', floor: 'on the floor', table: 'on table' }[w.kind];
}
function renderPartList() {
  const list = $('partList');
  list.innerHTML = '';
  for (const p of parts.list) {
    const b = document.createElement('button');
    b.className = 'part';
    b.setAttribute('aria-pressed', String(ui.selected === p.id));
    b.innerHTML = `<i class="c-${p.color} ${p.kind}"></i><span><b>${p.name}</b><small>${partWhere(p)}</small></span>`;
    b.addEventListener('click', () => selectPart(ui.selected === p.id ? null : p.id));
    list.appendChild(b);
  }
  // line counters
  const count = (k) => parts.list.filter((p) => parts.whereIs(p).kind === k).length;
  const inTransfer = parts.list.filter((p) => parts.whereIs(p).tray?.shared).length;
  $('lineStats').innerHTML = `<span>Table <b>${count('table')}</b></span><span>Transfer <b>${inTransfer}</b></span><span>Belt <b>${count('conveyor')}</b></span><span>Bin <b>${count('bin')}</b></span>`;
}
function selectPart(id) {
  ui.selected = id;
  parts.setHighlight(id);
  const p = id && parts.byId(id);
  $('pickHint').innerHTML = p
    ? `<b>${p.name}</b> selected. Tap a tray, the conveyor or a free spot on the table.`
    : '<b>1.</b> Tap a part. <b>2.</b> Tap a tray, the conveyor or a spot on the table. Out of one arm\'s reach, the arms hand it over through the transfer tray.';
  renderPartList();
  dirty = true;
}

// Plan a move for the selected part. One arm does it if it can reach both ends; otherwise the
// arm that can pick sets it in the transfer tray and the other arm finishes the move.
function placeSelected(dest) {
  const p = parts.byId(ui.selected);
  if (!p) return;
  if (anyEstop()) return toast('Release the E-stop first.');
  if (line.running) return toast('Stop the line first.');
  if (p.state !== 'rest') return toast('That part is not on a surface.');
  const support = parts.support(dest.x, dest.y, p.r, p);
  const zPlace = support + p.h - 0.02;
  const yaw = dest.yaw ?? 0;
  const canPick = (c) => { const g = c.graspPose(p); return c.canReach(g.x, g.y, g.zGrip, g.yaw) && c.canReach(g.x, g.y, g.top + 0.1, g.yaw); };
  const canPlace = (c, x, y, z) => c.canReach(x, y, z, yaw) && c.canReach(x, y, z + 0.12, yaw);
  const inTransfer = parts.whereIs(p).tray?.shared;
  const toTransfer = dest.trayId === 'transfer';
  const order = [ctl(), ...ctls.filter((c) => c !== ctl())];
  const solo = order.find((c) => canPick(c) && canPlace(c, dest.x, dest.y, zPlace));
  const zone = (cond) => (cond ? transferZone : null);
  const done = () => { setStatus('Move finished', 'ok'); renderPartList(); };

  if (solo) {
    if (solo.busy) return toast(`${solo.name} is busy. Wait for it to finish.`);
    solo.follow = null;
    solo.pickAndPlace(p, dest, { home: true, pickZone: zone(inTransfer), placeZone: zone(toTransfer) });
    solo.enqueue({ type: 'call', fn: done });
    selectPart(null);
    return;
  }
  const picker = order.find((c) => canPick(c));
  if (!picker) return toast('Neither arm can reach that part.');
  const placer = ctls.find((c) => c !== picker && canPlace(c, dest.x, dest.y, zPlace));
  if (!placer) return toast('Neither arm can reach that spot.');
  if (picker.busy || placer.busy) return toast('Wait for both arms to finish.');
  const slot = parts.freeSlot('transfer');
  if (!slot) return toast('The transfer tray is full.');
  picker.follow = placer.follow = null;
  setStatus(`Handing over: ${picker.name} → transfer tray → ${placer.name}`, 'info');
  picker.pickAndPlace(p, { x: slot[0], y: slot[1] }, { placeZone: transferZone, pickZone: zone(inTransfer) });
  picker.enqueue({
    type: 'call',
    fn: () => {
      if (p.state !== 'rest' || !parts.whereIs(p).tray?.shared) return;
      placer.pickAndPlace(p, dest, { home: true, pickZone: transferZone });
      placer.enqueue({ type: 'call', fn: done });
    },
  });
  selectPart(null);
}

$('lineBtn').addEventListener('click', () => {
  if (line.running) { line.halt('Line stopped by operator'); return; }
  if (anyEstop()) return toast('Release the E-stop first.');
  if (anyBusy()) return toast('Wait for the arms to finish.');
  for (const c of ctls) c.follow = null;
  selectPart(null);
  line.start();
});
$('sortBtn').addEventListener('click', () => {
  const c = ctls[0];
  if (anyEstop()) return toast('Release the E-stop first.');
  if (line.running) return toast('Stop the line first.');
  if (c.busy) return toast('Robot 1 is busy.');
  c.follow = null;
  selectPart(null);
  const next = () => {
    const p = parts.list.find((x) => {
      if (x.state !== 'rest') return false;
      const w = parts.whereIs(x);
      if (w.kind !== 'table' && !(w.kind === 'tray' && w.tray.id !== x.color)) return false;
      const g = c.graspPose(x);
      return c.canReach(g.x, g.y, g.zGrip, g.yaw);
    });
    renderPartList();
    if (!p) {
      c.goHome();
      c.enqueue({ type: 'call', fn: () => c.setStatus('Robot 1: all reachable parts sorted', 'ok') });
      return;
    }
    const [x, y] = parts.traySlot(p.color);
    c.pickAndPlace(p, { x, y }, { pickZone: parts.whereIs(p).tray?.shared ? transferZone : null });
    c.enqueue({ type: 'call', fn: next });
  };
  next();
});
$('resetParts').addEventListener('click', () => {
  line.halt();
  for (const c of ctls) {
    if (c.busy) c.stop();
    c.held = null;
    c.gap = GRIPPER_MAX_GAP;
    c.robot.setGap(c.gap);
  }
  parts.reset();
  renderPartList();
  setStatus('Parts reset', 'ok');
  dirty = true;
});
$('stopBtn').addEventListener('click', () => {
  line.halt('Stopped by operator');
  for (const c of ctls) c.stop();
  setStatus('Stopped by operator', 'warn');
});

// ---------------------------------------------------------------- program (teach), one per robot
const progs = ctls.map(() => ({ steps: [], running: false, loop: false, cur: -1, example: true }));
const prog = () => progs[active];
function poseText(c, q) {
  const f = c.pose(q);
  return `${mm(f.p[0])}, ${mm(f.p[1])}, ${mm(f.p[2])}`;
}
function renderProgram() {
  const pr = prog();
  const list = $('progList');
  list.innerHTML = '';
  if (!pr.steps.length) {
    list.innerHTML = '<div class="empty">No steps yet. Move the arm, then tap + PTP.</div>';
  }
  pr.steps.forEach((s, i) => {
    const row = document.createElement('div');
    row.className = 'step' + (pr.running && pr.cur === i ? ' cur' : '');
    const t = s.type === 'grip' ? (s.close ? 'grip' : 'open') : s.type;
    const label = { ptp: 'PTP', lin: 'LIN', grip: 'CLOSE', open: 'OPEN', wait: 'WAIT' }[t];
    const desc = s.type === 'wait' ? `${s.t.toFixed(1)} s` : s.q ? poseText(ctl(), s.q) : 'gripper';
    row.innerHTML = `<span class="n">${i + 1}</span><span class="t ${t}">${label}</span><span class="d">${desc}</span><button aria-label="Delete step ${i + 1}">×</button>`;
    row.querySelector('button').addEventListener('click', () => {
      if (pr.running) return;
      pr.steps.splice(i, 1);
      renderProgram();
    });
    list.appendChild(row);
  });
  $('progTag').textContent = pr.example ? `Robot ${active + 1} example program · edit freely` : `Robot ${active + 1} program`;
}
function addStep(s) {
  const pr = prog();
  if (pr.running) return toast('Stop the program to edit it.');
  pr.example = false;
  pr.steps.push(s);
  renderProgram();
  toast(`Step ${pr.steps.length} recorded for robot ${active + 1}`);
}
$('addPtp').addEventListener('click', () => addStep({ type: 'ptp', q: ctl().q.slice() }));
$('addLin').addEventListener('click', () => addStep({ type: 'lin', q: ctl().q.slice() }));
$('addClose').addEventListener('click', () => addStep({ type: 'grip', close: true }));
$('addOpen').addEventListener('click', () => addStep({ type: 'grip', close: false }));
$('addWait').addEventListener('click', () => addStep({ type: 'wait', t: 0.5 }));
$('clearProg').addEventListener('click', () => {
  const pr = prog();
  if (pr.running) return;
  pr.steps = [];
  pr.example = false;
  renderProgram();
});
$('loopProg').addEventListener('click', () => {
  const pr = prog();
  pr.loop = !pr.loop;
  updateProgButtons();
});
$('runProg').addEventListener('click', () => {
  const c = ctl();
  const pr = prog();
  if (pr.running) { c.stop(`${c.name}: program stopped`); return; }
  if (c.estop) return toast('Release the E-stop first.');
  if (line.running) return toast('Stop the line first.');
  if (!pr.steps.length) return toast('Record some steps first.');
  if (c.busy) return toast('Wait for the current move to finish.');
  c.follow = null;
  pr.running = true;
  updateProgButtons();
  c.setStatus(`${c.name}: program running`, 'info');
  queueProgram(active);
});
function queueProgram(i) {
  const c = ctls[i];
  const pr = progs[i];
  pr.steps.forEach((s, k) => {
    const mark = () => { pr.cur = k; if (i === active) renderProgram(); };
    if (s.type === 'ptp') c.enqueue({ type: 'ptp', q: s.q.slice(), onStart: mark });
    else if (s.type === 'lin') { const f = c.pose(s.q); c.enqueue({ type: 'lin', pose: { p: f.p, R: f.R }, onStart: mark }); }
    else if (s.type === 'grip') c.enqueue({ type: 'grip', close: s.close, onStart: mark });
    else if (s.type === 'wait') c.enqueue({ type: 'wait', T: s.t, onStart: mark });
  });
  c.enqueue({
    type: 'call',
    fn: () => {
      if (pr.running && pr.loop) { queueProgram(i); return; }
      pr.running = false;
      pr.cur = -1;
      if (i === active) { updateProgButtons(); renderProgram(); }
      c.setStatus(`${c.name}: program finished`, 'ok');
    },
  });
}
function updateProgButtons() {
  const pr = prog();
  $('runProg').textContent = pr.running ? 'Stop' : 'Run';
  $('loopProg').setAttribute('aria-pressed', String(pr.loop));
  $('loopProg').textContent = pr.loop ? 'Loop on' : 'Loop off';
  updateMode();
}
function updateMode() {
  $('mode').textContent = anyEstop() ? 'E-stop' : line.running || anyBusy() ? 'Auto' : 'Manual T1';
}
// Example programs: a demonstration path for each arm (no gripping).
{
  const ex = (c, x, y, z, yaw) => c.ikCell([x, y, z], toolDown(yaw), HOME);
  const path = (c, a, b, d, e) => [
    { type: 'ptp', q: HOME.slice() },
    { type: 'ptp', q: a }, { type: 'lin', q: b }, { type: 'wait', t: 0.4 }, { type: 'lin', q: a },
    { type: 'ptp', q: d }, { type: 'lin', q: e }, { type: 'wait', t: 0.4 }, { type: 'lin', q: d },
    { type: 'ptp', q: HOME.slice() },
  ].filter((s) => (s.type !== 'ptp' && s.type !== 'lin') || s.q);
  const [c1, c2] = ctls;
  progs[0].steps = path(c1, ex(c1, 0.42, 0.56, 0.2, 0), ex(c1, 0.42, 0.56, 0.09, 0), ex(c1, 0.6, -0.56, 0.2, 0.6), ex(c1, 0.6, -0.56, 0.09, 0.6));
  const [ix, iy] = CONVEYOR.infeed;
  progs[1].steps = path(c2, ex(c2, 0.88, 0, 0.2, Math.PI), ex(c2, 0.88, 0, 0.09, Math.PI), ex(c2, ix, iy, 0.25, -Math.PI / 2), ex(c2, ix, iy, 0.14, -Math.PI / 2));
}

// ---------------------------------------------------------------- readouts
function updateReadouts() {
  const c = ctl();
  const f = c.pose();
  const abc = toABC(f.R);
  $('hX').textContent = mm(f.p[0]);
  $('hY').textContent = mm(f.p[1]);
  $('hZ').textContent = mm(f.p[2]);
  $('hA').textContent = (abc[0] / DEG).toFixed(1);
  $('hB').textContent = (abc[1] / DEG).toFixed(1);
  $('hC').textContent = (abc[2] / DEG).toFixed(1);
  $('hGrip').textContent = c.held ? `holding ${c.held.name.toLowerCase()}` : c.gap < 0.002 ? 'closed' : c.gap > GRIPPER_MAX_GAP - 0.001 ? 'open' : `${(c.gap * 1000).toFixed(0)} mm`;
  const closeIt = c.gap > GRIPPER_MAX_GAP / 2 && !c.held;
  $('gripBtn').textContent = closeIt ? 'Close gripper' : 'Open gripper';
  $('tgtGrip').textContent = closeIt ? 'Close gripper' : 'Open gripper';
  if (ui.tab === 'p-jog') {
    for (let i = 0; i < 6; i++) {
      if (ui.frame === 'joint') {
        jogVals[i].textContent = `${(c.q[i] / DEG).toFixed(1)}°`;
        const [lo, hi] = LIMITS[i];
        const t = (c.q[i] - lo) / (hi - lo);
        jogBars[i].firstChild.style.left = `${t * 100}%`;
        jogBars[i].classList.toggle('warn', t < 0.04 || t > 0.96);
      } else {
        jogVals[i].textContent = i < 3 ? `${mm(f.p[i])}` : `${(abc[i - 3] / DEG).toFixed(1)}°`;
      }
    }
  }
  updateMode();
}

// ---------------------------------------------------------------- pointer interaction in the 3D view
const raycaster = new THREE.Raycaster();
const ndc = new THREE.Vector2();
let drag = null;
let tap = null;

function setNdc(e) {
  const r = canvas.getBoundingClientRect();
  ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
  raycaster.setFromCamera(ndc, camera);
}

// Capture-phase on the container so target dragging wins over OrbitControls.
$('app').addEventListener('pointerdown', (e) => {
  if (e.target !== canvas) return;
  tap = { x: e.clientX, y: e.clientY, t: performance.now(), id: e.pointerId };
  if (ui.tab !== 'p-ik' || !target.visible) return;
  setNdc(e);
  if (raycaster.intersectObject(tgtHit, false).length) {
    e.stopPropagation();
    canvas.setPointerCapture(e.pointerId);
    const worldPos = target.getWorldPosition(new THREE.Vector3());
    let normal;
    if (tgt.plane === 'h') normal = new THREE.Vector3(0, 1, 0);
    else {
      normal = camera.getWorldDirection(new THREE.Vector3());
      normal.y = 0;
      normal.normalize();
    }
    const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(normal, worldPos);
    const hit = raycaster.ray.intersectPlane(plane, new THREE.Vector3());
    drag = { id: e.pointerId, plane, offset: hit ? worldPos.clone().sub(hit) : new THREE.Vector3() };
    tgtMat.emissiveIntensity = 1.2;
    controls.enabled = false;
    ctl().jog = null;
    if (tgt.live) { ctl().follow = { p: tgt.p, R: tgt.R }; ctl().setStatus(`${ctl().name}: tracking target`, 'ok'); }
  }
}, { capture: true });

canvas.addEventListener('pointermove', (e) => {
  if (!drag || e.pointerId !== drag.id) return;
  setNdc(e);
  const hit = raycaster.ray.intersectPlane(drag.plane, new THREE.Vector3());
  if (!hit) return;
  hit.add(drag.offset);
  const local = cell.worldToLocal(hit);
  // keep the target in a sensible box around the cell
  local.x = THREE.MathUtils.clamp(local.x, -1.2, 2.9);
  local.y = THREE.MathUtils.clamp(local.y, -1.6, 1.4);
  local.z = THREE.MathUtils.clamp(local.z, 0.012, 1.5);
  tgt.p = [local.x, local.y, local.z];
  syncTarget();
});

const endDrag = (e) => {
  if (drag && e.pointerId === drag.id) {
    drag = null;
    controls.enabled = true;
    tgtMat.emissiveIntensity = 0.6;
    dirty = true;
  }
};
canvas.addEventListener('pointerup', (e) => {
  const wasDrag = !!drag;
  endDrag(e);
  if (wasDrag || !tap || tap.id !== e.pointerId) return;
  const moved = Math.hypot(e.clientX - tap.x, e.clientY - tap.y);
  if (moved < 8 && performance.now() - tap.t < 350) handleTap(e);
  tap = null;
});
canvas.addEventListener('pointercancel', endDrag);

function robotAt(obj) {
  for (let o = obj; o; o = o.parent) {
    const i = ctls.findIndex((c) => c.robot.root === o);
    if (i >= 0) return i;
  }
  return -1;
}

function handleTap(e) {
  setNdc(e);
  if (ui.tab === 'p-pick') {
    const meshes = parts.list.filter((p) => p.state !== 'held').map((p) => p.mesh);
    const ph = raycaster.intersectObjects(meshes, false)[0];
    if (ph) { selectPart(ph.object.userData.partId === ui.selected ? null : ph.object.userData.partId); return; }
    if (!ui.selected) { toast('Tap a part first.'); return; }
    const trayMeshes = world.trays.map((t) => t.mesh);
    const th = raycaster.intersectObjects([...trayMeshes, world.conveyor.mesh, world.bin.mesh, world.table], false)[0];
    if (!th) return;
    if (th.object === world.conveyor.mesh || th.object === world.bin.mesh) {
      if (!parts.infeedClear()) return toast('The conveyor infeed is occupied. Try again in a moment.');
      const [x, y] = CONVEYOR.infeed;
      placeSelected({ x, y, yaw: -Math.PI / 2 });
      return;
    }
    const local = cell.worldToLocal(th.point.clone());
    const trayId = th.object.userData.trayId || parts.trayAt(local.x, local.y)?.id;
    if (trayId) {
      const slot = parts.freeSlot(trayId);
      if (!slot && trayId === 'transfer') return toast('The transfer tray is full.');
      const [x, y] = slot || parts.traySlot(trayId);
      placeSelected({ x, y, trayId });
    } else {
      placeSelected({ x: local.x, y: local.y });
    }
  } else if (ui.tab === 'p-ik') {
    const th = raycaster.intersectObjects([world.table, world.conveyor.mesh, ...world.trays.map((t) => t.mesh), ...parts.list.map((p) => p.mesh)], false)[0];
    if (!th) return;
    const local = cell.worldToLocal(th.point.clone());
    tgt.p = [local.x, local.y, Math.max(local.z + 0.08, 0.08)];
    syncTarget();
  } else {
    // tapping an arm selects it
    const hit = raycaster.intersectObjects(ctls.map((c) => c.robot.root), true)[0];
    if (hit) { const i = robotAt(hit.object); if (i >= 0) selectRobot(i); }
  }
}

// ---------------------------------------------------------------- layout / resize
let pixelRatio = Math.min(devicePixelRatio || 1, 2);
function layout() {
  const w = innerWidth, h = innerHeight;
  renderer.setPixelRatio(pixelRatio);
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  const sheet = $('sheet');
  const landscapeDock = matchMedia('(orientation: landscape) and (min-width: 700px)').matches;
  camera.fov = h > w ? 52 : 40;
  // keep the cell centred in the area not covered by the pendant
  if (landscapeDock) {
    const sw = sheet.getBoundingClientRect().width;
    camera.setViewOffset(w, h, sw / 2, 0, w, h);
  } else {
    const sh = sheet.getBoundingClientRect().height;
    camera.setViewOffset(w, h, 0, sh * 0.42, w, h);
  }
  camera.updateProjectionMatrix();
  dirty = true;
}
let wasPortrait = innerHeight > innerWidth;
addEventListener('resize', () => {
  layout();
  const portrait = innerHeight > innerWidth;
  if (portrait !== wasPortrait) { wasPortrait = portrait; setFollowCam(false); viewFit(false); }
});
new ResizeObserver(() => layout()).observe($('sheet'));
controls.addEventListener('change', () => { dirty = true; });

// ---------------------------------------------------------------- loop
let last = performance.now();
let hudTimer = 0;
let listTimer = 0;
const frameTimes = [];
// One simulation step; also used by the headless tests.
function simulate(dt) {
  line.update();
  let moved = false;
  for (const c of ctls) moved = c.update(dt) || moved;
  const belt = line.conveyorOn && !anyEstop();
  if (belt) world.conveyor.update(dt);
  const falling = parts.step(dt, belt);
  return { moved, active: moved || falling || belt };
}
function frame(now) {
  requestAnimationFrame(frame);
  let dt = (now - last) / 1000;
  last = now;
  if (dt > 0.05) dt = 0.05;

  const { moved, active: anything } = simulate(dt);
  if (anything) dirty = true;

  if (camAnim) {
    camAnim.t = Math.min(1, camAnim.t + dt / 0.6);
    const s = camAnim.t * camAnim.t * (3 - 2 * camAnim.t);
    controls.target.lerpVectors(camAnim.t0, camAnim.t1, s);
    camera.position.lerpVectors(camAnim.p0, camAnim.p1, s);
    if (camAnim.t >= 1) camAnim = null;
    dirty = true;
  }
  if (ui.follow && moved && !camAnim) {
    const tcp = ctl().robot.tcp.getWorldPosition(new THREE.Vector3());
    const delta = tcp.sub(controls.target).multiplyScalar(Math.min(1, dt * 6));
    controls.target.add(delta);
    camera.position.add(delta);
  }
  controls.update();

  hudTimer += dt;
  listTimer += dt;
  if (hudTimer > 0.08 && anything) {
    hudTimer = 0;
    updateReadouts();
    if (ui.tab === 'p-ik' && !drag && !tgt.live) syncTarget();
  }
  if (listTimer > 0.4 && anything && ui.tab === 'p-pick') { listTimer = 0; renderPartList(); }
  // stack light mirrors the cell state
  const light = anyEstop() ? 'red' : line.running || anyBusy() ? 'green' : 'amber';
  if (light !== frame.light) { frame.light = light; world.setStackLight(light); dirty = true; updateMode(); }

  if (dirty) {
    renderer.render(scene, camera);
    dirty = false;
    adaptQuality(dt);
  }
}
// Drop resolution a notch if the GPU cannot keep up (frame time over ~22 ms).
function adaptQuality(dt) {
  frameTimes.push(dt);
  if (frameTimes.length < 90) return;
  const avg = frameTimes.reduce((a, b) => a + b, 0) / frameTimes.length;
  frameTimes.length = 0;
  if (avg > 0.022 && pixelRatio > 1) {
    pixelRatio = Math.max(1, pixelRatio - 0.25);
    layout();
  }
}

// ---------------------------------------------------------------- boot
buildJogRows();
renderPartList();
renderProgram();
updateProgButtons();
layout();
viewFit(false);
updateReadouts();
setStatus('Ready. Pick tab → Run line to start the two-robot transfer.', 'ok');
requestAnimationFrame((t) => { last = t; frame(t); });
requestAnimationFrame(() => $('loading').classList.add('done'));
window.__sim = { ctls, parts, line, camera, controls, selectTab, selectRobot, simulate, fk, world, cell };
