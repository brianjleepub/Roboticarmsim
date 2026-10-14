import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { createRenderer, createWorld, RISER_H, TABLE } from './scene.js';
import { RobotModel, makeMaterials, GRIPPER_MAX_GAP } from './robot.js';
import { Parts } from './parts.js';
import { Controller } from './motion.js';
import { fk, ik, LIMITS, DEG, HOME, toolDown, toABC, fromABC } from './kinematics.js';

const $ = (id) => document.getElementById(id);

// ---------------------------------------------------------------- setup
const canvas = $('view');
const renderer = createRenderer(canvas);
const world = createWorld(renderer);
const { scene, cell } = world;

const robot = new RobotModel(makeMaterials());
cell.add(robot.root);
const parts = new Parts(cell);
const ctl = new Controller({ robot, parts });

const camera = new THREE.PerspectiveCamera(45, 1, 0.05, 60);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.09;
controls.minDistance = 0.7;
controls.maxDistance = 7;
controls.maxPolarAngle = Math.PI * 0.495;
controls.rotateSpeed = 0.7;
controls.zoomSpeed = 0.9;
controls.touches = { ONE: THREE.TOUCH.ROTATE, TWO: THREE.TOUCH.DOLLY_PAN };

const HOME_TARGET = new THREE.Vector3(0.35, RISER_H + 0.3, 0);
function viewFit(animate = true) {
  const portrait = innerHeight > innerWidth;
  const dist = portrait ? 4.6 : 2.7;
  const dir = new THREE.Vector3(1.0, 0.62, 0.95).normalize();
  setView(HOME_TARGET.clone(), HOME_TARGET.clone().addScaledVector(dir, dist), animate);
}
function viewTop() {
  const t = new THREE.Vector3(0.55, RISER_H, 0);
  setView(t, t.clone().add(new THREE.Vector3(0.001, 2.7, 0.0)), true);
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

const tgt = { p: [0.665, 0, 0.75], R: toolDown(0), orient: 'down', plane: 'h', live: true };

function setTargetFromTool() {
  const f = fk(ctl.q);
  tgt.p = f.p.slice();
  tgt.R = tgt.orient === 'down' ? toolDown(toABC(f.R)[0]) : f.R;
  if (tgt.orient === 'down') {
    // derive yaw from the current finger axis so the wrist does not spin
    const yAxis = [f.R[1], f.R[4]];
    tgt.R = toolDown(Math.atan2(yAxis[1], yAxis[0]) - Math.PI / 2);
  }
  syncTarget();
}

function syncTarget() {
  target.position.set(...tgt.p);
  const m = new THREE.Matrix4().set(
    tgt.R[0], tgt.R[1], tgt.R[2], 0, tgt.R[3], tgt.R[4], tgt.R[5], 0, tgt.R[6], tgt.R[7], tgt.R[8], 0, 0, 0, 0, 1,
  );
  target.quaternion.setFromRotationMatrix(m);
  const ok = !!ik(tgt.p, tgt.R, ctl.q);
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
  reach.textContent = ok ? 'reachable' : 'out of reach';
  reach.className = ok ? '' : 'bad';
  if (tgt.live && ui.tab === 'p-ik') ctl.follow = { p: tgt.p, R: tgt.R };
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
  toast.timer = setTimeout(() => t.classList.remove('show'), 2200);
}

ctl.on('status', ({ text, level }) => {
  $('statusText').textContent = text;
  $('status').dataset.level = level;
});
ctl.on('grip', () => { renderPartList(); });
ctl.on('stopped', () => { prog.running = false; updateProgButtons(); renderProgram(); });

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
    if (!ctl.busy) setTargetFromTool();
  } else {
    ctl.follow = null;
  }
  if (pane !== 'p-pick') selectPart(null);
  $('sheet').classList.remove('min');
  layout();
  dirty = true;
}

// sheet collapse
$('handle').addEventListener('click', () => { $('sheet').classList.toggle('min'); setTimeout(layout, 260); layout(); });

// speed override
$('override').addEventListener('input', (e) => {
  ctl.override = e.target.value / 100;
  $('ovrOut').textContent = `${e.target.value}%`;
});

// E-stop
$('estop').addEventListener('click', () => {
  const on = !ctl.estop;
  ctl.setEstop(on);
  $('estop').setAttribute('aria-pressed', String(on));
  $('estop').textContent = on ? 'RESET' : 'E-STOP';
  if (on) { prog.running = false; updateProgButtons(); }
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
    const tcp = robot.tcp.getWorldPosition(new THREE.Vector3());
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
  const start = (e) => {
    e.preventDefault();
    if (ctl.estop) { toast('Release the E-stop first.'); return; }
    if (ctl.busy) { toast('Stop the running motion before jogging.'); return; }
    b.setPointerCapture?.(e.pointerId);
    b.classList.add('on');
    ctl.follow = null;
    ctl.jog = { frame: ui.frame, axis: +b.dataset.axis, dir: +b.dataset.dir };
    ctl.setStatus(`Jogging ${ui.frame === 'joint' ? 'A' + (+b.dataset.axis + 1) : CART_NAMES[+b.dataset.axis]} (${ui.frame === 'joint' ? 'axes' : ui.frame})`, 'info');
  };
  const end = () => {
    if (!b.classList.contains('on')) return;
    b.classList.remove('on');
    ctl.jog = null;
    ctl.setStatus('Ready', 'ok');
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
  if (ctl.estop) return toast('Release the E-stop first.');
  ctl.follow = null;
  ctl.goHome();
});
const toggleGrip = () => {
  if (ctl.estop) return toast('Release the E-stop first.');
  ctl.gripper(ctl.gap > GRIPPER_MAX_GAP / 2 && !ctl.held);
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
  ctl.follow = tgt.live ? { p: tgt.p, R: tgt.R } : null;
  syncTarget();
});
$('tgtSnap').addEventListener('click', () => { ctl.follow = null; setTargetFromTool(); });
$('movePtp').addEventListener('click', () => {
  if (ctl.estop) return toast('Release the E-stop first.');
  ctl.enqueue({ type: 'ptp', pose: { p: tgt.p.slice(), R: tgt.R }, label: 'Target', onStart: () => ctl.setStatus('PTP to target', 'info'), onDone: () => ctl.setStatus('At target', 'ok') });
});
$('moveLin').addEventListener('click', () => {
  if (ctl.estop) return toast('Release the E-stop first.');
  ctl.enqueue({ type: 'lin', pose: { p: tgt.p.slice(), R: tgt.R }, label: 'Target', onStart: () => ctl.setStatus('LIN to target', 'info'), onDone: () => ctl.setStatus('At target', 'ok') });
});

// ---------------------------------------------------------------- pick panel
function partWhere(p) {
  if (p.state === 'held') return 'in gripper';
  const tray = parts.trayAt(p.mesh.position.x, p.mesh.position.y);
  return tray ? `in ${tray.label.toLowerCase()} tray` : 'on table';
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
}
function selectPart(id) {
  ui.selected = id;
  parts.setHighlight(id);
  const p = id && parts.byId(id);
  $('pickHint').innerHTML = p
    ? `<b>${p.name}</b> selected. Now tap a tray or a free spot on the table.`
    : '<b>1.</b> Tap a part (in the view or below). <b>2.</b> Tap a tray or a spot on the table.';
  renderPartList();
  dirty = true;
}
function placeSelected(x, y, trayId) {
  const p = parts.byId(ui.selected);
  if (!p) return;
  if (ctl.estop) return toast('Release the E-stop first.');
  if (ctl.busy) return toast('Wait for the current move to finish.');
  if (p.state !== 'rest') return toast('That part is not on a surface.');
  let dest = { x, y };
  if (trayId) { const [sx, sy] = parts.traySlot(trayId); dest = { x: sx, y: sy }; }
  if (!ik([dest.x, dest.y, 0.15], toolDown(0), ctl.q) || !ik([dest.x, dest.y, 0.04], toolDown(0), ctl.q)) {
    return toast('That spot is out of reach.');
  }
  const g = ctl.graspPose(p);
  if (!ik([g.x, g.y, g.zGrip], toolDown(g.yaw), ctl.q)) return toast('That part is out of reach.');
  ctl.follow = null;
  ctl.pickAndPlace(p, dest, { home: true });
  ctl.enqueue({ type: 'call', fn: () => { ctl.setStatus('Pick and place done', 'ok'); renderPartList(); } });
  selectPart(null);
}
$('sortBtn').addEventListener('click', () => {
  if (ctl.estop) return toast('Release the E-stop first.');
  if (ctl.busy) return toast('Already running.');
  ctl.follow = null;
  selectPart(null);
  const next = () => {
    const p = parts.list.find((x) => x.state === 'rest' && parts.trayAt(x.mesh.position.x, x.mesh.position.y)?.id !== x.color);
    renderPartList();
    if (!p) {
      ctl.goHome();
      ctl.enqueue({ type: 'call', fn: () => ctl.setStatus('All parts sorted', 'ok') });
      return;
    }
    const [x, y] = parts.traySlot(p.color);
    ctl.pickAndPlace(p, { x, y });
    ctl.enqueue({ type: 'call', fn: next });
  };
  next();
});
$('resetParts').addEventListener('click', () => {
  if (ctl.busy) ctl.stop('Stopped');
  if (ctl.held) { ctl.held = null; }
  parts.reset();
  ctl.gap = GRIPPER_MAX_GAP;
  robot.setGap(ctl.gap);
  renderPartList();
  dirty = true;
  toast('Parts reset');
});
$('stopBtn').addEventListener('click', () => ctl.stop('Stopped by operator'));

// ---------------------------------------------------------------- program (teach)
const prog = { steps: [], running: false, loop: false, cur: -1, example: true };
function poseText(q) {
  const f = fk(q);
  return `${mm(f.p[0])}, ${mm(f.p[1])}, ${mm(f.p[2])}`;
}
function renderProgram() {
  const list = $('progList');
  list.innerHTML = '';
  if (!prog.steps.length) {
    list.innerHTML = '<div class="empty">No steps yet. Move the arm, then tap + PTP.</div>';
  }
  prog.steps.forEach((s, i) => {
    const row = document.createElement('div');
    row.className = 'step' + (prog.running && prog.cur === i ? ' cur' : '');
    const t = s.type === 'grip' ? (s.close ? 'grip' : 'open') : s.type;
    const label = { ptp: 'PTP', lin: 'LIN', grip: 'CLOSE', open: 'OPEN', wait: 'WAIT' }[t];
    const desc = s.type === 'wait' ? `${s.t.toFixed(1)} s` : s.q ? poseText(s.q) : s.close ? 'gripper' : 'gripper';
    row.innerHTML = `<span class="n">${i + 1}</span><span class="t ${t}">${label}</span><span class="d">${desc}</span><button aria-label="Delete step ${i + 1}">×</button>`;
    row.querySelector('button').addEventListener('click', () => {
      if (prog.running) return;
      prog.steps.splice(i, 1);
      renderProgram();
    });
    list.appendChild(row);
  });
  $('progTag').hidden = !prog.example;
}
function addStep(s) {
  if (prog.running) return toast('Stop the program to edit it.');
  if (prog.example) { prog.example = false; }
  prog.steps.push(s);
  renderProgram();
  toast(`Step ${prog.steps.length} recorded`);
}
$('addPtp').addEventListener('click', () => addStep({ type: 'ptp', q: ctl.q.slice() }));
$('addLin').addEventListener('click', () => addStep({ type: 'lin', q: ctl.q.slice() }));
$('addClose').addEventListener('click', () => addStep({ type: 'grip', close: true }));
$('addOpen').addEventListener('click', () => addStep({ type: 'grip', close: false }));
$('addWait').addEventListener('click', () => addStep({ type: 'wait', t: 0.5 }));
$('clearProg').addEventListener('click', () => {
  if (prog.running) return;
  prog.steps = [];
  prog.example = false;
  renderProgram();
});
$('loopProg').addEventListener('click', () => {
  prog.loop = !prog.loop;
  $('loopProg').setAttribute('aria-pressed', String(prog.loop));
  $('loopProg').textContent = prog.loop ? 'Loop on' : 'Loop off';
});
$('runProg').addEventListener('click', () => {
  if (prog.running) { ctl.stop('Program stopped'); return; }
  if (ctl.estop) return toast('Release the E-stop first.');
  if (!prog.steps.length) return toast('Record some steps first.');
  if (ctl.busy) return toast('Wait for the current move to finish.');
  ctl.follow = null;
  prog.running = true;
  updateProgButtons();
  ctl.setStatus('Program running', 'info');
  queueProgram();
});
function queueProgram() {
  prog.steps.forEach((s, i) => {
    const mark = () => { prog.cur = i; renderProgram(); };
    if (s.type === 'ptp') ctl.enqueue({ type: 'ptp', q: s.q.slice(), onStart: mark });
    else if (s.type === 'lin') { const f = fk(s.q); ctl.enqueue({ type: 'lin', pose: { p: f.p, R: f.R }, onStart: mark }); }
    else if (s.type === 'grip') ctl.enqueue({ type: 'grip', close: s.close, onStart: mark });
    else if (s.type === 'wait') ctl.enqueue({ type: 'wait', T: s.t, onStart: mark });
  });
  ctl.enqueue({
    type: 'call',
    fn: () => {
      if (prog.running && prog.loop) { queueProgram(); return; }
      prog.running = false;
      prog.cur = -1;
      updateProgButtons();
      renderProgram();
      ctl.setStatus('Program finished', 'ok');
    },
  });
}
function updateProgButtons() {
  $('runProg').textContent = prog.running ? 'Stop' : 'Run';
  updateMode();
}
function updateMode() {
  $('mode').textContent = ctl.estop ? 'E-stop' : prog.running || ctl.busy ? 'Auto' : 'Manual T1';
}
// Example program: a demonstration path over the trays (no gripping).
function examplePose(x, y, z, yaw) { return ik([x, y, z], toolDown(yaw), HOME); }
{
  const a = examplePose(0.42, 0.56, 0.2, 0);
  const b = examplePose(0.42, 0.56, 0.09, 0);
  const c = examplePose(0.6, -0.56, 0.2, 0.6);
  const d = examplePose(0.6, -0.56, 0.09, 0.6);
  prog.steps = [
    { type: 'ptp', q: HOME.slice() },
    { type: 'ptp', q: a }, { type: 'lin', q: b }, { type: 'wait', t: 0.4 }, { type: 'lin', q: a },
    { type: 'ptp', q: c }, { type: 'lin', q: d }, { type: 'wait', t: 0.4 }, { type: 'lin', q: c },
    { type: 'ptp', q: HOME.slice() },
  ].filter((s) => s.type !== 'ptp' && s.type !== 'lin' || s.q);
}

// ---------------------------------------------------------------- readouts
function updateReadouts() {
  const f = fk(ctl.q);
  const abc = toABC(f.R);
  $('hX').textContent = mm(f.p[0]);
  $('hY').textContent = mm(f.p[1]);
  $('hZ').textContent = mm(f.p[2]);
  $('hA').textContent = (abc[0] / DEG).toFixed(1);
  $('hB').textContent = (abc[1] / DEG).toFixed(1);
  $('hC').textContent = (abc[2] / DEG).toFixed(1);
  $('hGrip').textContent = ctl.held ? `holding ${ctl.held.name.toLowerCase()}` : ctl.gap < 0.002 ? 'closed' : ctl.gap > GRIPPER_MAX_GAP - 0.001 ? 'open' : `${(ctl.gap * 1000).toFixed(0)} mm`;
  const closeIt = ctl.gap > GRIPPER_MAX_GAP / 2 && !ctl.held;
  $('gripBtn').textContent = closeIt ? 'Close gripper' : 'Open gripper';
  $('tgtGrip').textContent = closeIt ? 'Close gripper' : 'Open gripper';
  if (ui.tab === 'p-jog') {
    for (let i = 0; i < 6; i++) {
      if (ui.frame === 'joint') {
        jogVals[i].textContent = `${(ctl.q[i] / DEG).toFixed(1)}°`;
        const [lo, hi] = LIMITS[i];
        const t = (ctl.q[i] - lo) / (hi - lo);
        jogBars[i].firstChild.style.left = `${t * 100}%`;
        jogBars[i].classList.toggle('warn', t < 0.04 || t > 0.96);
      } else {
        const v = i < 3 ? `${mm(f.p[i])}` : `${(abc[i - 3] / DEG).toFixed(1)}°`;
        jogVals[i].textContent = v;
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
    ctl.jog = null;
    if (tgt.live) { ctl.follow = { p: tgt.p, R: tgt.R }; ctl.setStatus('Tracking target', 'ok'); }
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
  local.x = THREE.MathUtils.clamp(local.x, -1.2, 1.4);
  local.y = THREE.MathUtils.clamp(local.y, -1.3, 1.3);
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

function handleTap(e) {
  setNdc(e);
  if (ui.tab === 'p-pick') {
    const meshes = parts.list.filter((p) => p.state !== 'held').map((p) => p.mesh);
    const ph = raycaster.intersectObjects(meshes, false)[0];
    if (ph) { selectPart(ph.object.userData.partId === ui.selected ? null : ph.object.userData.partId); return; }
    if (!ui.selected) { toast('Tap a part first.'); return; }
    const trayMeshes = world.trays.map((t) => t.mesh);
    const th = raycaster.intersectObjects([...trayMeshes, world.table], false)[0];
    if (!th) return;
    const local = cell.worldToLocal(th.point.clone());
    const tray = th.object.userData.trayId || parts.trayAt(local.x, local.y)?.id;
    placeSelected(local.x, local.y, tray);
  } else if (ui.tab === 'p-ik') {
    const th = raycaster.intersectObjects([world.table, ...world.trays.map((t) => t.mesh), ...parts.list.map((p) => p.mesh)], false)[0];
    if (!th) return;
    const local = cell.worldToLocal(th.point.clone());
    tgt.p = [local.x, local.y, Math.max(local.z + 0.08, 0.08)];
    syncTarget();
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
  // keep the robot centred in the area not covered by the pendant
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
const frameTimes = [];
function frame(now) {
  requestAnimationFrame(frame);
  let dt = (now - last) / 1000;
  last = now;
  if (dt > 0.05) dt = 0.05;

  const moved = ctl.update(dt);
  const falling = parts.step(dt);
  if (moved || falling) dirty = true;

  if (camAnim) {
    camAnim.t = Math.min(1, camAnim.t + dt / 0.6);
    const s = camAnim.t * camAnim.t * (3 - 2 * camAnim.t);
    controls.target.lerpVectors(camAnim.t0, camAnim.t1, s);
    camera.position.lerpVectors(camAnim.p0, camAnim.p1, s);
    if (camAnim.t >= 1) camAnim = null;
    dirty = true;
  }
  if (ui.follow && moved && !camAnim) {
    const tcp = robot.tcp.getWorldPosition(new THREE.Vector3());
    const delta = tcp.sub(controls.target).multiplyScalar(Math.min(1, dt * 6));
    controls.target.add(delta);
    camera.position.add(delta);
  }
  controls.update();

  hudTimer += dt;
  if (hudTimer > 0.08 && (moved || falling)) {
    hudTimer = 0;
    updateReadouts();
    if (ui.tab === 'p-ik' && !drag && !tgt.live) syncTarget();
  }
  // stack light mirrors the controller state
  const light = ctl.estop ? 'red' : ctl.busy ? 'green' : 'amber';
  if (light !== frame.light) { frame.light = light; world.setStackLight(light); dirty = true; updateMode(); }

  if (dirty) {
    const t0 = performance.now();
    renderer.render(scene, camera);
    dirty = false;
    adaptQuality(performance.now() - t0, dt);
  }
}
// Drop resolution a notch if the GPU cannot keep up (frame time over ~22 ms).
function adaptQuality(cpu, dt) {
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
layout();
viewFit(false);
updateReadouts();
ctl.setStatus('Ready. Hold + / − to jog.', 'ok');
requestAnimationFrame((t) => { last = t; frame(t); });
requestAnimationFrame(() => $('loading').classList.add('done'));
window.__sim = { ctl, parts, robot, camera, controls, selectTab, fk };
