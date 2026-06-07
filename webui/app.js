// MyCobot Pose Visualizer — static three.js app.
// Fetches live state from the Pi REST API (HTTP GET) and renders the arm as a
// kinematic skeleton driven by URDF forward kinematics. Also overlays the
// firmware-reported pose and a pick-target marker.
//
// No build step: ES modules + import map (see index.html). No mesh files are
// shipped with the URDF, so we render link frames + bones rather than meshes.

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

// ---- config -----------------------------------------------------------------

const URDF_URL = './mycobot.urdf'; // served next to this app (see server mount)

// angles[] from /joints/angles map to these revolute joints, in order.
const ARM_JOINTS = [
  'joint2_to_joint1',
  'joint3_to_joint2',
  'joint4_to_joint3',
  'joint5_to_joint4',
  'joint6_to_joint5',
  'joint6output_to_joint6',
];
// Per-joint sign / offset(rad) — tweak here if a joint visually rotates the
// wrong way vs hardware. Defaults assume URDF convention == firmware angle.
const ARM_SIGN = [1, 1, 1, 1, 1, 1];
const ARM_OFFSET = [0, 0, 0, 0, 0, 0];

// Link whose world origin we treat as the visualized end-effector.
const TCP_LINK = 'gripper_base';
// Chain (link names) to draw bones through.
const BONE_CHAIN = ['g_base', 'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint6_flange', 'gripper_base'];

const LS = 'mycobot-viz';

// ---- scene ------------------------------------------------------------------

const canvas = document.getElementById('canvas');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));

const scene = new THREE.Scene();
scene.background = new THREE.Color(0xeef1f4);

const camera = new THREE.PerspectiveCamera(50, 1, 0.01, 100);
camera.up.set(0, 0, 1); // URDF is Z-up
camera.position.set(0.45, -0.45, 0.4);

const controls = new OrbitControls(camera, canvas);
controls.target.set(0, 0, 0.15);
controls.enableDamping = true;

scene.add(new THREE.AmbientLight(0xffffff, 0.7));
const dir = new THREE.DirectionalLight(0xffffff, 0.8);
dir.position.set(1, -1, 2);
scene.add(dir);

// Z-up grid (GridHelper lies in XZ; rotate into XY).
const grid = new THREE.GridHelper(1, 20, 0x9aa7b6, 0xcdd5de);
grid.rotation.x = Math.PI / 2;
scene.add(grid);

// World axes at base.
scene.add(new THREE.AxesHelper(0.1));

// ---- URDF parsing + kinematics ---------------------------------------------

const X = new THREE.Vector3(1, 0, 0);
const Y = new THREE.Vector3(0, 1, 0);
const Z = new THREE.Vector3(0, 0, 1);

function rpyToQuat(r, p, y) {
  // URDF fixed-axis rpy: R = Rz(y) * Ry(p) * Rx(r)
  const q = new THREE.Quaternion().setFromAxisAngle(Z, y);
  q.multiply(new THREE.Quaternion().setFromAxisAngle(Y, p));
  q.multiply(new THREE.Quaternion().setFromAxisAngle(X, r));
  return q;
}

function nums(s) {
  return (s || '').trim().split(/\s+/).map(Number);
}

// Parses URDF text into a THREE object tree. Returns { root, links, joints }.
function buildRobot(text) {
  // Drop anything after the root close tag (the committed file has a stray
  // merge-conflict marker after </robot> that breaks XML parsing).
  const end = text.indexOf('</robot>');
  if (end !== -1) text = text.slice(0, end + '</robot>'.length);

  const xml = new DOMParser().parseFromString(text, 'application/xml');
  if (xml.querySelector('parsererror')) throw new Error('URDF parse error');

  const links = new Map(); // name -> Object3D
  for (const el of xml.querySelectorAll('robot > link')) {
    const o = new THREE.Object3D();
    o.name = el.getAttribute('name');
    links.set(o.name, o);
  }

  const joints = []; // {name,type,obj,baseQuat,axis,mimic}
  const childNames = new Set();
  for (const el of xml.querySelectorAll('robot > joint')) {
    const name = el.getAttribute('name');
    const type = el.getAttribute('type');
    const parent = el.querySelector('parent').getAttribute('link');
    const child = el.querySelector('child').getAttribute('link');
    const originEl = el.querySelector('origin');
    const xyz = originEl && originEl.getAttribute('xyz') ? nums(originEl.getAttribute('xyz')) : [0, 0, 0];
    const rpy = originEl && originEl.getAttribute('rpy') ? nums(originEl.getAttribute('rpy')) : [0, 0, 0];
    const axisEl = el.querySelector('axis');
    const axis = axisEl ? nums(axisEl.getAttribute('xyz')) : [0, 0, 1];
    const mimicEl = el.querySelector('mimic');
    const mimic = mimicEl ? {
      joint: mimicEl.getAttribute('joint'),
      multiplier: parseFloat(mimicEl.getAttribute('multiplier') || '1'),
      offset: parseFloat(mimicEl.getAttribute('offset') || '0'),
    } : null;

    const parentObj = links.get(parent);
    const childObj = links.get(child);
    if (!parentObj || !childObj) continue;

    const jointObj = new THREE.Object3D();
    jointObj.position.set(xyz[0], xyz[1], xyz[2]);
    const baseQuat = rpyToQuat(rpy[0], rpy[1], rpy[2]);
    jointObj.quaternion.copy(baseQuat);
    parentObj.add(jointObj);
    jointObj.add(childObj);
    childNames.add(child);

    joints.push({
      name, type, obj: jointObj,
      baseQuat,
      axis: new THREE.Vector3(axis[0], axis[1], axis[2]).normalize(),
      mimic,
    });
  }

  let rootName = null;
  for (const n of links.keys()) if (!childNames.has(n)) { rootName = n; break; }
  const root = links.get(rootName);

  return { root, links, joints, jointByName: new Map(joints.map(j => [j.name, j])) };
}

// Applies joint values (rad) and resolves mimic joints.
function setJointValues(robot, valueByName) {
  const resolved = new Map(valueByName);
  for (const j of robot.joints) {
    if (j.type !== 'revolute' && j.type !== 'continuous') continue;
    let v = resolved.get(j.name);
    if (v === undefined && j.mimic) {
      const src = resolved.get(j.mimic.joint) ?? 0;
      v = src * j.mimic.multiplier + j.mimic.offset;
    }
    if (v === undefined) v = 0;
    const q = j.baseQuat.clone();
    q.multiply(new THREE.Quaternion().setFromAxisAngle(j.axis, v));
    j.obj.quaternion.copy(q);
  }
  robot.root.updateMatrixWorld(true);
}

// ---- visual decorations (frames + bones + markers) --------------------------

let framesGroup = new THREE.Group();
let robot = null;
let boneLine = null;
const jointDots = [];

function decorate() {
  // Per-link coordinate frames.
  for (const o of robot.links.values()) {
    const ax = new THREE.AxesHelper(0.025);
    ax.material.depthTest = false;
    o.add(ax);
    framesGroup.add(ax);
  }
  scene.add(framesGroup);

  // Bones: a polyline updated each frame from world positions of BONE_CHAIN.
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(BONE_CHAIN.length * 3), 3));
  boneLine = new THREE.Line(geo, new THREE.LineBasicMaterial({ color: 0x8fb6ff }));
  boneLine.renderOrder = 1;
  scene.add(boneLine);

  for (const _ of BONE_CHAIN) {
    const dot = new THREE.Mesh(
      new THREE.SphereGeometry(0.006, 12, 12),
      new THREE.MeshStandardMaterial({ color: 0x4da3ff, emissive: 0x113355 }),
    );
    jointDots.push(dot);
    scene.add(dot);
  }
}

function updateBones() {
  const pos = boneLine.geometry.attributes.position;
  const v = new THREE.Vector3();
  BONE_CHAIN.forEach((name, i) => {
    const o = robot.links.get(name);
    if (!o) return;
    o.getWorldPosition(v);
    pos.setXYZ(i, v.x, v.y, v.z);
    jointDots[i].position.copy(v);
  });
  pos.needsUpdate = true;
}

// TCP marker (URDF FK end-effector).
const tcpMarker = new THREE.Mesh(
  new THREE.SphereGeometry(0.012, 16, 16),
  new THREE.MeshStandardMaterial({ color: 0x4da3ff, emissive: 0x1a4a7a }),
);
scene.add(tcpMarker);

// Firmware-reported pose marker.
const fwMarker = new THREE.Mesh(
  new THREE.SphereGeometry(0.012, 16, 16),
  new THREE.MeshStandardMaterial({ color: 0xff8c42, emissive: 0x5a2a0a, transparent: true, opacity: 0.85 }),
);
scene.add(fwMarker);

// Pick-target marker (yellow) with a drop line to the ground.
const targetGroup = new THREE.Group();
const targetBall = new THREE.Mesh(
  new THREE.SphereGeometry(0.01, 16, 16),
  new THREE.MeshStandardMaterial({ color: 0xffd43b, emissive: 0x5a4a00 }),
);
targetGroup.add(targetBall);
const dropGeo = new THREE.BufferGeometry();
dropGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
const dropLine = new THREE.Line(dropGeo, new THREE.LineDashedMaterial({ color: 0xffd43b, dashSize: 0.01, gapSize: 0.008 }));
targetGroup.add(dropLine);
scene.add(targetGroup);

function updateTarget() {
  const show = el('showTarget').checked;
  const x = parseFloat(el('tx').value), y = parseFloat(el('ty').value), z = parseFloat(el('tz').value);
  const ok = show && [x, y, z].every(Number.isFinite);
  targetGroup.visible = ok;
  if (!ok) return;
  const p = new THREE.Vector3(x, y, z).multiplyScalar(0.001);
  targetBall.position.copy(p);
  const dp = dropLine.geometry.attributes.position;
  dp.setXYZ(0, p.x, p.y, p.z);
  dp.setXYZ(1, p.x, p.y, 0);
  dp.needsUpdate = true;
  dropLine.computeLineDistances();
}

// ---- polling ----------------------------------------------------------------

let timer = null;
let connected = false;

function apiBase() {
  return el('apiBase').value.replace(/\/+$/, '');
}

async function getJSON(path, signal) {
  const r = await fetch(apiBase() + path, { signal });
  if (!r.ok) throw new Error(path + ' → ' + r.status);
  return r.json();
}

async function poll() {
  const ctrl = new AbortController();
  const t = setTimeout(() => ctrl.abort(), 4000);
  try {
    const [angles, coords, grip] = await Promise.allSettled([
      getJSON('/joints/angles', ctrl.signal),
      getJSON('/robot/coords', ctrl.signal),
      getJSON('/gripper/status', ctrl.signal),
    ]);

    if (angles.status === 'fulfilled' && Array.isArray(angles.value.angles)) {
      const deg = angles.value.angles;
      const map = new Map();
      ARM_JOINTS.forEach((name, i) => {
        const rad = THREE.MathUtils.degToRad((deg[i] ?? 0) * ARM_SIGN[i]) + ARM_OFFSET[i];
        map.set(name, rad);
      });
      setJointValues(robot, map);
      updateBones();
      const tcp = new THREE.Vector3();
      robot.links.get(TCP_LINK).getWorldPosition(tcp);
      tcpMarker.position.copy(tcp);
      setText('fk_x', (tcp.x * 1000).toFixed(1));
      setText('fk_y', (tcp.y * 1000).toFixed(1));
      setText('fk_z', (tcp.z * 1000).toFixed(1));
      deg.forEach((d, i) => setText('j' + i, (d ?? 0).toFixed(1)));
      mark(true);
    }

    if (coords.status === 'fulfilled' && Array.isArray(coords.value.coords)) {
      const c = coords.value.coords;
      setText('fw_x', c[0].toFixed(1));
      setText('fw_y', c[1].toFixed(1));
      setText('fw_z', c[2].toFixed(1));
      fwMarker.position.set(c[0], c[1], c[2]).multiplyScalar(0.001);
      fwMarker.visible = el('showFw').checked;
    } else {
      fwMarker.visible = false;
    }

    if (grip.status === 'fulfilled') {
      const g = grip.value;
      const state = g.state ?? g.value ?? g.is_moving;
      const p = el('gripState');
      p.textContent = JSON.stringify(g).slice(0, 40);
      p.className = 'pill good';
    }
  } catch (e) {
    mark(false, e.message);
  } finally {
    clearTimeout(t);
  }
}

function start() {
  if (timer) return;
  localStorage.setItem(LS, JSON.stringify(settings()));
  poll();
  timer = setInterval(poll, Math.max(100, parseInt(el('pollMs').value) || 300));
  connected = true;
  el('toggle').textContent = 'Disconnect';
}

function stop() {
  clearInterval(timer); timer = null; connected = false;
  el('toggle').textContent = 'Connect';
  mark(null, 'stopped');
}

function mark(ok, msg) {
  const p = el('connState');
  if (ok === true) { p.textContent = 'live'; p.className = 'pill good'; setText('connMsg', ''); }
  else if (ok === false) { p.textContent = 'error'; p.className = 'pill bad'; setText('connMsg', msg || ''); }
  else { p.textContent = 'idle'; p.className = 'pill idle'; setText('connMsg', msg || ''); }
}

// ---- UI helpers -------------------------------------------------------------

function el(id) { return document.getElementById(id); }
function setText(id, t) { const e = el(id); if (e) e.textContent = t; }

function settings() {
  return {
    apiBase: el('apiBase').value, pollMs: el('pollMs').value,
    tx: el('tx').value, ty: el('ty').value, tz: el('tz').value,
    targetUrl: el('targetUrl').value, camUrl: el('camUrl').value,
  };
}

function buildJointGrid() {
  const g = el('jgrid');
  for (let i = 0; i < 6; i++) {
    const b = document.createElement('div');
    b.className = 'jbox';
    b.innerHTML = `<div class="lbl">J${i + 1}</div><div class="val" id="j${i}">–</div>`;
    g.appendChild(b);
  }
}

function wireUI() {
  el('toggle').onclick = () => (connected ? stop() : start());
  el('showFrames').onchange = () => framesGroup.visible = el('showFrames').checked;
  el('showGrid').onchange = () => grid.visible = el('showGrid').checked;
  el('showFw').onchange = () => fwMarker.visible = el('showFw').checked && fwMarker.visible;
  ['tx', 'ty', 'tz', 'showTarget'].forEach(id => el(id).oninput = updateTarget);
  el('loadTargets').onclick = async () => {
    const url = el('targetUrl').value.trim();
    if (!url) return;
    try {
      const r = await fetch(url); const j = await r.json();
      const t = Array.isArray(j) ? j[0] : (j.targets ? j.targets[0] : j);
      const pos = t.position || t;
      el('tx').value = pos.x ?? pos[0]; el('ty').value = pos.y ?? pos[1]; el('tz').value = pos.z ?? pos[2];
      updateTarget();
    } catch (e) { alert('targets fetch failed: ' + e.message); }
  };
  const cam = el('cam');
  el('camToggle').onclick = () => {
    const url = el('camUrl').value.trim();
    if (!url) return;
    if (cam.style.display === 'block') { cam.style.display = 'none'; el('camToggle').textContent = 'Show'; }
    else { cam.src = url; cam.style.display = 'block'; el('camToggle').textContent = 'Hide'; }
  };
}

function restore() {
  let s = {};
  try { s = JSON.parse(localStorage.getItem(LS) || '{}'); } catch {}
  el('apiBase').value = s.apiBase || (location.origin.startsWith('http') ? location.origin : 'http://localhost:8080');
  if (s.pollMs) el('pollMs').value = s.pollMs;
  for (const k of ['tx', 'ty', 'tz', 'targetUrl', 'camUrl']) if (s[k] != null) el(k).value = s[k];
}

// ---- main loop --------------------------------------------------------------

function resize() {
  const v = document.getElementById('view');
  const w = v.clientWidth, h = v.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);

let last = performance.now(), frames = 0;
function animate() {
  requestAnimationFrame(animate);
  controls.update();
  updateTarget();
  renderer.render(scene, camera);
  frames++;
  const now = performance.now();
  if (now - last > 1000) { setText('fps', Math.round(frames * 1000 / (now - last)) + ' fps'); frames = 0; last = now; }
}

async function init() {
  buildJointGrid();
  restore();
  wireUI();
  resize();
  animate();
  try {
    const text = await (await fetch(URDF_URL)).text();
    robot = buildRobot(text);
    scene.add(robot.root);
    decorate();
    // Neutral pose at start.
    setJointValues(robot, new Map(ARM_JOINTS.map(n => [n, 0])));
    updateBones();
    mark(null, 'URDF loaded — set API base and Connect');

    // Debug/console hook: drive the model without the API, e.g.
    //   viz.setPose([0, 30, -60, 0, 0, 0])   // degrees, J1..J6
    window.viz = {
      robot, setJointValues, updateBones, ARM_JOINTS,
      setPose(deg) {
        const m = new Map(ARM_JOINTS.map((n, i) =>
          [n, THREE.MathUtils.degToRad((deg[i] ?? 0) * ARM_SIGN[i]) + ARM_OFFSET[i]]));
        setJointValues(robot, m); updateBones();
        const tcp = new THREE.Vector3();
        robot.links.get(TCP_LINK).getWorldPosition(tcp);
        return { tcp_mm: tcp.toArray().map(v => +(v * 1000).toFixed(1)) };
      },
    };
  } catch (e) {
    mark(false, 'URDF load failed: ' + e.message);
    console.error(e);
  }
}

init();
