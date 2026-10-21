/**
 * Miku Arm · 浏览器 3D 预览器
 * ---------------------------------------------------------------
 * 直接读 URDF + STL，自己搭关节树，不依赖 ROS / RViz。
 * 交互：左键转视角、滚轮缩放、右键平移、拖机械臂摆关节（或拖末端走 IK）。
 */

import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';

/* ==================================================================
   0. 配置
   ================================================================== */

const CFG = {
  // miku_dummy.urdf 的关节原点和 STL 是对齐的；miku_arm.urdf 是 SW 原始导出，
  // 原点在一个旋转过的坐标里，直接用会把连杆拆散。
  urdfUrl: 'assets/urdf/ur5e.urdf',
  /** URDF 里 package://<包名>/... 的解析前缀 */
  packageRoots: { arm_control: 'assets' },
  /** URDF 是 Z 轴朝上，three 是 Y 轴朝上，这里做一次转换 */
  zUpToYUp: true,
  homePose: [15, -10, 40, 0, 45, 0], // 度
};

// 想临时看别的 URDF：?urdf=assets/urdf/miku_arm.urdf
const urdfParam = new URLSearchParams(location.search).get('urdf');
if (urdfParam) CFG.urdfUrl = urdfParam;
if (CFG.urdfUrl.includes('ur5e')) CFG.homePose = [-90, -90, 90, -90, -90, 0];

/* ==================================================================
   1. 小工具
   ================================================================== */

const DEG = 180 / Math.PI;
const RAD = Math.PI / 180;

/** URDF 的 rpy（固定轴 XYZ）→ 四元数，等价于 Rz(yaw)·Ry(pitch)·Rx(roll) */
function rpyToQuat(rpy) {
  const e = new THREE.Euler(rpy[0], rpy[1], rpy[2], 'ZYX');
  return new THREE.Quaternion().setFromEuler(e);
}

function num(v, fallback = 0) {
  const n = parseFloat(v);
  return Number.isFinite(n) ? n : fallback;
}

function readVec3(str, fallback = [0, 0, 0]) {
  if (!str) return new THREE.Vector3(...fallback);
  const p = str.trim().split(/\s+/).map(Number);
  return new THREE.Vector3(
    Number.isFinite(p[0]) ? p[0] : fallback[0],
    Number.isFinite(p[1]) ? p[1] : fallback[1],
    Number.isFinite(p[2]) ? p[2] : fallback[2]
  );
}

/** 取直接子元素（不用 querySelector，免得匹配到嵌套） */
function kids(el, tag) {
  const out = [];
  if (!el) return out;
  for (const c of el.children) if (c.nodeName === tag) out.push(c);
  return out;
}
const kid = (el, tag) => kids(el, tag)[0] || null;

function readOrigin(el) {
  const o = kid(el, 'origin');
  const rpy = ((o && o.getAttribute('rpy')) || '').trim().split(/\s+/);
  return {
    pos: readVec3(o && o.getAttribute('xyz')),
    quat: rpyToQuat([num(rpy[0]), num(rpy[1]), num(rpy[2])]),
  };
}

/* ==================================================================
   2. STL 解析（二进制 + ASCII 都吃）
   ================================================================== */

function parseSTL(buffer) {
  const dv = new DataView(buffer);
  const nTri = buffer.byteLength >= 84 ? dv.getUint32(80, true) : 0;
  const looksBinary = buffer.byteLength >= 84 && 84 + nTri * 50 === buffer.byteLength;

  if (!looksBinary) return parseAsciiSTL(new TextDecoder().decode(buffer));

  const pos = new Float32Array(nTri * 9);
  const nor = new Float32Array(nTri * 9);
  let o = 84;
  for (let i = 0; i < nTri; i++) {
    const nx = dv.getFloat32(o, true);
    const ny = dv.getFloat32(o + 4, true);
    const nz = dv.getFloat32(o + 8, true);
    o += 12;
    for (let v = 0; v < 3; v++) {
      const k = i * 9 + v * 3;
      pos[k] = dv.getFloat32(o, true);
      pos[k + 1] = dv.getFloat32(o + 4, true);
      pos[k + 2] = dv.getFloat32(o + 8, true);
      nor[k] = nx;
      nor[k + 1] = ny;
      nor[k + 2] = nz;
      o += 12;
    }
    o += 2;
  }
  return finishGeometry(pos, nor);
}

function parseAsciiSTL(text) {
  const verts = [];
  const normals = [];
  const re =
    /facet\s+normal\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)[\s\S]*?vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)/g;
  let m;
  while ((m = re.exec(text))) {
    const v = m.slice(1).map(Number);
    for (let i = 0; i < 3; i++) {
      normals.push(v[0], v[1], v[2]);
      verts.push(v[3 + i * 3], v[4 + i * 3], v[5 + i * 3]);
    }
  }
  return finishGeometry(new Float32Array(verts), new Float32Array(normals));
}

function finishGeometry(pos, nor) {
  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
  if (nor && nor.length === pos.length) geo.setAttribute('normal', new THREE.BufferAttribute(nor, 3));
  else geo.computeVertexNormals();
  geo.computeBoundingBox();
  geo.computeBoundingSphere();
  return geo;
}

async function loadSTL(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`网格加载失败 ${res.status}：${url}`);
  return parseSTL(await res.arrayBuffer());
}

/* ==================================================================
   3. URDF 解析 + 关节树构建
   ================================================================== */

class Joint {
  constructor(spec) {
    this.name = spec.name;
    this.type = spec.type; // revolute | continuous | prismatic | fixed
    this.parentName = spec.parent;
    this.childName = spec.child;
    this.originPos = spec.origin.pos.clone();
    this.originQuat = spec.origin.quat.clone();
    this.axis = spec.axis.clone().normalize();
    this.lower = spec.lower;
    this.upper = spec.upper;
    this.value = 0;
    this.obj = null; // THREE.Object3D
    this._q = new THREE.Quaternion();
  }
  get movable() {
    return this.type === 'revolute' || this.type === 'continuous' || this.type === 'prismatic';
  }
  get min() {
    return this.type === 'continuous' ? -Math.PI : this.lower;
  }
  get max() {
    return this.type === 'continuous' ? Math.PI : this.upper;
  }
  apply() {
    if (this.type === 'fixed') {
      this.obj.quaternion.copy(this.originQuat);
      this.obj.position.copy(this.originPos);
    } else if (this.type === 'prismatic') {
      this.obj.quaternion.copy(this.originQuat);
      this.obj.position.copy(this.originPos).addScaledVector(this.axis, this.value);
    } else {
      this.obj.position.copy(this.originPos);
      this._q.setFromAxisAngle(this.axis, this.value);
      this.obj.quaternion.copy(this.originQuat).multiply(this._q);
    }
  }
}

class Robot {
  constructor() {
    this.root = new THREE.Group();
    this.root.name = 'robot';
    this.joints = [];
    this.links = new Map(); // name -> { obj, meshes }
    this.pickables = [];
    this.linkJoint = new Map(); // linkName -> 父关节
    this.ee = null;
    this.eeTipLocal = 0;
  }
  get movableJoints() {
    return this.joints.filter((j) => j.movable);
  }
  eeWorld(out = new THREE.Vector3()) {
    return this.ee.getWorldPosition(out);
  }
  setPoseRad(values) {
    this.movableJoints.forEach((j, i) => {
      if (i >= values.length) return;
      j.value = THREE.MathUtils.clamp(values[i], j.min, j.max);
      j.apply();
    });
    this.root.updateMatrixWorld(true);
  }
  getPoseRad() {
    return this.movableJoints.map((j) => j.value);
  }
}

function resolveMeshUrl(filename) {
  let p = String(filename || '').replace(/\\/g, '/');
  const m = p.match(/^package:\/\/([^/]+)\/(.*)$/);
  if (m) {
    const root = CFG.packageRoots[m[1]];
    p = root ? `${root}/${m[2]}` : m[2];
  }
  return p.replace(/^\.?\//, '');
}

function geometryFromEl(geoEl) {
  if (!geoEl) return null;
  const mesh = kid(geoEl, 'mesh');
  if (mesh) {
    return {
      kind: 'mesh',
      url: resolveMeshUrl(mesh.getAttribute('filename')),
      scale: readVec3(mesh.getAttribute('scale'), [1, 1, 1]),
    };
  }
  const box = kid(geoEl, 'box');
  if (box) return { kind: 'box', size: readVec3(box.getAttribute('size'), [0.1, 0.1, 0.1]) };
  const cyl = kid(geoEl, 'cylinder');
  if (cyl) {
    return {
      kind: 'cylinder',
      radius: num(cyl.getAttribute('radius'), 0.05),
      length: num(cyl.getAttribute('length'), 0.1),
    };
  }
  const sph = kid(geoEl, 'sphere');
  if (sph) return { kind: 'sphere', radius: num(sph.getAttribute('radius'), 0.05) };
  return null;
}

/** 找一组网格里离原点最远的顶点，当工具点（夹爪尖端） */
function computeToolTip(meshes) {
  let best = -1;
  const v = new THREE.Vector3();
  for (const m of meshes) {
    const pos = m.geometry.attributes.position;
    for (let i = 0; i < pos.count; i++) {
      v.fromBufferAttribute(pos, i);
      const d = v.length();
      if (d > best) best = d;
    }
  }
  if (best <= 0) return new THREE.Vector3();

  // 尖端点云取平均，免得标记落在某个三角面的角上
  const sum = new THREE.Vector3();
  let n = 0;
  const cutoff = best * 0.965;
  for (const m of meshes) {
    const pos = m.geometry.attributes.position;
    for (let i = 0; i < pos.count; i++) {
      v.fromBufferAttribute(pos, i);
      if (v.length() >= cutoff) {
        sum.add(v);
        n++;
      }
    }
  }
  return n ? sum.divideScalar(n) : new THREE.Vector3(0, best, 0);
}

async function loadRobot(url) {
  const text = await fetch(url).then((r) => {
    if (!r.ok) throw new Error(`URDF 读取失败 ${r.status}：${url}`);
    return r.text();
  });
  const doc = new DOMParser().parseFromString(text, 'application/xml');
  const parseErr = doc.querySelector('parsererror');
  if (parseErr) throw new Error('URDF 不是合法 XML：\n' + parseErr.textContent);

  const robotEl =
    doc.documentElement.nodeName === 'robot' ? doc.documentElement : doc.querySelector('robot');
  if (!robotEl) throw new Error('URDF 里找不到 <robot> 节点');

  /* ---- links ---- */
  const linkSpecs = new Map();
  for (const linkEl of kids(robotEl, 'link')) {
    const visuals = [];
    for (const vEl of kids(linkEl, 'visual')) {
      const geo = geometryFromEl(kid(vEl, 'geometry'));
      if (!geo) continue;
      const matEl = kid(vEl, 'material');
      const colorEl = matEl && kid(matEl, 'color');
      const rgba = colorEl ? colorEl.getAttribute('rgba').trim().split(/\s+/).map(Number) : null;
      visuals.push({ geo, origin: readOrigin(vEl), rgba });
    }
    linkSpecs.set(linkEl.getAttribute('name'), { visuals });
  }

  /* ---- joints ---- */
  const jointSpecs = [];
  for (const jEl of kids(robotEl, 'joint')) {
    const type = (jEl.getAttribute('type') || 'fixed').toLowerCase();
    const limEl = kid(jEl, 'limit');
    const axisEl = kid(jEl, 'axis');
    jointSpecs.push({
      name: jEl.getAttribute('name'),
      type,
      parent: kid(jEl, 'parent')?.getAttribute('link'),
      child: kid(jEl, 'child')?.getAttribute('link'),
      origin: readOrigin(jEl),
      axis: readVec3(axisEl?.getAttribute('xyz'), [1, 0, 0]),
      lower: limEl ? num(limEl.getAttribute('lower'), -Math.PI) : -Math.PI,
      upper: limEl ? num(limEl.getAttribute('upper'), Math.PI) : Math.PI,
    });
  }

  /* ---- 组装树 ---- */
  const robot = new Robot();
  const jointsByParent = new Map();
  for (const j of jointSpecs) {
    if (!jointsByParent.has(j.parent)) jointsByParent.set(j.parent, []);
    jointsByParent.get(j.parent).push(j);
  }
  const childLinks = new Set(jointSpecs.map((j) => j.child));
  const rootName =
    [...linkSpecs.keys()].find((n) => !childLinks.has(n)) || linkSpecs.keys().next().value;

  const geoCache = new Map();
  const materials = new Map();
  const rgbaMats = new Map();

  const urdfMats = new Map();
  for (const m of kids(robotEl, 'material')) {
    const c = kid(m, 'color');
    if (c) urdfMats.set(m.getAttribute('name'), c.getAttribute('rgba'));
  }

  const materialFor = (name, idx) => {
    if (materials.has(name)) return materials.get(name);
    const rgba = urdfMats.get(name);
    const col = rgba
      ? new THREE.Color(...rgba.trim().split(/\s+/).slice(0, 3).map(Number))
      : new THREE.Color().setHSL((0.52 + idx * 0.035) % 1, 0.16, 0.72);
    const mat = new THREE.MeshStandardMaterial({
      color: col,
      metalness: 0.35,
      roughness: 0.42,
      envMapIntensity: 0.9,
    });
    materials.set(name, mat);
    return mat;
  };

  const materialFromRgba = (rgba) => {
    const key = rgba.join(',');
    if (rgbaMats.has(key)) return rgbaMats.get(key);
    const mat = new THREE.MeshStandardMaterial({
      color: new THREE.Color(rgba[0], rgba[1], rgba[2]),
      metalness: 0.35,
      roughness: 0.42,
      envMapIntensity: 0.9,
    });
    rgbaMats.set(key, mat);
    return mat;
  };

  async function buildMesh(visual, matName, idx) {
    let geo;
    if (visual.geo.kind === 'mesh') {
      const url = visual.geo.url;
      if (!geoCache.has(url)) geoCache.set(url, loadSTL(url));
      geo = await geoCache.get(url);
      const s = visual.geo.scale;
      if (s && !(s.x === 1 && s.y === 1 && s.z === 1)) {
        geo = geo.clone();
        geo.scale(s.x, s.y, s.z);
      }
    } else if (visual.geo.kind === 'box') {
      geo = new THREE.BoxGeometry(...visual.geo.size.toArray());
    } else if (visual.geo.kind === 'cylinder') {
      geo = new THREE.CylinderGeometry(visual.geo.radius, visual.geo.radius, visual.geo.length, 32);
      geo.rotateX(Math.PI / 2); // URDF 圆柱沿 z 轴，three 默认沿 y
    } else {
      geo = new THREE.SphereGeometry(visual.geo.radius, 32, 16);
    }

    const m = new THREE.Mesh(geo, visual.rgba ? materialFromRgba(visual.rgba) : materialFor(matName, idx));
    m.castShadow = true;
    m.receiveShadow = true;
    m.position.copy(visual.origin.pos);
    m.quaternion.copy(visual.origin.quat);
    return m;
  }

  const buildLink = async (linkName, idx) => {
    const spec = linkSpecs.get(linkName);
    const group = new THREE.Group();
    group.name = linkName;
    const info = { obj: group, meshes: [], spec };
    robot.links.set(linkName, info);

    for (const visual of (spec && spec.visuals) || []) {
      const mesh = await buildMesh(visual, linkName, idx);
      mesh.userData.link = linkName;
      group.add(mesh);
      info.meshes.push(mesh);
      robot.pickables.push(mesh);
    }

    for (const js of jointsByParent.get(linkName) || []) {
      const joint = new Joint(js);
      const jObj = new THREE.Object3D();
      jObj.name = joint.name;
      joint.obj = jObj;
      joint.apply();
      group.add(jObj);
      robot.joints.push(joint);
      robot.linkJoint.set(joint.childName, joint);
      jObj.add(await buildLink(joint.childName, idx + 1));
    }
    return group;
  };

  robot.root.add(await buildLink(rootName, 0));

  if (CFG.zUpToYUp) robot.root.rotation.x = -Math.PI / 2;

  /* ---- 末端工具点：最后一级 link 上离原点最远的顶点 ---- */
  const lastJoint = robot.joints.filter((j) => j.movable).slice(-1)[0];
  const tipLink = lastJoint ? lastJoint.childName : rootName;
  const tipInfo = robot.links.get(tipLink);
  const tip = computeToolTip((tipInfo && tipInfo.meshes) || []);

  robot.eeTipLocal = tip.length();
  const ee = new THREE.Object3D();
  ee.position.copy(tip);
  if (tipInfo) tipInfo.obj.add(ee);
  robot.ee = ee;

  // 末端标记（跟着夹爪跑）
  const marker = new THREE.Group();
  marker.add(
    new THREE.Mesh(
      new THREE.SphereGeometry(0.0055, 16, 12),
      new THREE.MeshBasicMaterial({ color: 0x39c5bb })
    )
  );
  marker.add(
    new THREE.Mesh(
      new THREE.RingGeometry(0.009, 0.011, 32),
      new THREE.MeshBasicMaterial({
        color: 0x39c5bb,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.65,
      })
    )
  );
  ee.add(marker);
  robot.marker = marker;

  robot.root.updateMatrixWorld(true);
  return robot;
}

/* ==================================================================
   4. 场景
   ================================================================== */

const canvas = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.05;
renderer.outputColorSpace = THREE.SRGBColorSpace;

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0e1116);
scene.fog = new THREE.Fog(0x0e1116, 1.8, 5.5);

// 想要浅色背景截图：?bg=f2f4f7
const bgParam = new URLSearchParams(location.search).get('bg');
if (bgParam && /^#?[0-9a-f]{6}$/i.test(bgParam)) {
  const c = new THREE.Color(`#${bgParam.replace('#', '')}`);
  scene.background = c;
  scene.fog = new THREE.Fog(c, 1.8, 5.5);
  document.body.style.background = `#${c.getHexString()}`;
}

const camera = new THREE.PerspectiveCamera(40, innerWidth / innerHeight, 0.005, 100);
camera.position.set(0.55, 0.42, 0.55);

const controls = new OrbitControls(camera, canvas);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.rotateSpeed = 0.85;
controls.zoomSpeed = 0.9;
controls.panSpeed = 0.8;
controls.minDistance = 0.08;
controls.maxDistance = 8;
controls.target.set(0, 0.18, 0);

/* ---- 环境贴图：拿 canvas 画个假影棚，金属件才不会发黑 ---- */
function makeStudioEnv() {
  const c = document.createElement('canvas');
  c.width = 512;
  c.height = 256;
  const g = c.getContext('2d');
  const grd = g.createLinearGradient(0, 0, 0, 256);
  grd.addColorStop(0, '#7b8ea6');
  grd.addColorStop(0.45, '#39434f');
  grd.addColorStop(1, '#0d1116');
  g.fillStyle = grd;
  g.fillRect(0, 0, 512, 256);

  const blob = (x, y, rx, ry, fill) => {
    const rg = g.createRadialGradient(x, y, 0, x, y, Math.max(rx, ry));
    rg.addColorStop(0, fill);
    rg.addColorStop(1, 'rgba(0,0,0,0)');
    g.fillStyle = rg;
    g.beginPath();
    g.ellipse(x, y, rx, ry, 0, 0, Math.PI * 2);
    g.fill();
  };
  blob(150, 40, 130, 55, 'rgba(255,255,255,0.95)');
  blob(380, 60, 110, 70, 'rgba(150,215,255,0.75)');
  blob(300, 210, 220, 60, 'rgba(90,110,130,0.55)');

  const tex = new THREE.CanvasTexture(c);
  tex.mapping = THREE.EquirectangularReflectionMapping;
  tex.colorSpace = THREE.SRGBColorSpace;
  const pmrem = new THREE.PMREMGenerator(renderer);
  const env = pmrem.fromEquirectangular(tex).texture;
  tex.dispose();
  pmrem.dispose();
  return env;
}
scene.environment = makeStudioEnv();

/* ---- 灯光 ---- */
scene.add(new THREE.HemisphereLight(0xdfe9f5, 0x141a22, 1.15));

const keyLight = new THREE.DirectionalLight(0xffffff, 2.1);
keyLight.position.set(1.6, 2.4, 1.4);
keyLight.castShadow = true;
keyLight.shadow.mapSize.set(4096, 4096);
keyLight.shadow.bias = -0.0004;
keyLight.shadow.normalBias = 0.0012;
const sc = keyLight.shadow.camera;
sc.near = 0.05;
sc.far = 8;
sc.left = -0.9;
sc.right = 0.9;
sc.top = 0.9;
sc.bottom = -0.9;
scene.add(keyLight);

const fillLight = new THREE.DirectionalLight(0x8fb8e0, 0.75);
fillLight.position.set(-1.8, 1.1, -1.2);
scene.add(fillLight);

const rimLight = new THREE.DirectionalLight(0xff9ec4, 0.55);
rimLight.position.set(-0.6, 0.8, -2.2);
scene.add(rimLight);

/* ---- 地面 ---- */
const ground = new THREE.Mesh(
  new THREE.PlaneGeometry(6, 6),
  new THREE.MeshStandardMaterial({ color: 0x171e26, roughness: 0.94, metalness: 0.04 })
);
ground.rotation.x = -Math.PI / 2;
ground.position.y = 0;
ground.receiveShadow = true;
scene.add(ground);

const grid = new THREE.GridHelper(3, 60, 0x4a7fa8, 0x2a3947);
grid.position.y = 0.0015;
grid.material.transparent = true;
grid.material.opacity = 0.8;
scene.add(grid);

const axes = new THREE.AxesHelper(0.25);
axes.setColors(new THREE.Color(0xff6b81), new THREE.Color(0x6ee7a0), new THREE.Color(0x6cb6ff));
axes.position.y = 0.001;
axes.visible = false;
scene.add(axes);

/* ---- 拖末端时的提示 ---- */
const dragHint = new THREE.Group();
const dragBall = new THREE.Mesh(
  new THREE.SphereGeometry(0.008, 24, 16),
  new THREE.MeshBasicMaterial({ color: 0x39c5bb, transparent: true, opacity: 0.9 })
);
dragHint.add(dragBall);
const dragLine = new THREE.Line(
  new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(), new THREE.Vector3()]),
  new THREE.LineBasicMaterial({ color: 0x39c5bb, transparent: true, opacity: 0.55 })
);
dragHint.add(dragLine);
dragHint.visible = false;
scene.add(dragHint);

/* ==================================================================
   5. 载入机器人
   ================================================================== */

let robot = null;
const ui = {
  eePos: document.getElementById('eePos'),
  modeTip: document.getElementById('modeTip'),
  joints: document.getElementById('joints'),
};

boot('正在解析 URDF…');

loadRobot(CFG.urdfUrl)
  .then((r) => {
    robot = r;
    scene.add(robot.root);
    frameShadow();
    applyPoseDeg(getPoseFromUrl() || CFG.homePose);
    fitView();
    buildJointUI();
    const modelSel = document.getElementById('modelSel');
    if (modelSel) {
      const cur = CFG.urdfUrl;
      if ([...modelSel.options].some((o) => o.value === cur)) modelSel.value = cur;
      modelSel.addEventListener('change', () => {
        const u = new URL(location.href);
        u.searchParams.set('urdf', modelSel.value);
        u.searchParams.delete('q');
        location.href = u.toString();
      });
    }
    bootDone();

    if (new URLSearchParams(location.search).has('dbg')) dumpDebug();
    if (new URLSearchParams(location.search).has('selftest')) runSelfTest();

    window.__viewer = {
      robot,
      scene,
      camera,
      setJoints: (deg) => applyPoseDeg(deg),
      getJoints: () => robot.getPoseRad().map((v) => +(v * DEG).toFixed(2)),
      fit: fitView,
      ready: true,
    };
    console.log('[viewer] 关节数', robot.movableJoints.length, '可视网格', robot.pickables.length);
  })
  .catch((err) => {
    console.error(err);
    bootError(err);
  });

/** ?dbg=1：关节/网格的世界坐标写进 <body data-dbg>，方便无头截图时对账 */
function dumpDebug() {
  const out = { joints: [], meshes: [] };
  for (const j of robot.joints) {
    const p = j.obj.getWorldPosition(new THREE.Vector3());
    out.joints.push({ name: j.name, pos: [+p.x.toFixed(4), +p.y.toFixed(4), +p.z.toFixed(4)] });
  }
  for (const m of robot.pickables) {
    const b = new THREE.Box3().setFromObject(m);
    const c = b.getCenter(new THREE.Vector3());
    const s = b.getSize(new THREE.Vector3());
    out.meshes.push({
      link: m.userData.link,
      center: [+c.x.toFixed(3), +c.y.toFixed(3), +c.z.toFixed(3)],
      size: [+s.x.toFixed(3), +s.y.toFixed(3), +s.z.toFixed(3)],
    });
  }
  document.body.dataset.dbg = JSON.stringify(out);
}

function boot(msg) {
  const el = document.getElementById('bootMsg');
  if (el) el.textContent = msg;
}
function bootDone() {
  const el = document.getElementById('boot');
  el.classList.add('done');
  setTimeout(() => el.remove(), 600);
}
function bootError(err) {
  const el = document.getElementById('boot');
  const msg = String((err && err.message) || err).replace(/[<>&]/g, (c) => ({ '<': '&lt;', '>': '&gt;', '&': '&amp;' }[c]));
  el.innerHTML =
    '<div class="box"><div class="msg err">模型载入失败\n\n' +
    msg +
    '\n\n请确认是通过本地服务器打开（不要直接双击 index.html）。</div></div>';
}

/* ==================================================================
   6. 视角 / 阴影范围
   ================================================================== */

function frameShadow() {
  const box = new THREE.Box3().setFromObject(robot.root);
  const size = box.getSize(new THREE.Vector3());
  // 阴影相机要盖住整块地面，否则区域外会冒出一条硬边
  const r = THREE.MathUtils.clamp(Math.max(size.x, size.y, size.z) * 2.6, 0.8, 3.0);
  const cam = keyLight.shadow.camera;
  cam.left = -r;
  cam.right = r;
  cam.top = r;
  cam.bottom = -r;
  cam.far = r * 8;
  cam.updateProjectionMatrix();
}

function fitView() {
  const box = new THREE.Box3().setFromObject(robot.root);
  const size = box.getSize(new THREE.Vector3());
  const center = box.getCenter(new THREE.Vector3());

  // 按最大边 + 画幅比例算距离，长条形模型才不会被推得太远
  const maxDim = Math.max(size.x, size.y, size.z, 0.1);
  const fovY = camera.fov * RAD;
  const distV = maxDim / 2 / Math.tan(fovY / 2);
  const distH = maxDim / 2 / (Math.tan(fovY / 2) * Math.max(camera.aspect, 0.3));
  const dist = Math.max(distV, distH) * 2.0;

  controls.target.copy(center);
  camera.position.copy(center).add(new THREE.Vector3(0.75, 0.25, 1).normalize().multiplyScalar(dist));
  camera.near = Math.max(dist / 800, 0.002);
  camera.far = dist * 12;
  camera.updateProjectionMatrix();
  controls.update();
}

/* ==================================================================
   7. 关节面板
   ================================================================== */

const uiRefs = []; // { joint, slider, input, label }

function buildJointUI() {
  ui.joints.querySelectorAll('.joint').forEach((n) => n.remove());

  robot.movableJoints.forEach((joint, i) => {
    const row = document.createElement('div');
    row.className = 'joint';

    const label = document.createElement('div');
    label.className = 'jn';
    label.textContent = `J${i + 1}`;
    label.title = joint.name;

    const slider = document.createElement('input');
    slider.type = 'range';
    slider.min = String(joint.min * DEG);
    slider.max = String(joint.max * DEG);
    slider.step = '0.5';

    const input = document.createElement('input');
    input.type = 'number';
    input.step = '1';
    input.min = String(Math.round(joint.min * DEG));
    input.max = String(Math.round(joint.max * DEG));

    const onInput = (v) => {
      const rad = THREE.MathUtils.clamp(v * RAD, joint.min, joint.max);
      joint.value = rad;
      joint.apply();
      robot.root.updateMatrixWorld(true);
      slider.value = String(rad * DEG);
      input.value = String(+(rad * DEG).toFixed(1));
      updateReadout();
    };

    slider.addEventListener('input', () => onInput(parseFloat(slider.value)));
    input.addEventListener('change', () => onInput(num(input.value, joint.value * DEG)));

    row.append(label, slider, input);
    ui.joints.appendChild(row);
    uiRefs.push({ joint, slider, input, label });
  });

  refreshJointUI();
}

function refreshJointUI() {
  for (const ref of uiRefs) {
    const deg = ref.joint.value * DEG;
    ref.slider.value = String(deg);
    ref.input.value = String(+deg.toFixed(1));
  }
  updateReadout();
}

function applyPoseDeg(degArr) {
  robot.setPoseRad(degArr.map((d) => d * RAD));
  refreshJointUI();
}

function getPoseFromUrl() {
  const q = new URLSearchParams(location.search).get('q');
  if (!q) return null;
  const arr = q.split(',').map(Number).filter((n) => Number.isFinite(n));
  return arr.length ? arr : null;
}

const _eeV = new THREE.Vector3();
function updateReadout() {
  if (!robot) return;
  robot.root.updateMatrixWorld(true);
  robot.eeWorld(_eeV);
  const mm = (v) => (v * 1000).toFixed(0).padStart(4, ' ');
  ui.eePos.textContent = `X ${mm(_eeV.x)}  Y ${mm(_eeV.y)}  Z ${mm(_eeV.z)} mm`;
}

/* ==================================================================
   8. 拖拽交互
   ================================================================== */

const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
let drag = null;
let mode = 'joint';

const _q1 = new THREE.Quaternion();

function updatePointer(ev) {
  const rect = canvas.getBoundingClientRect();
  pointer.x = ((ev.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((ev.clientY - rect.top) / rect.height) * 2 + 1;
}

function pickLink(ev) {
  updatePointer(ev);
  raycaster.setFromCamera(pointer, camera);
  const hits = raycaster.intersectObjects(robot.pickables, false);
  return hits[0] || null;
}

function toScreenPx(v3) {
  const p = v3.clone().project(camera);
  return new THREE.Vector2((p.x * 0.5 + 0.5) * innerWidth, (-p.y * 0.5 + 0.5) * innerHeight);
}

function onPointerDown(ev) {
  if (ev.button !== 0 || !robot) return;
  if (ev.target !== canvas) return; // 点在面板/按钮上时不管

  const hit = pickLink(ev);
  if (!hit) return;

  const joint = robot.linkJoint.get(hit.object.userData.link);

  if (mode === 'joint') {
    if (!joint || joint.type === 'fixed') {
      flashPanelHint('这个部件是固定的，拖不动');
      return;
    }
    drag = makeJointDrag(ev, hit, joint);
  } else {
    drag = makeEEDrag();
  }
  if (!drag) return;

  controls.enabled = false;
  canvas.style.cursor = 'grabbing';
  document.body.classList.add('dragging');
  if (drag.joint) highlightJoint(drag.joint);
  try {
    canvas.setPointerCapture(ev.pointerId);
  } catch {
    /* 合成事件没有真实 pointerId，忽略 */
  }
  ev.preventDefault();
  ev.stopPropagation(); // 别让 OrbitControls 同时接管这次左键
}

function onPointerMove(ev) {
  if (drag) {
    if (drag.kind === 'joint') moveJointDrag(ev);
    else moveEEDrag(ev);
    return;
  }
  if (!robot || ev.target !== canvas) return;
  const hit = pickLink(ev);
  canvas.style.cursor = hit ? 'grab' : 'default';
}

function onPointerUp(ev) {
  if (!drag) return;
  drag = null;
  controls.enabled = true;
  canvas.style.cursor = 'default';
  document.body.classList.remove('dragging');
  dragHint.visible = false;
  highlightJoint(null);
  try {
    if (canvas.hasPointerCapture(ev.pointerId)) canvas.releasePointerCapture(ev.pointerId);
  } catch {
    /* 同上 */
  }
}

// 监听放在 window 捕获阶段：拖动时先于 OrbitControls 拿到事件，鼠标划出画布也不丢
window.addEventListener('pointerdown', onPointerDown, true);
window.addEventListener('pointermove', onPointerMove);
window.addEventListener('pointerup', onPointerUp);
window.addEventListener('pointercancel', onPointerUp);

/* ---------------- 关节拖拽 ---------------- */

function makeJointDrag(ev, hit, joint) {
  const pivot = joint.obj.getWorldPosition(new THREE.Vector3());
  const axis = joint.axis
    .clone()
    .applyQuaternion(joint.obj.getWorldQuaternion(new THREE.Quaternion()))
    .normalize();
  const radiusVec = hit.point.clone().sub(pivot);
  const lever = Math.max(radiusVec.length(), 0.02);

  // 正向 θ 时这个点往哪走
  const tangent = new THREE.Vector3().crossVectors(axis, radiusVec).normalize();
  const sHit = toScreenPx(hit.point);
  const sTan = toScreenPx(hit.point.clone().addScaledVector(tangent, 0.05)).sub(sHit);

  let gain; // 每像素多少弧度
  if (sTan.length() > 6) {
    const metersPerPx = 0.05 / sTan.length();
    gain = (metersPerPx / lever) * 0.6; // 0.6 是阻尼，给 1.0 会有点飘
  } else {
    gain = 0.008; // 关节轴几乎正对镜头时兜底
  }
  gain = THREE.MathUtils.clamp(gain, 0.0012, 0.05);

  return {
    kind: 'joint',
    joint,
    gain,
    tanHat: sTan.length() > 1 ? sTan.clone().normalize() : new THREE.Vector2(1, 0),
    last: new THREE.Vector2(ev.clientX, ev.clientY),
  };
}

function moveJointDrag(ev) {
  const d = new THREE.Vector2(ev.clientX - drag.last.x, ev.clientY - drag.last.y);
  drag.last.set(ev.clientX, ev.clientY);
  const along = d.dot(drag.tanHat);
  if (!along) return;
  const j = drag.joint;
  j.value = THREE.MathUtils.clamp(j.value + along * drag.gain, j.min, j.max);
  j.apply();
  robot.root.updateMatrixWorld(true);
  refreshJointUI();
}

/* ---------------- 末端 IK 拖拽 ---------------- */

function makeEEDrag() {
  const eePos = robot.eeWorld(new THREE.Vector3());
  const camDir = new THREE.Vector3();
  camera.getWorldDirection(camDir);
  const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(camDir, eePos);

  dragHint.visible = true;
  dragBall.position.copy(eePos);
  updateDragLine(eePos, eePos);
  return { kind: 'ee', plane, eePos: eePos.clone(), target: eePos.clone() };
}

function moveEEDrag(ev) {
  updatePointer(ev);
  raycaster.setFromCamera(pointer, camera);
  const hit = raycaster.ray.intersectPlane(drag.plane, new THREE.Vector3());
  if (!hit) return;

  drag.target.copy(hit);
  solveIK(hit);
  refreshJointUI();

  const eeNow = robot.eeWorld(new THREE.Vector3());
  dragBall.position.copy(hit);
  updateDragLine(eeNow, hit);
}

function updateDragLine(a, b) {
  const pos = dragLine.geometry.attributes.position;
  pos.setXYZ(0, a.x, a.y, a.z);
  pos.setXYZ(1, b.x, b.y, b.z);
  pos.needsUpdate = true;
  dragLine.geometry.computeBoundingSphere();
}

/* ---------------- 阻尼最小二乘 IK（只管位置） ---------------- */

const _ikA = new THREE.Matrix3();
const _ikR = new THREE.Vector3();
const _ikAxis = new THREE.Vector3();
const _ikJ = new THREE.Vector3();
const _ikErr = new THREE.Vector3();
const _ikDq = [];

function solveIK(target, iterations = 24) {
  const joints = robot.movableJoints.filter((j) => j.type !== 'prismatic');
  if (!joints.length) return;

  const lambda2 = 1e-4;
  const maxStep = 0.3; // rad / 迭代
  _ikDq.length = joints.length;

  for (let it = 0; it < iterations; it++) {
    robot.root.updateMatrixWorld(true);
    const pEE = robot.eeWorld(new THREE.Vector3());
    _ikErr.copy(target).sub(pEE);
    if (_ikErr.length() < 3e-4) break;

    // J·Jᵀ
    const A = new Float64Array(9);
    const cols = [];
    for (const j of joints) {
      const origin = j.obj.getWorldPosition(new THREE.Vector3());
      _ikAxis.copy(j.axis).applyQuaternion(j.obj.getWorldQuaternion(_q1)).normalize();
      _ikR.copy(pEE).sub(origin);
      _ikJ.crossVectors(_ikAxis, _ikR); // 这个关节给末端的瞬时速度方向
      cols.push(_ikJ.clone());
      const x = _ikJ.x;
      const y = _ikJ.y;
      const z = _ikJ.z;
      A[0] += x * x;
      A[1] += x * y;
      A[2] += x * z;
      A[3] += y * x;
      A[4] += y * y;
      A[5] += y * z;
      A[6] += z * x;
      A[7] += z * y;
      A[8] += z * z;
    }
    A[0] += lambda2;
    A[4] += lambda2;
    A[8] += lambda2;

    _ikA.set(A[0], A[1], A[2], A[3], A[4], A[5], A[6], A[7], A[8]).invert();
    const e = _ikA.elements;
    const x = e[0] * _ikErr.x + e[3] * _ikErr.y + e[6] * _ikErr.z;
    const y = e[1] * _ikErr.x + e[4] * _ikErr.y + e[7] * _ikErr.z;
    const z = e[2] * _ikErr.x + e[5] * _ikErr.y + e[8] * _ikErr.z;

    let maxDq = 0;
    joints.forEach((j, i) => {
      const dq = cols[i].x * x + cols[i].y * y + cols[i].z * z;
      _ikDq[i] = dq;
      maxDq = Math.max(maxDq, Math.abs(dq));
    });
    const scale = maxDq > maxStep ? maxStep / maxDq : 1;

    joints.forEach((j, i) => {
      j.value = THREE.MathUtils.clamp(j.value + _ikDq[i] * scale, j.min, j.max);
      j.apply();
    });
  }
  robot.root.updateMatrixWorld(true);
}

/* ---------------- 高亮 / 提示 ---------------- */

let hotJoint = null;
function highlightJoint(joint) {
  if (hotJoint === joint) return;
  for (const ref of uiRefs) ref.label.classList.toggle('hot', ref.joint === joint);
  hotJoint = joint;
}

let hintTimer = 0;
function flashPanelHint(text) {
  ui.modeTip.textContent = text;
  clearTimeout(hintTimer);
  hintTimer = setTimeout(() => {
    ui.modeTip.textContent = mode === 'joint' ? '关节拖拽' : '末端拖拽';
  }, 1600);
}

/* ==================================================================
   9. 面板事件
   ================================================================== */

document.getElementById('modeSeg').addEventListener('click', (e) => {
  const btn = e.target.closest('button[data-mode]');
  if (!btn) return;
  mode = btn.dataset.mode;
  [...btn.parentNode.children].forEach((b) => b.classList.toggle('on', b === btn));
  ui.modeTip.textContent = mode === 'joint' ? '关节拖拽' : '末端拖拽';
  dragHint.visible = false;
});

document.getElementById('btnZero').addEventListener('click', () => applyPoseDeg([0, 0, 0, 0, 0, 0]));
document.getElementById('btnHome').addEventListener('click', () => applyPoseDeg(CFG.homePose));
document.getElementById('btnFit').addEventListener('click', fitView);

document.getElementById('btnRand').addEventListener('click', () => {
  applyPoseDeg(
    robot.movableJoints.map((j) => {
      const mid = ((j.min + j.max) / 2) * DEG;
      const span = ((j.max - j.min) / 2) * DEG;
      return mid + (Math.random() - 0.5) * span * 1.1;
    })
  );
});

document.getElementById('tGrid').addEventListener('change', (e) => {
  grid.visible = e.target.checked;
});
document.getElementById('tAxes').addEventListener('change', (e) => {
  axes.visible = e.target.checked;
});
document.getElementById('tShadow').addEventListener('change', (e) => {
  renderer.shadowMap.enabled = e.target.checked;
  ground.visible = e.target.checked;
  scene.traverse((o) => {
    if (o.isMesh && o.material) o.material.needsUpdate = true;
  });
});
document.getElementById('tSpin').addEventListener('change', (e) => {
  controls.autoRotate = e.target.checked;
  controls.autoRotateSpeed = 1.2;
});
document.getElementById('tMarker').addEventListener('change', (e) => {
  if (robot && robot.marker) robot.marker.visible = e.target.checked;
  if (!e.target.checked) dragHint.visible = false;
});

document.getElementById('tWire').addEventListener('change', (e) => {
  const on = e.target.checked;
  robot.pickables.forEach((m) => {
    m.material.wireframe = on;
    m.material.needsUpdate = true;
  });
});

document.getElementById('btnShot').addEventListener('click', () => {
  dragHint.visible = false;
  renderer.render(scene, camera);
  canvas.toBlob((blob) => {
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = `miku-arm-${new Date().toISOString().slice(0, 19).replace(/[:T]/g, '')}.png`;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 4000);
  }, 'image/png');
});

document.getElementById('btnCopy').addEventListener('click', async () => {
  const text = JSON.stringify(robot.getPoseRad().map((v) => +(v * DEG).toFixed(2)));
  try {
    await navigator.clipboard.writeText(text);
    flashPanelHint('已复制关节角');
  } catch {
    flashPanelHint('复制失败，值见控制台');
  }
  console.log('[pose deg]', text, '\n[pose rad]', robot.getPoseRad());
});

/* 面板折叠 */
const panel = document.getElementById('panel');
const toggleBtn = document.getElementById('toggle');
document.getElementById('hide').addEventListener('click', () => {
  panel.classList.add('closed');
  toggleBtn.classList.add('show');
});
toggleBtn.addEventListener('click', () => {
  panel.classList.remove('closed');
  toggleBtn.classList.remove('show');
});

/* 键盘：R 复位姿态，F 复位视角，1-6 选关节，Q/E 微调 */
let selJoint = 0;
addEventListener('keydown', (e) => {
  if (!robot || e.target.matches('input, textarea')) return;
  if (e.key === 'r' || e.key === 'R') applyPoseDeg(CFG.homePose);
  if (e.key === 'f' || e.key === 'F') fitView();

  const n = parseInt(e.key, 10);
  if (n >= 1 && n <= uiRefs.length) {
    selJoint = n - 1;
    highlightJoint(uiRefs[selJoint].joint);
    return;
  }
  if (e.key === 'q' || e.key === 'e') {
    const ref = uiRefs[selJoint];
    if (!ref) return;
    const j = ref.joint;
    const step = (e.key === 'e' ? 1 : -1) * 8 * RAD;
    j.value = THREE.MathUtils.clamp(j.value + step, j.min, j.max);
    j.apply();
    robot.root.updateMatrixWorld(true);
    refreshJointUI();
  }
});

/* ==================================================================
   10. 自检（?selftest=1，无头浏览器里跑，结果写进 <body data-test>）
   ================================================================== */

async function runSelfTest() {
  const out = {};
  const home = () => robot.setPoseRad(CFG.homePose.map((d) => d * RAD));
  const fire = (type, x, y) =>
    canvas.dispatchEvent(
      new PointerEvent(type, {
        clientX: x,
        clientY: y,
        button: 0,
        buttons: 1,
        bubbles: true,
        pointerId: 1,
      })
    );

  // 1) IK 精度
  home();
  robot.root.updateMatrixWorld(true);
  const start = robot.eeWorld(new THREE.Vector3());
  const targets = [
    start.clone().add(new THREE.Vector3(0.06, 0.03, 0)),
    start.clone().add(new THREE.Vector3(-0.05, -0.07, 0.04)),
    new THREE.Vector3(0.16, 0.22, 0.18),
  ];
  out.ik = targets.map((t) => {
    home();
    solveIK(t, 90);
    const p = robot.eeWorld(new THREE.Vector3());
    return {
      target: t.toArray().map((v) => +v.toFixed(3)),
      got: p.toArray().map((v) => +v.toFixed(3)),
      errMm: +(p.distanceTo(t) * 1000).toFixed(2),
    };
  });

  // 2) 关节拖拽
  home();
  robot.root.updateMatrixWorld(true);
  const mesh4 = robot.links.get('link_4').meshes[0];
  const s4 = toScreenPx(new THREE.Box3().setFromObject(mesh4).getCenter(new THREE.Vector3()));
  const poseBefore = robot.getPoseRad().map((v) => +(v * DEG).toFixed(2));
  fire('pointerdown', s4.x, s4.y);
  const grabbed = !!drag;
  const grabbedJoint = drag && drag.joint ? drag.joint.name : null;
  fire('pointermove', s4.x + 70, s4.y + 10);
  fire('pointerup', s4.x + 70, s4.y + 10);
  const poseAfter = robot.getPoseRad().map((v) => +(v * DEG).toFixed(2));
  out.jointDrag = {
    grabbed,
    grabbedJoint,
    poseBefore,
    poseAfter,
    changed: poseAfter.filter((v, i) => Math.abs(v - poseBefore[i]) > 0.01).length,
  };

  // 3) 末端拖拽方向扫描
  const mesh6 = robot.links.get('link_6').meshes[0];
  out.eeSweep = [
    [110, 0],
    [-110, 0],
    [0, -110],
    [0, 110],
    [80, 80],
  ].map(([dx, dy]) => {
    mode = 'ee';
    home();
    robot.root.updateMatrixWorld(true);
    const s = toScreenPx(new THREE.Box3().setFromObject(mesh6).getCenter(new THREE.Vector3()));
    const p0 = robot.eeWorld(new THREE.Vector3());
    fire('pointerdown', s.x, s.y);
    fire('pointermove', s.x + dx, s.y + dy);
    const t = drag ? drag.target.clone() : p0;
    fire('pointerup', s.x + dx, s.y + dy);
    const p1 = robot.eeWorld(new THREE.Vector3());
    return {
      drag: [dx, dy],
      movedMm: +(p0.distanceTo(p1) * 1000).toFixed(1),
      targetDistMm: +(p0.distanceTo(t) * 1000).toFixed(1),
      errMm: +(p1.distanceTo(t) * 1000).toFixed(1),
    };
  });

  mode = 'joint';
  home();
  document.body.dataset.test = JSON.stringify(out);
  console.log('[selftest]', out);
}

/* ==================================================================
   11. 渲染循环
   ================================================================== */

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});

let lastReadout = 0;
function tick(t) {
  requestAnimationFrame(tick);
  controls.update();
  renderer.render(scene, camera);
  if (robot && t - lastReadout > 120) {
    lastReadout = t;
    updateReadout();
  }
}
requestAnimationFrame(tick);


