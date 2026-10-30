// Frontend minimo del editor URDF: solo preview local, sin conexion a ROS ni hardware.
import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import { STLLoader } from 'three/examples/jsm/loaders/STLLoader.js';
import URDFLoader from '/static/vendor/urdf-loader/src/URDFLoader.js';

const $ = (id) => document.getElementById(id);
const original = {};   // nombre -> valor numerico del archivo
const current = {};    // nombre -> valor en la UI
const inputs = {};     // nombre -> [range, number]
let robot = null;
let meshWarnings = [];
let serverWarnings = [];
let timer = null;
let requestSeq = 0;

// ------------------------------------------------------------------ escena 3D
const viewer = $('viewer');
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
viewer.appendChild(renderer.domElement);
const scene = new THREE.Scene();
scene.background = new THREE.Color(0xf2f2f2);
const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 50);
camera.position.set(2.0, 1.4, 2.0);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0, 0.7, 0);
scene.add(new THREE.HemisphereLight(0xffffff, 0x666666, 2.0));
const sun = new THREE.DirectionalLight(0xffffff, 1.5);
sun.position.set(3, 5, 2);
scene.add(sun);
scene.add(new THREE.GridHelper(4, 40, 0x888888, 0xcccccc));
const axes = new THREE.AxesHelper(0.3);   // ejes de three (Y arriba); el robot se rota a Z arriba
scene.add(axes);

function resize() {
  const { clientWidth: w, clientHeight: h } = viewer;
  renderer.setSize(w, h);
  camera.aspect = w / Math.max(h, 1);
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);
resize();
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });

function placeholder(material) {
  const mat = new THREE.MeshBasicMaterial({ color: 0xff00ff, wireframe: true });
  return new THREE.Mesh(new THREE.BoxGeometry(0.05, 0.05, 0.05), mat);
}

function loadMesh(path, manager, material, done) {
  if (!/\.stl$/i.test(path)) {
    meshWarnings.push(`Formato de mesh no soportado en preview: ${path} (placeholder)`);
    done(placeholder(material));
    return;
  }
  new STLLoader(manager).load(
    path,
    (geom) => done(new THREE.Mesh(geom, material || new THREE.MeshPhongMaterial())),
    undefined,
    () => {
      meshWarnings.push(`Mesh no cargado: ${path} (placeholder magenta)`);
      renderWarnings();
      done(placeholder(material));
    },
  );
}

function showUrdf(urdfText) {
  const loader = new URDFLoader();
  loader.packages = (pkg) => `/meshes/${pkg}`;
  loader.loadMeshCb = loadMesh;
  meshWarnings = [];
  const next = loader.parse(urdfText);
  next.rotation.x = -Math.PI / 2;   // URDF Z-up -> three.js Y-up
  if (robot) scene.remove(robot);
  robot = next;
  scene.add(robot);
}

// ------------------------------------------------------------------ UI
function setStatus(text, kind) {
  const el = $('status');
  el.textContent = text;
  el.className = `status ${kind || ''}`;
}

function renderWarnings() {
  const ul = $('warnings');
  ul.replaceChildren();
  for (const w of [...serverWarnings, ...meshWarnings]) {
    const li = document.createElement('li');
    li.textContent = w;
    ul.appendChild(li);
  }
}

function changedValues() {
  const out = {};
  for (const [name, v] of Object.entries(current)) {
    if (Math.abs(v - original[name]) > 1e-9) out[name] = v;
  }
  return out;
}

async function postJson(url, body) {
  const res = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  const data = await res.json().catch(() => ({}));
  if (!res.ok) {
    const d = data.detail;
    const msg = d && d.errors ? d.errors.join('\n') : (typeof d === 'string' ? d : res.statusText);
    throw new Error(msg);
  }
  return data;
}

function applyResult(data) {
  serverWarnings = data.warnings || [];
  $('diff').textContent = data.diff || '(sin cambios)';
  $('check').textContent = data.check_urdf ? data.check_urdf.output : '';
  showUrdf(data.urdf);
  renderWarnings();
  setStatus(data.check_urdf && data.check_urdf.ok ? 'URDF valido (check_urdf OK)' : 'check_urdf con errores',
            data.check_urdf && data.check_urdf.ok ? 'ok' : 'err');
}

async function refreshPreview() {
  const seq = ++requestSeq;
  setStatus('generando preview…');
  try {
    const data = await postJson('/api/model/preview', { values: changedValues() });
    if (seq === requestSeq) applyResult(data);
  } catch (err) {
    if (seq === requestSeq) {
      serverWarnings = [`Rechazado: ${err.message}`];
      renderWarnings();
      setStatus('valor rechazado', 'err');
    }
  }
}

function schedulePreview() {
  clearTimeout(timer);
  timer = setTimeout(refreshPreview, 300);
}

function buildParams(params) {
  const root = $('params');
  root.replaceChildren();
  const groups = {};
  const readonly = [];
  for (const p of params) {
    if (!p.editable) { readonly.push(p); continue; }
    (groups[p.group] ||= []).push(p);
    original[p.name] = p.value;
    current[p.name] = p.value;
  }
  for (const [group, items] of Object.entries(groups)) {
    const fs = document.createElement('fieldset');
    const lg = document.createElement('legend');
    lg.textContent = group;
    fs.appendChild(lg);
    for (const p of items) {
      const row = document.createElement('label');
      row.className = 'param';
      row.title = `${p.name} [${p.min}, ${p.max}] ${p.unit}` + (p.warning ? `\n⚠ ${p.warning}` : '');
      const name = document.createElement('span');
      name.textContent = `${p.label}${p.warning ? ' ⚠' : ''}`;
      const range = Object.assign(document.createElement('input'),
        { type: 'range', min: p.min, max: p.max, step: p.step, value: p.value });
      const num = Object.assign(document.createElement('input'),
        { type: 'number', min: p.min, max: p.max, step: p.step, value: p.value });
      const unit = document.createElement('small');
      unit.textContent = p.unit;
      const sync = (src, dst) => src.addEventListener('input', () => {
        const v = Number(src.value);
        if (!Number.isFinite(v)) return;
        dst.value = src.value;
        current[p.name] = v;
        schedulePreview();
      });
      sync(range, num);
      sync(num, range);
      inputs[p.name] = [range, num];
      row.append(name, range, num, unit);
      fs.appendChild(row);
    }
    root.appendChild(fs);
  }
  const det = document.createElement('details');
  det.innerHTML = '<summary>Solo lectura (derivadas / protegidas)</summary>';
  const ul = document.createElement('ul');
  ul.className = 'readonly';
  for (const p of readonly) {
    const li = document.createElement('li');
    const tag = p.protected ? 'protegida' : (p.kind === 'derived' ? 'derivada' : 'no editable');
    li.textContent = `${p.name} = ${p.raw}  [${tag}]`;
    if (p.protected_reason) li.title = p.protected_reason;
    ul.appendChild(li);
  }
  det.appendChild(ul);
  root.appendChild(det);
}

$('restore').addEventListener('click', () => {
  for (const [name, [range, num]] of Object.entries(inputs)) {
    range.value = num.value = original[name];
    current[name] = original[name];
  }
  refreshPreview();
});

$('export').addEventListener('click', async () => {
  const values = changedValues();
  const label = $('label').value.trim() || undefined;
  if (!Object.keys(values).length && !confirm('No hay cambios. ¿Guardar candidato identico al original?')) return;
  if (!confirm('Se escribira un candidato SOLO en tools/urdf_editor/candidates/. El repo no se modifica. ¿Continuar?')) return;
  try {
    const data = await postJson('/api/model/export', { values, label });
    alert(`Candidato guardado:\n${data.files.join('\n')}`);
  } catch (err) {
    alert(`Exportacion rechazada:\n${err.message}`);
  }
});

async function init() {
  try {
    const res = await fetch('/api/params');
    buildParams((await res.json()).params);
    const urdf = await fetch('/api/model/urdf');
    if (!urdf.ok) throw new Error((await urdf.json()).detail || urdf.statusText);
    applyResult(await urdf.json());
  } catch (err) {
    setStatus(`error: ${err.message}`, 'err');
  }
}
init();
