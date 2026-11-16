// Endoscope view of the dVRK mini simulator.
// World units here are millimetres (the robots publish metres).
(function () {
"use strict";
const MM = 1000, FLOOR = -130;
const COLORS = { PSM1: 0xd9822b, PSM2: 0x3f9b4a };          // orange right, green left
const HEX = { PSM1: "#e08a36", PSM2: "#4fb35a", MTMR: "#e08a36", MTML: "#4fb35a" };
const SIDES = ["MTML", "MTMR"], ARM = { MTML: "PSM2", MTMR: "PSM1" };
const v3 = (a, s = MM) => new THREE.Vector3(a[0] * s, a[1] * s, a[2] * s);
const Y = new THREE.Vector3(0, 1, 0);

// ------------------------------------------------------------------ renderer
const view = document.getElementById("view");
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
renderer.setSize(innerWidth, innerHeight);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;
renderer.shadowMap.autoUpdate = false;
renderer.outputEncoding = THREE.sRGBEncoding;
renderer.autoClear = false;
view.appendChild(renderer.domElement);

const BG = 0x160504;
const scene = new THREE.Scene();
scene.fog = new THREE.Fog(BG, 140, 380);
const camera = new THREE.PerspectiveCamera(62, innerWidth / innerHeight, 1, 1500);
camera.up.set(0, 0, 1);
camera.position.set(0, -45, -43);
camera.lookAt(0, 0, -125);

scene.add(new THREE.HemisphereLight(0xffeee8, 0x3a0f0c, 0.35));
const sun = new THREE.DirectionalLight(0xffffff, 0.55);       // a light from above, for the shadows
sun.position.set(25, 10, 120);
sun.target.position.set(0, 0, FLOOR);
sun.castShadow = true;
sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -110, right: 110, top: 110, bottom: -110, near: 20, far: 400 });
sun.shadow.bias = -0.0006;
scene.add(sun, sun.target);
const headlight = new THREE.PointLight(0xfff3e6, 1.15, 420, 1.3);  // the endoscope's own light
scene.add(headlight);

// ------------------------------------------------------------------- tissue
function tissueTexture(base, seed) {
  const c = document.createElement("canvas"); c.width = c.height = 1024;
  const g = c.getContext("2d");
  let s = seed;
  const rnd = () => { s = (s * 16807) % 2147483647; return s / 2147483647; };
  g.fillStyle = base; g.fillRect(0, 0, 1024, 1024);
  const tiled = draw => { for (const ox of [-1024, 0, 1024]) for (const oy of [-1024, 0, 1024]) draw(ox, oy); };  // seamless
  for (let i = 0; i < 900; i++) {
    const x = rnd() * 1024, y = rnd() * 1024, r = 8 + rnd() * 70, e = 0.4 + rnd() * 0.6, rot = rnd() * 3;
    g.fillStyle = rnd() < 0.5 ? `rgba(255,200,185,${0.03 + rnd() * 0.06})` : `rgba(100,15,20,${0.03 + rnd() * 0.07})`;
    tiled((ox, oy) => { g.beginPath(); g.ellipse(x + ox, y + oy, r, r * e, rot, 0, Math.PI * 2); g.fill(); });
  }
  for (let i = 0; i < 70; i++) {                          // capillaries
    let x = rnd() * 1024, y = rnd() * 1024, a = rnd() * 6.28;
    const pts = [[x, y]];
    for (let k = 0; k < 40; k++) { a += (rnd() - 0.5) * 0.7; x += Math.cos(a) * 9; y += Math.sin(a) * 9; pts.push([x, y]); }
    g.strokeStyle = `rgba(120,10,40,${0.18 + rnd() * 0.25})`; g.lineWidth = 0.6 + rnd() * 1.6;
    tiled((ox, oy) => { g.beginPath(); pts.forEach(([px, py], j) => j ? g.lineTo(px + ox, py + oy) : g.moveTo(px + ox, py + oy)); g.stroke(); });
  }
  const t = new THREE.CanvasTexture(c);
  t.wrapS = t.wrapT = THREE.RepeatWrapping;
  t.encoding = THREE.sRGBEncoding;
  return t;
}
const floorTex = tissueTexture("#c86a5c", 7);
floorTex.repeat.set(2, 2);
const floorGeo = new THREE.PlaneGeometry(420, 420, 140, 140);
{ // flat where the work happens, gentle folds further away
  const p = floorGeo.attributes.position;
  for (let i = 0; i < p.count; i++) {
    const x = p.getX(i), y = p.getY(i), r = Math.hypot(x, y);
    const w = Math.min(1, Math.max(0, (r - 70) / 60));
    p.setZ(i, w * (6 * Math.sin(x / 23) * Math.cos(y / 31) + 4 * Math.sin((x + y) / 17)));
  }
  floorGeo.computeVertexNormals();
}
const floor = new THREE.Mesh(floorGeo, new THREE.MeshPhongMaterial({
  map: floorTex, bumpMap: floorTex, bumpScale: 0.6, shininess: 38, specular: 0x5a3030 }));
floor.position.z = FLOOR;
floor.receiveShadow = true;
scene.add(floor);
const domeTex = tissueTexture("#a4473f", 11); domeTex.repeat.set(4, 3);
const dome = new THREE.Mesh(new THREE.SphereGeometry(1, 64, 40),
  new THREE.MeshPhongMaterial({ map: domeTex, side: THREE.BackSide, shininess: 20, specular: 0x331515 }));
dome.scale.set(260, 220, 140);
dome.position.set(0, 0, FLOOR);
scene.add(dome);

// ----------------------------------------------- overview-only objects
// The overview camera looks at the scene from outside: hide the dome and
// show the abdominal wall, the endoscope and what it sees.
const overviewOnly = [];
const wall = new THREE.Mesh(new THREE.PlaneGeometry(260, 220),
  new THREE.MeshBasicMaterial({ color: 0xe9bca6, transparent: true, opacity: 0.18, side: THREE.DoubleSide, depthWrite: false }));
const ecmShaft = new THREE.Mesh(new THREE.CylinderGeometry(5, 5, 1, 24),
  new THREE.MeshPhongMaterial({ color: 0x9aa3aa, shininess: 80 }));
const frustum = new THREE.LineSegments(new THREE.BufferGeometry(),
  new THREE.LineBasicMaterial({ color: 0x7fd4ff }));
overviewOnly.push(wall, ecmShaft, frustum);
scene.add(wall, ecmShaft, frustum);
function updateFrustum() {              // a small pyramid showing what the endoscope sees
  const L = 45, h = Math.tan(THREE.MathUtils.degToRad(camera.fov / 2)) * L, w = h * camera.aspect;
  const c = [[-w, -h], [w, -h], [w, h], [-w, h]].map(([x, y]) => new THREE.Vector3(x, y, -L).applyQuaternion(camera.quaternion).add(camera.position));
  const o = camera.position, pts = [];
  for (let i = 0; i < 4; i++) pts.push(o, c[i], c[i], c[(i + 1) % 4]);
  frustum.geometry.setFromPoints(pts);
}
const ovCam = new THREE.PerspectiveCamera(36, 300 / 190, 5, 3000);
ovCam.up.set(0, 0, 1); ovCam.position.set(150, -230, 40); ovCam.lookAt(0, 0, -95);

// -------------------------------------------------------------- instruments
const metal = new THREE.MeshPhongMaterial({ color: 0xc3cad0, shininess: 140, specular: 0xffffff });
const darkMetal = new THREE.MeshPhongMaterial({ color: 0x2b3034, shininess: 60, specular: 0x666666 });
function setCylinder(mesh, a, b) {             // stretch a unit-height cylinder between two points
  const d = new THREE.Vector3().subVectors(b, a), len = d.length();
  mesh.position.copy(a).addScaledVector(d, 0.5);
  mesh.quaternion.setFromUnitVectors(Y, d.normalize());
  mesh.scale.set(1, len, 1);
}
function shadowed(m) { m.castShadow = true; return m; }
function makeInstrument(color) {
  const mat = new THREE.MeshPhongMaterial({ color, shininess: 70, specular: 0x555555 });
  const o = {
    shaft: shadowed(new THREE.Mesh(new THREE.CylinderGeometry(4.2, 4.2, 1, 32), mat)),
    collar: shadowed(new THREE.Mesh(new THREE.CylinderGeometry(4.3, 4.3, 1, 32), metal)),
    clevis: shadowed(new THREE.Mesh(new THREE.CylinderGeometry(2.4, 3.6, 1, 24), metal)),
    port: new THREE.Mesh(new THREE.CylinderGeometry(6, 6, 1, 32, 1, true), darkMetal),
    wrist: new THREE.Group(), pivots: [],
  };
  o.port.material = darkMetal.clone(); o.port.material.side = THREE.DoubleSide;
  const knuckle = shadowed(new THREE.Mesh(new THREE.CylinderGeometry(2.3, 2.3, 4.6, 20), metal));
  knuckle.rotation.z = Math.PI / 2;                       // axis along the wrist's x
  o.wrist.add(knuckle);
  for (const s of [1, -1]) {
    const pivot = new THREE.Group();
    const jawGeo = new THREE.BoxGeometry(1.5, 1.1, 10.5);
    jawGeo.translate(s * 0.55, 0, 5.2);
    const jaw = shadowed(new THREE.Mesh(jawGeo, metal));
    pivot.add(jaw); o.wrist.add(pivot); o.pivots.push([pivot, s]);
  }
  scene.add(o.shaft, o.collar, o.clevis, o.port, o.wrist);
  return o;
}
const inst = { PSM1: makeInstrument(COLORS.PSM1), PSM2: makeInstrument(COLORS.PSM2) };
const basis = new THREE.Matrix4();
function rotFromRows(R, flip) {                 // our matrices arrive as rows; columns are the axes
  const x = new THREE.Vector3(R[0][0], R[1][0], R[2][0]);
  const y = new THREE.Vector3(R[0][1], R[1][1], R[2][1]);
  const z = new THREE.Vector3(R[0][2], R[1][2], R[2][2]);
  if (flip) { y.negate(); z.negate(); }
  basis.makeBasis(x, y, z);
  return new THREE.Quaternion().setFromRotationMatrix(basis);
}
function updateInstrument(o, a) {
  const rcm = v3(a.rcm), wrist = v3(a.wrist);
  const u = new THREE.Vector3().subVectors(wrist, rcm).normalize();
  const shaftEnd = wrist.clone().addScaledVector(u, -7);
  setCylinder(o.shaft, rcm.clone().addScaledVector(u, -160), shaftEnd);
  setCylinder(o.collar, shaftEnd.clone().addScaledVector(u, -2.5), shaftEnd);
  setCylinder(o.clevis, shaftEnd, wrist);
  setCylinder(o.port, rcm.clone().addScaledVector(u, -25), rcm.clone().addScaledVector(u, 14));
  o.wrist.position.copy(wrist);
  o.wrist.quaternion.copy(rotFromRows(a.R));
  for (const [p, s] of o.pivots) p.rotation.y = s * a.jaw / 2;
}

// --------------------------------------------------------------- the scene
let vesselMeshes = [];
function buildVessels(lines) {
  for (const m of vesselMeshes) scene.remove(m);
  vesselMeshes = lines.map(l => {
    const curve = new THREE.CatmullRomCurve3(l.pts.map(p => v3(p)));
    const m = new THREE.Mesh(new THREE.TubeGeometry(curve, 96, l.r * MM, 16, false),
      new THREE.MeshPhongMaterial({ color: 0x7d0f1f, shininess: 90, specular: 0x772222, emissive: 0x000000 }));
    m.castShadow = m.receiveShadow = true;
    scene.add(m);
    return m;
  });
}
const pool = (make) => { const items = []; return n => { while (items.length < n) items.push(make()); items.forEach((m, i) => m.visible = i < n); return items; }; };
const targetPool = pool(() => {
  const g = new THREE.Group();
  g.add(new THREE.Mesh(new THREE.SphereGeometry(4, 24, 16),
    new THREE.MeshPhongMaterial({ color: 0xffe066, transparent: true, opacity: 0.28, depthWrite: false })));
  g.add(shadowed(new THREE.Mesh(new THREE.SphereGeometry(1.4, 16, 12),
    new THREE.MeshPhongMaterial({ color: 0xffd23f, emissive: 0x553f00 }))));
  scene.add(g); return g;
});
const beadPool = pool(() => {
  const m = shadowed(new THREE.Mesh(new THREE.SphereGeometry(2.5, 24, 16),
    new THREE.MeshPhongMaterial({ color: 0xf28c28, shininess: 90, specular: 0x666666 })));
  scene.add(m); return m;
});
const ringPool = pool(() => {
  const m = new THREE.Mesh(new THREE.TorusGeometry(7, 0.8, 12, 48),
    new THREE.MeshPhongMaterial({ color: 0x35d08a, emissive: 0x0c3d24, shininess: 60 }));
  m.receiveShadow = true; scene.add(m); return m;
});

// ------------------------------------------------------------ masters inset
const mScene = new THREE.Scene();
mScene.add(new THREE.HemisphereLight(0xffffff, 0x334455, 0.75));
const mLight = new THREE.DirectionalLight(0xffffff, 0.6); mLight.position.set(1, -2, 3); mScene.add(mLight);
const mCam = new THREE.PerspectiveCamera(34, 300 / 190, 0.01, 10);
mCam.up.set(0, 0, 1); mCam.position.set(0.08, -0.78, 0.36); mCam.lookAt(0, 0.12, 0.0);
const grey = new THREE.MeshPhongMaterial({ color: 0x8b9298 });
const black = new THREE.MeshPhongMaterial({ color: 0x1d2124 });
const gold = new THREE.MeshPhongMaterial({ color: 0xd4a63a, shininess: 80 });
const masters = {};
for (const s of SIDES) {
  const col = new THREE.MeshPhongMaterial({ color: HEX[s] });
  const m = {
    col,
    pillar: new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.018, 1, 12), grey),
    upper: new THREE.Mesh(new THREE.CylinderGeometry(0.016, 0.016, 1, 12), col),
    fore: new THREE.Mesh(new THREE.CylinderGeometry(0.013, 0.013, 1, 12), col),
    ring: new THREE.Mesh(new THREE.TorusGeometry(0.035, 0.004, 8, 32), black),
    handle: new THREE.Mesh(new THREE.CylinderGeometry(0.007, 0.009, 1, 12), black),
    f1: new THREE.Mesh(new THREE.BoxGeometry(0.006, 0.065, 0.004), gold),
    f2: new THREE.Mesh(new THREE.BoxGeometry(0.006, 0.065, 0.004), gold),
  };
  for (const k of ["pillar", "upper", "fore", "ring", "handle", "f1", "f2"]) mScene.add(m[k]);
  masters[s] = m;
}
const mFloor = new THREE.Mesh(new THREE.PlaneGeometry(1.0, 0.7), new THREE.MeshPhongMaterial({ color: 0x2a3036 }));
mFloor.position.set(0, 0.1, -0.2); mScene.add(mFloor);
function colAxes(R) { return [0, 1, 2].map(j => new THREE.Vector3(R[0][j], R[1][j], R[2][j])); }
function updateMaster(s, d, active) {
  const m = masters[s], S = v3(d.S, 1), E = v3(d.E, 1), H = v3(d.H, 1);
  setCylinder(m.pillar, S.clone().add(new THREE.Vector3(0, 0, 0.16)), S);
  setCylinder(m.upper, S, E); setCylinder(m.fore, E, H);
  const [x, y, z] = colAxes(d.R);
  m.ring.position.copy(H);
  m.ring.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), y);      // ring around the handle axis
  const back = H.clone().addScaledVector(y, -0.08);
  setCylinder(m.handle, back, H);
  const g = d.grip / 2;
  [[m.f1, 1], [m.f2, -1]].forEach(([f, sgn]) => {
    const dir = y.clone().multiplyScalar(-Math.cos(g)).addScaledVector(x, sgn * Math.sin(g)).normalize();
    f.position.copy(H).addScaledVector(y, -0.015).addScaledVector(dir, 0.0325);
    f.quaternion.setFromUnitVectors(Y, dir);
  });
  m.col.emissive.setHex(active ? 0x222222 : 0x000000);
  m.col.opacity = 1;
}

// ------------------------------------------------------------- 2D hand pads
const pads = document.getElementById("pads"), pg = pads.getContext("2d");
function drawPads(st) {
  const W = pads.width, H = pads.height, [hx, hy, hz] = st.workspace;
  pg.clearRect(0, 0, W, H);
  SIDES.forEach((s, i) => {
    const w = 132, h = w * hz / hx, x0 = 8 + i * (w + 20), y0 = 22;
    const act = st.active === s;
    pg.strokeStyle = HEX[s]; pg.lineWidth = act ? 3 : 1;
    pg.fillStyle = st.clutch ? "rgba(231,178,74,.18)" : act ? "rgba(255,255,255,.06)" : "rgba(0,0,0,0)";
    pg.fillRect(x0, y0, w, h); pg.strokeRect(x0, y0, w, h);
    pg.fillStyle = HEX[s]; pg.font = "600 12px system-ui, sans-serif";
    pg.fillText(`${s} → ${ARM[s]}${st.mtm[s].latched ? "  ✊" : ""}`, x0, 15);
    const hd = st.mtm[s].hand;
    const px = x0 + w / 2 + hd[0] / hx * w / 2, py = y0 + h / 2 - hd[2] / hz * h / 2;
    pg.beginPath(); pg.arc(px, py, act ? 6 : 4, 0, 6.29); pg.fill();
    pg.fillStyle = "#e7b24a";                                   // depth bar
    pg.fillRect(x0 + w / 2, y0 + h + 4, hd[1] / hy * w / 2, 4);
  });
}

// --------------------------------------------------------------- the HUD
const hud = document.getElementById("hud"), task = document.getElementById("task");
const tag = (on, text, cls = "on") => `<span class="tag ${on ? cls : "off"}">${text}</span>`;
function updateHud(st) {
  const tp = st.teleop.MTMR || {}, tl = st.teleop.MTML || {};
  const follow = (tp.state === "FOLLOWING") && (tl.state === "FOLLOWING");
  hud.innerHTML =
    `${tag(follow, follow ? "GO" : (tp.state || "WAITING"), "go")} ${tag(st.clutch, "CLUTCH")} ${tag(st.camera_mode, "CAMERA")}<br>` +
    `active hand <b style="color:${HEX[st.active]}">${st.active} → ${ARM[st.active]}</b><br>` +
    `scale <b>${tp.scale ?? "?"}:1</b> · tremor <b>${st.tremor ? "on" : "off"}</b> · filter <b>${tp.tremor_filter ? "on" : "off"}</b> · delay <b>${(tp.latency_ms || 0).toFixed(0)} ms</b>`;
  const s = st.status || {};
  if (s.state === "running" && s.task === "reach")
    task.innerHTML = `Reach: target <b>${s.index + 1}/${s.n}</b> · ${s.time_s.toFixed(1)} s · path ${s.path_mm.toFixed(0)} mm · contacts ${s.contacts}`;
  else if (s.state === "running" && s.task === "handover")
    task.innerHTML = `Handover: placed <b>${s.beads.filter(b => b.placed).length}/${s.beads.length}</b> · ${s.time_s.toFixed(1)} s · handovers ${s.handovers} · drops ${s.drops} · contacts ${s.contacts}<br>` +
      `<span style="opacity:.75">pick with PSM2 (green, left hand), pass to PSM1 (orange, right hand), put down in the green ring</span>`;
  else if (s.state === "done")
    task.innerHTML = `<span class="tag go">TASK COMPLETE</span> ${s.task}: ${s.time_s.toFixed(1)} s · path ${s.path_mm.toFixed(0)} mm · ${s.contacts} contact(s)` +
      (s.task === "handover" ? ` · ${s.handovers} handovers · ${s.drops} drops` : "") + " · N or B to try again";
  else task.innerHTML = "N: reach task · B: handover task";
}

// ------------------------------------------------------------------ state
let state = null, ws = null, lastMsg = 0;
const offline = document.getElementById("offline");
function apply(st) {
  state = st;
  for (const a of ["PSM1", "PSM2"]) {
    const o = inst[a], d = st.arms[a];
    for (const k of ["shaft", "collar", "clevis", "port", "wrist"]) o[k].visible = !!d;
    if (d) updateInstrument(o, d);
  }
  if (st.ecm) {
    camera.position.copy(v3(st.ecm.p));
    camera.quaternion.copy(rotFromRows(st.ecm.R, true));   // three.js cameras look along -z, y up
    const rcm = v3(st.ecm.rcm), u = new THREE.Vector3().subVectors(camera.position, rcm).normalize();
    setCylinder(ecmShaft, rcm.clone().addScaledVector(u, -120), camera.position);
  }
  headlight.position.copy(camera.position);
  const tg = targetPool(st.targets.length);
  st.targets.forEach((p, i) => { tg[i].position.copy(v3(p)); tg[i].children[0].material.opacity = i === 0 ? 0.35 : 0.12;
    tg[i].children[1].visible = i === 0; });
  const beads = st.status && st.status.task === "handover" ? st.status.beads : [];
  const bm = beadPool(beads.length), rm = ringPool(beads.length);
  beads.forEach((b, i) => {
    let p = v3(b.p);
    if (b.held.length && b.held.every(a => st.arms[a])) {        // follow the jaws smoothly
      p = new THREE.Vector3();
      b.held.forEach(a => p.add(v3(st.arms[a].tip)));
      p.multiplyScalar(1 / b.held.length);
    }
    bm[i].position.copy(p);
    bm[i].material.color.setHex(b.placed ? 0x35d08a : b.held.length ? 0xffb347 : 0xf28c28);
    rm[i].position.copy(v3(b.goal)).add(new THREE.Vector3(0, 0, 0.3));
  });
  for (const m of vesselMeshes) m.material.emissive.setHex(st.contact ? 0x991111 : 0x000000);
  document.getElementById("flash").classList.toggle("hit", st.contact);
  SIDES.forEach(s => updateMaster(s, st.mtm[s], s === st.active));
  drawPads(st); updateHud(st);
}
function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => { ws.send(JSON.stringify({ t: "hello" })); };
  ws.onmessage = ev => {
    lastMsg = performance.now();
    const m = JSON.parse(ev.data);
    if (m.t === "state") apply(m);
    else if (m.t === "vessels" && m.lines.length) buildVessels(m.lines);
  };
  ws.onclose = () => setTimeout(connect, 1000);
}
connect();
setInterval(() => {                                           // the scene publishes vessels once a second
  if (ws && ws.readyState === 1 && !vesselMeshes.length) ws.send(JSON.stringify({ t: "hello" }));
  offline.style.display = performance.now() - lastMsg > 1500 ? "block" : "none";
}, 1000);
const send = m => { if (ws && ws.readyState === 1) ws.send(JSON.stringify(m)); };

// ------------------------------------------------------------------ input
const sit = document.getElementById("sit"), help = document.getElementById("help");
let dx = 0, dy = 0;
sit.addEventListener("click", () => renderer.domElement.requestPointerLock());
document.addEventListener("pointerlockchange", () => {
  const locked = document.pointerLockElement === renderer.domElement;
  sit.style.display = locked ? "none" : "grid";
  if (!locked) send({ t: "grip", down: false });
});
document.addEventListener("mousemove", e => {
  if (document.pointerLockElement !== renderer.domElement) return;
  dx += e.movementX; dy += e.movementY;
});
document.addEventListener("mousedown", e => {
  if (document.pointerLockElement === renderer.domElement && e.button === 0) send({ t: "grip", down: true });
});
document.addEventListener("mouseup", e => { if (e.button === 0) send({ t: "grip", down: false }); });
document.addEventListener("contextmenu", e => e.preventDefault());
window.addEventListener("wheel", e => {
  if (document.pointerLockElement !== renderer.domElement) return;
  e.preventDefault(); send({ t: "wheel", d: e.deltaY });
}, { passive: false });
const HELD = new Set(["a", "d", "w", "s", "q", "e"]), held = new Set();
let stereo = false, overview = true;
addEventListener("blur", () => held.clear());
document.addEventListener("keyup", e => held.delete(e.key.toLowerCase()));
const KEYS = new Set(["tab", "x", "c", " ", "1", "2", "3", "5", "t", "f", "+", "=", "-", "n", "b", "g", "h",
  "j", "l", "i", "k", "u", "o", "v", "arrowleft", "arrowright", "arrowup", "arrowdown", "pageup", "pagedown", "home"]);
document.addEventListener("keydown", e => {
  const k = e.key.toLowerCase();
  if (k === "?") { help.style.display = help.style.display === "none" ? "block" : "none"; return; }
  if (HELD.has(k)) { e.preventDefault(); held.add(k); return; }        // continuous motion, see frame()
  if (k === "p") { stereo = !stereo; resize(); return; }
  if (k === "m") { overview = !overview; return; }
  if (!KEYS.has(k)) return;
  e.preventDefault();
  const repeatable = k.startsWith("arrow") || k.startsWith("page") || "jlikuo".includes(k) && k.length === 1;
  if (e.repeat && !repeatable) return;
  send({ t: "key", k });
});

// ----------------------------------------------------------------- render
function resize() {
  renderer.setSize(innerWidth, innerHeight);
  camera.aspect = (stereo ? innerWidth / 2 : innerWidth) / innerHeight;
  camera.updateProjectionMatrix();
  for (const id of ["eyeL", "eyeR"]) document.getElementById(id).style.display = stereo ? "block" : "none";
}
addEventListener("resize", resize);
const STILL = location.search.includes("still");     // ?still: render only on request (automated tests)
const BASELINE = 5;                       // mm between the two cameras of the stereo endoscope
const eye = camera.clone();
let lastFrame = performance.now();
function vp(x, y, w, h) { renderer.setScissor(x, y, w, h); renderer.setViewport(x, y, w, h); }
function frame() {
  if (!STILL) requestAnimationFrame(frame);
  const now = performance.now(), dt = Math.min(0.1, (now - lastFrame) / 1000);
  lastFrame = now;
  if (dx || dy) { send({ t: "move", dx, dy }); dx = dy = 0; }
  if (held.size) {
    const axis = (p, n) => (held.has(p) ? 1 : 0) - (held.has(n) ? 1 : 0);
    send({ t: "nudge", right: axis("d", "a"), forward: axis("w", "s"), up: axis("q", "e"), dt });
  }
  const W = innerWidth, H = innerHeight;
  const B = help.style.display === "none" ? 12 : help.offsetHeight + 24;   // the panels sit above the key list
  pads.style.bottom = B + "px";
  document.getElementById("mtmLabel").style.bottom = (B + 202) + "px";
  document.getElementById("ovLabel").style.bottom = (B + 356) + "px";
  renderer.setScissorTest(true);
  renderer.shadowMap.needsUpdate = true;                   // shadows once per frame, not once per view
  for (const o of overviewOnly) o.visible = false;
  dome.visible = true; scene.fog.far = 380;
  if (stereo) {                                              // left and right cameras of the endoscope
    [-1, 1].forEach((side, i) => {
      eye.copy(camera);
      eye.position.add(new THREE.Vector3(side * BASELINE / 2, 0, 0).applyQuaternion(camera.quaternion));
      vp(i * W / 2, 0, W / 2, H);
      renderer.setClearColor(BG, 1); renderer.clear();
      renderer.render(scene, eye);
    });
  } else {
    vp(0, 0, W, H);
    renderer.setClearColor(BG, 1); renderer.clear();
    renderer.render(scene, camera);
  }
  if (state) {
    vp(12, B, 300, 190);                                   // the masters, bottom left
    renderer.setClearColor(0x15191d, 1); renderer.clear();
    renderer.render(mScene, mCam);
    if (overview) {                                          // the scene from outside, right
      for (const o of overviewOnly) o.visible = true;
      dome.visible = false; scene.fog.far = 1e5;
      updateFrustum();
      vp(W - 312, B + 154, 300, 190);
      renderer.setClearColor(0x15191d, 1); renderer.clear();
      renderer.render(scene, ovCam);
      for (const o of overviewOnly) o.visible = false;
      dome.visible = true; scene.fog.far = 380;
    }
  }
  renderer.setScissorTest(false);
  document.getElementById("ovLabel").style.display = state && overview ? "block" : "none";
}
frame();
window.renderOnce = () => { frame(); return renderer.domElement.toDataURL("image/png"); };
})();
