(function(){
const DATA = window.ARMS;
const host = document.getElementById('stage');
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
let renderer, scene, camera, controls, group, armIdx = 0, pose = 'walking_stance';
function w2t(p){ return new THREE.Vector3(p[0], p[2], -p[1]); }
function init(){
  renderer = new THREE.WebGLRenderer({antialias:true, alpha:true});
  renderer.setPixelRatio(Math.min(2, window.devicePixelRatio||1));
  host.appendChild(renderer.domElement);
  scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(38, 1, 0.01, 10);
  camera.position.set(0.55, 0.45, 0.95);
  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.target.set(0.38, -0.05, 0); controls.enableDamping = true;
  scene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 0.9));
  const d = new THREE.DirectionalLight(0xffffff, 0.7); d.position.set(1, 2, 1.5); scene.add(d);
  resize(); window.addEventListener('resize', resize);
  build(); animate();
}
function resize(){
  const w = host.clientWidth, h = host.clientHeight;
  renderer.setSize(w, h); camera.aspect = w / h; camera.updateProjectionMatrix();
}
function mat(c, o){ return new THREE.MeshStandardMaterial({color:new THREE.Color(c), roughness:0.55, metalness:0.05, transparent:o<1, opacity:o}); }
function build(){
  if (group) scene.remove(group);
  group = new THREE.Group(); scene.add(group);
  const arm = DATA[armIdx];
  const cTendon = css('--arm'), cServo = css('--servo'), cGhost = css('--muted'), cTarget = css('--target');
  // hub and ground
  const hub = new THREE.Mesh(new THREE.SphereGeometry(0.1, 32, 20), mat(cGhost, 0.25)); group.add(hub);
  const grid = new THREE.GridHelper(1.4, 28, new THREE.Color(cGhost), new THREE.Color(cGhost));
  grid.position.set(0.3, -0.2, 0); grid.material.transparent = true; grid.material.opacity = 0.25; group.add(grid);
  const pts = arm.poses[pose];
  for (let i = 0; i < pts.length - 1; i++){
    const a = w2t(pts[i]), b = w2t(pts[i+1]); const r = arm.radii[i];
    const len = a.distanceTo(b); if (len < 1e-5) continue;
    const m = mat((arm.node_act[i] === 'local' || arm.node_act[i] === 'graded') ? cServo : cTendon, 1);
    const cyl = new THREE.Mesh(new THREE.CylinderGeometry(r*0.92, r*0.92, len, 20), m);
    cyl.position.copy(a).add(b).multiplyScalar(0.5);
    cyl.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0), b.clone().sub(a).normalize());
    group.add(cyl);
    const s = new THREE.Mesh(new THREE.SphereGeometry(r*0.95, 16, 12), m); s.position.copy(b); group.add(s);
  }
  const tip = new THREE.Mesh(new THREE.SphereGeometry(0.02, 16, 12), mat(cServo, 1)); tip.position.copy(w2t(pts[pts.length-1])); group.add(tip);
  if (arm.targets[pose]){
    const g = new THREE.BufferGeometry().setFromPoints(arm.targets[pose].map(w2t));
    const l = new THREE.Line(g, new THREE.LineDashedMaterial({color:new THREE.Color(cTarget), dashSize:0.012, gapSize:0.008}));
    l.computeLineDistances(); group.add(l);
  }
  document.getElementById('armLabel').textContent = arm.label;
}
function animate(){ requestAnimationFrame(animate); controls.update(); renderer.render(scene, camera); }
document.querySelectorAll('[data-arm]').forEach(b => b.addEventListener('click', () => {
  armIdx = +b.dataset.arm; document.querySelectorAll('[data-arm]').forEach(x => x.setAttribute('aria-pressed', x === b)); build(); }));
document.querySelectorAll('[data-pose]').forEach(b => b.addEventListener('click', () => {
  pose = b.dataset.pose; document.querySelectorAll('[data-pose]').forEach(x => x.setAttribute('aria-pressed', x === b)); build(); }));
window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', build);
new MutationObserver(build).observe(document.documentElement, {attributes:true, attributeFilter:['data-theme']});
try { init(); } catch(e){ host.innerHTML = '<p class="fallback">3D preview needs WebGL. The table below has the same arms.</p>'; }
})();
