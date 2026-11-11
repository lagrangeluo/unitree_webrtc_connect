// Follow camera, lighting and map rendering ported from unitree_ui's Scene3D.
import * as THREE from './three.module.js';
import { OrbitControls } from './examples/jsm/controls/OrbitControls.js';
import { HDRLoader } from './examples/jsm/loaders/HDRLoader.js';
import { Go2Model } from './robot-model.js';
import { VoxelMap } from './voxel-map.js';

export class Go2Scene {
  constructor(canvas) {
    this.canvas = canvas;
    this.renderer = new THREE.WebGLRenderer({canvas, antialias: true, alpha: false});
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x282828);
    this.followFog = new THREE.Fog(0x282828, 0.015, 20);
    this.scene.fog = this.followFog;
    this.camera = new THREE.PerspectiveCamera(70, 1, 0.1, 500);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(-1.2, 0, 0.7);
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.target.set(4, 0, -0.3);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.1;
    this.controls.minDistance = 0.5;
    this.controls.maxDistance = 20;
    this.controls.enabled = false;
    this.controls.update();
    this.viewType = 'follow';
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.6));
    const light = new THREE.DirectionalLight(0xffffff, 1.2);
    light.position.set(5, -5, 8);
    light.castShadow = true;
    light.shadow.mapSize.set(2048, 2048);
    Object.assign(light.shadow.camera, {near: 0.5, far: 30, left: -5, right: 5, top: 5, bottom: -5});
    this.scene.add(light);
    this.scene.add(new THREE.HemisphereLight(0x8888ff, 0x444422, 0.4));
    const grid = new THREE.GridHelper(40, 40, 0x888888, 0x888888);
    grid.rotateX(Math.PI / 2);
    this.scene.add(grid);
    new HDRLoader().load('/models/venice_sunset_1k.hdr', texture => {
      texture.mapping = THREE.EquirectangularReflectionMapping;
      this.scene.environment = texture;
    }, undefined, error => console.error('[go2_web] Environment load failed', error));
    this.model = new Go2Model(this.scene);
    this.voxelMap = new VoxelMap(this.scene, (message, error = false) => {
      const status = document.getElementById('fp-status');
      status.textContent = message;
      status.classList.toggle('fresh', !error);
      document.getElementById('fp-empty').hidden = !error;
    });
    canvas.addEventListener('dblclick', () => this.toggleView());
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(canvas.parentElement);
    this.resize();
    this.loop();
  }

  resize() {
    const {width, height} = this.canvas.parentElement.getBoundingClientRect();
    this.renderer.setSize(Math.max(1, width), Math.max(1, height), false);
    this.camera.aspect = Math.max(1, width) / Math.max(1, height);
    this.camera.updateProjectionMatrix();
  }

  update(motors, position, quaternion) {
    this.model.update(motors);
    this.model.pose(position, quaternion);
  }

  offset(x, y, z) {
    const yaw = new THREE.Euler().setFromQuaternion(this.model.quaternion, 'ZYX').z;
    return new THREE.Vector3(x * Math.cos(yaw) - y * Math.sin(yaw),
      x * Math.sin(yaw) + y * Math.cos(yaw), z);
  }

  toggleView() {
    this.setView(this.viewType === 'follow' ? 'overview' : 'follow');
  }

  setView(view) {
    this.viewType = view;
    this.controls.enabled = view === 'overview';
    this.scene.fog = view === 'follow' ? this.followFog : null;
    if (view === 'overview') {
      this.camera.position.copy(this.model.position).add(this.offset(-3, 0, 3));
      this.controls.target.copy(this.model.position);
    }
    // Clear orbit momentum when returning to the locked follow camera.
    this.controls.enableDamping = view === 'overview';
    this.updateFollowCamera();
    this.controls.update();
    const button = document.getElementById('toggle-3d-view');
    if (button) button.textContent = view === 'follow' ? '切换总览' : '返回跟随';
    document.getElementById('fp-help').textContent = view === 'follow'
      ? '跟随视角 · 双击切换总览 · X 前 / Y 左 / Z 上'
      : '总览视角 · 左键旋转 · 右键平移 · 滚轮缩放 · 双击返回跟随';
  }

  updateFollowCamera() {
    if (this.viewType !== 'follow') return;
    this.camera.position.copy(this.model.position).add(this.offset(-1.2, 0, 1));
    this.controls.target.copy(this.model.position).add(this.offset(4, 0, 0));
  }

  clear() {
    this.voxelMap.clear();
    const status = document.getElementById('fp-status');
    status.textContent = '等待体素地图';
    status.classList.remove('fresh');
    document.getElementById('fp-empty').hidden = false;
  }

  loop() {
    this.animationId = requestAnimationFrame(() => this.loop());
    this.model.animate(performance.now());
    this.updateFollowCamera();
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  destroy() {
    cancelAnimationFrame(this.animationId);
    this.resizeObserver.disconnect();
    this.controls.dispose();
    this.voxelMap.destroy();
    this.renderer.dispose();
  }
}
