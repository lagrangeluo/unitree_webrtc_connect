// Ported from unitree_ui/src/ui/scene/robot-model.ts.
import * as THREE from './three.module.js';
import { GLTFLoader } from './examples/jsm/loaders/GLTFLoader.js';
const LEGS = ['FR', 'FL', 'RR', 'RL'];
const JOINTS = ['hip', 'thigh', 'calf'];

export class Go2Model {
  constructor(scene) {
    this.scene = scene;
    this.model = null;
    this.joints = new Map();
    // Keep telemetry even while the GLB is loading.
    this.position = new THREE.Vector3(0, 0, -0.3);
    this.quaternion = new THREE.Quaternion();
    this.motors = [];
    this.load();
  }

  async load() {
    try {
      const {scene: model} = await new GLTFLoader().loadAsync('/models/Go2.glb');
      this.model = model;
      for (const name of ['ExtendRail', ...LEGS.map(leg => `Rod${leg}`)]) {
        const part = model.getObjectByName(name);
        if (part) part.visible = false;
      }
      model.traverse(object => {
        if (!object.isMesh) return;
        object.castShadow = object.receiveShadow = true;
        for (const material of Array.isArray(object.material) ? object.material : [object.material]) {
          material.transparent = true;
          material.opacity = 0.85;
        }
      });
      for (const leg of LEGS) for (const joint of JOINTS) {
        const prefix = joint === 'hip' ? 'Hip' : joint === 'thigh' ? 'Thigh' : 'Calf';
        const bone = model.getObjectByName(`${prefix}Bone${leg}`);
        if (bone) this.joints.set(joint + leg, {
          bone, axis: new THREE.Vector3(joint === 'hip' ? 1 : 0, 0, joint === 'hip' ? 0 : 1),
          initial: bone.quaternion.clone(), offset: joint === 'thigh' ? Math.PI / 4 : joint === 'calf' ? -Math.PI / 2 : 0,
        });
      }
      const radar = model.getObjectByName('RadarBone');
      if (radar) this.joints.set('radar', {
        bone: radar, axis: new THREE.Vector3(0, -1, 0), initial: radar.quaternion.clone(), offset: 0,
      });
      model.position.copy(this.position);
      model.quaternion.copy(this.quaternion);
      this.update(this.motors);
      this.scene.add(model);
      window.dispatchEvent(new CustomEvent('go2-model-loaded'));
    } catch (error) {
      console.error('[go2_web] Go2.glb load failed', error);
      window.dispatchEvent(new CustomEvent('go2-model-error', {detail: String(error)}));
    }
  }

  setJoint(name, angle) {
    const joint = this.joints.get(name);
    if (!joint || !Number.isFinite(angle)) return;
    const rotation = new THREE.Quaternion().setFromAxisAngle(joint.axis, angle - joint.offset);
    joint.bone.quaternion.copy(rotation.premultiply(joint.initial));
  }

  update(motors) {
    this.motors = Array.isArray(motors) ? motors : [];
    for (let i = 0; i < Math.min(12, this.motors.length); i++) {
      this.setJoint(JOINTS[i % 3] + LEGS[Math.floor(i / 3)], this.motors[i]?.q);
    }
  }

  pose(position, quaternion) {
    if (Array.isArray(position) && position.length >= 3 && position.every(Number.isFinite)) {
      this.position.set(position[0], position[1], position[2] - 0.3);
    }
    if (Array.isArray(quaternion) && quaternion.length === 4 && quaternion.every(Number.isFinite) &&
        quaternion.some(v => v !== 0)) this.quaternion.fromArray(quaternion).normalize();
    if (!this.model) return;
    this.model.position.copy(this.position);
    this.model.quaternion.copy(this.quaternion);
  }

  animate(now) {
    this.setJoint('radar', now / 1000 * Math.PI * 2);
  }
}
