// Ported from unitree_ui/src/ui/scene/voxel-map.ts.
import * as THREE from './three.module.js';

export class VoxelMap {
  constructor(scene, onStatus) {
    this.scene = scene;
    this.onStatus = onStatus;
    this.mesh = null;
    this.generation = 0;
    this.busy = false;
    this.pending = null;
    const texture = new THREE.TextureLoader().load('/models/axisColor4.png');
    texture.magFilter = THREE.NearestFilter;
    texture.minFilter = THREE.NearestFilter;
    this.material = new THREE.MeshBasicMaterial({map: texture, side: THREE.DoubleSide});
    this.worker = new Worker(new URL('./voxel-worker.js', import.meta.url), {type: 'module'});
    this.worker.onmessage = ({data}) => {
      this.busy = false;
      if (data.generation === this.generation) {
        if (data.error) this.onStatus(data.error, true);
        else this.applyGeometry(data);
      }
      this.flush();
    };
    this.worker.onerror = (error) => {
      this.busy = false;
      this.onStatus(`体素解码失败：${error.message}`, true);
    };
  }

  processCompressed(payload) {
    // The map retains full resolution; only the separate debug cloud is sampled.
    this.pending = payload;
    this.flush();
  }

  flush() {
    if (this.busy || !this.pending) return;
    const payload = this.pending;
    this.pending = null;
    try {
      const bytes = Uint8Array.from(atob(payload.data), c => c.charCodeAt(0));
      this.busy = true;
      this.worker.postMessage({...payload, data: bytes, generation: this.generation}, [bytes.buffer]);
    } catch (error) {
      this.busy = false;
      this.onStatus(`体素数据无效：${error.message}`, true);
    }
  }

  applyGeometry({geometryData, resolution, origin}) {
    if (!Number.isFinite(resolution) || resolution <= 0 ||
        !Array.isArray(origin) || origin.length < 3 || !origin.every(Number.isFinite)) {
      this.onStatus('体素坐标元数据无效', true);
      return;
    }
    this.removeMesh();
    if (geometryData.face_count) {
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.BufferAttribute(geometryData.positions, 3));
      geometry.setAttribute('uv', new THREE.BufferAttribute(geometryData.uvs, 2, true));
      geometry.setIndex(new THREE.BufferAttribute(geometryData.indices, 1));
      this.mesh = new THREE.Mesh(geometry, this.material);
      this.mesh.scale.setScalar(resolution);
      this.mesh.position.fromArray(origin);
      this.mesh.frustumCulled = false;
      this.scene.add(this.mesh);
    }
    this.onStatus(`${geometryData.point_count.toLocaleString()} 体素 · ${geometryData.face_count.toLocaleString()} 面`);
  }

  removeMesh() {
    if (!this.mesh) return;
    this.scene.remove(this.mesh);
    this.mesh.geometry.dispose();
    this.mesh = null;
  }

  clear() {
    this.generation++;
    this.pending = null;
    this.removeMesh();
  }

  destroy() {
    this.clear();
    this.worker.terminate();
    this.material.map.dispose();
    this.material.dispose();
  }
}
