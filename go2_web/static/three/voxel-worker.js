// Ported from unitree_ui/src/workers/voxel-worker.ts; see UNITREE_UI_LICENSE.txt.
const workerLog = console;
class VoxelDecompressor {
  mod;
  _input;
  _decompressBuffer;
  _positions;
  _uvs;
  _indices;
  _decompressedSize;
  _faceCount;
  _pointCount;
  _decompressBufferSize;
  constructor(mod, decompressBufferSize) {
    this.mod = mod;
    this._input = mod._malloc(61440);
    this._decompressBuffer = mod._malloc(8e4);
    this._positions = mod._malloc(288e4);
    this._uvs = mod._malloc(192e4);
    this._indices = mod._malloc(576e4);
    this._decompressedSize = mod._malloc(4);
    this._faceCount = mod._malloc(4);
    this._pointCount = mod._malloc(4);
    this._decompressBufferSize = decompressBufferSize;
  }
  generate(data, zNormalized) {
    this.mod.HEAPU8 = new Uint8Array(this.mod.memory.buffer);
    if (data.length > 61440) throw new Error("\u538B\u7F29\u4F53\u7D20\u8D85\u51FA\u89E3\u7801\u7F13\u51B2\u533A");
    this.mod.HEAPU8.set(data, this._input);
    this.mod._generate(
      this._input,
      data.length,
      this._decompressBufferSize,
      this._decompressBuffer,
      this._decompressedSize,
      this._positions,
      this._uvs,
      this._indices,
      this._faceCount,
      this._pointCount,
      zNormalized
    );
    this.mod.HEAPU8 = new Uint8Array(this.mod.memory.buffer);
    const pointCount = this.mod.getValue(this._pointCount, "i32");
    const faceCount = this.mod.getValue(this._faceCount, "i32");
    const positions = new Uint8Array(
      this.mod.HEAPU8.subarray(this._positions, this._positions + faceCount * 12).slice()
    );
    const uvs = new Uint8Array(
      this.mod.HEAPU8.subarray(this._uvs, this._uvs + faceCount * 8).slice()
    );
    const indices = new Uint32Array(
      this.mod.HEAPU8.subarray(this._indices, this._indices + faceCount * 24).slice().buffer
    );
    return { point_count: pointCount, face_count: faceCount, positions, uvs, indices };
  }
}
let decompressor = null;
async function initWasm() {
  try {
    const response = await fetch("/static/libvoxel.wasm");
    const wasmBytes = await response.arrayBuffer();
    let wasmHeapU8 = null;
    const importObj = {
      a: {
        // emscripten_memcpy_js (import 'b')
        b: (dest, src, num) => {
          if (wasmHeapU8) {
            wasmHeapU8.copyWithin(dest, src, src + num);
          }
        },
        // emscripten_resize_heap (import 'a')
        a: () => {
          return 0;
        }
      }
    };
    const result = await WebAssembly.instantiate(wasmBytes, importObj);
    const exports = result.instance.exports;
    const wasmMemory = exports.c;
    wasmHeapU8 = new Uint8Array(wasmMemory.buffer);
    const ctors = exports.d;
    if (ctors) ctors();
    const mod = {
      _generate: exports.e,
      _malloc: exports.f,
      _free: exports.g,
      HEAPU8: wasmHeapU8,
      memory: wasmMemory,
      getValue: (ptr, type) => {
        const buf = wasmMemory.buffer;
        switch (type) {
          case "i32":
            return new Int32Array(buf)[ptr >> 2];
          case "i16":
            return new Int16Array(buf)[ptr >> 1];
          case "i8":
            return new Int8Array(buf)[ptr];
          case "float":
            return new Float32Array(buf)[ptr >> 2];
          case "double":
            return new Float64Array(buf)[ptr >> 3];
          default:
            return new Int32Array(buf)[ptr >> 2];
        }
      }
    };
    decompressor = new VoxelDecompressor(mod, 8e4);
    workerLog.info("[go2:voxel-worker] WASM loaded, decompressor ready");
  } catch (err) {
    workerLog.error("[go2:voxel-worker] Failed to load WASM:", err);
  }
}
const ready = initWasm();
const ctx = self;
ctx.addEventListener("message", async (e) => {
  await ready;
  const msg = e.data;
  if (!decompressor) {
    ctx.postMessage({ generation: msg.generation, error: "\u4F53\u7D20\u89E3\u7801\u5668\u52A0\u8F7D\u5931\u8D25" });
    return;
  }
  const rawData = msg.data;
  const resolution = Number(msg.resolution) || 0.1;
  const origin = msg.origin;
  if (!rawData || !Number.isFinite(msg.resolution) || msg.resolution <= 0 ||
      !Array.isArray(origin) || origin.length < 3 || !origin.every(Number.isFinite)) {
    ctx.postMessage({generation: msg.generation, error: '体素坐标元数据无效'});
    return;
  }
  const data = new Uint8Array(rawData instanceof ArrayBuffer ? rawData : rawData.buffer || rawData);
  const zNormalized = Math.floor(origin[2] / resolution);
  try {
    const geometryData = decompressor.generate(data, zNormalized);
    ctx.postMessage({ geometryData, resolution, origin, generation: msg.generation }, [geometryData.positions.buffer, geometryData.uvs.buffer, geometryData.indices.buffer]);
  } catch (error) {
    ctx.postMessage({ generation: msg.generation, error: String(error) });
  }
});
