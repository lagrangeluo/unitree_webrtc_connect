"""Minimal online 2D mapping, A* planning and pure-pursuit following.

Scope (deliberately small):
- Trusts the robot's own odometry (rt/utlidar/robot_pose). The voxel map
  (rt/utlidar/voxel_map_compressed) is already in the same ``odom`` frame, so
  frames are pasted into one global grid without scan matching.
- The map only lives for the current power-on session.
- Planning only goes through cells that have been observed free.

Only numpy is required. Map integration and planning are CPU work and are
run by the caller in a worker thread; ``OccupancyMap.lock`` guards the grid.
"""
import base64
import collections
import heapq
import math
import struct
import threading
import time
import zlib

import numpy as np

RES = 0.05                 # grid cell size (m)
SIZE_M = 60.0              # map covers [-30, 30] m around the power-on origin
MAX_RANGE = 8.0            # ignore voxels farther than this from the robot
ROBOT_HEIGHT = 0.40        # Go2 standing height (m)
# Heights are measured from the *bottom* of the floor layer. On the real Go2
# voxel map the floor is not a thin layer: one 5 cm column typically stacks
# 3-5 voxels (~0.25 m) because of glossy-floor/lidar noise accumulated by the
# robot's local map. Obstacles therefore start above that thickness.
# These match the Unitree voxel colour bands seen in the 3D view on this
# floor (bottom ~ -0.15 m in odom z): green = floor noise, yellow/orange =
# obstacles, purple/red (above the robot) = ignored.
FLOOR_THICKNESS = 0.30                               # green / yellow boundary
OBST_BAND = (FLOOR_THICKNESS, 0.70)                  # orange / purple boundary
GROUND_BAND = (-0.10, FLOOR_THICKNESS - 0.05)
FLOOR_TILE = 1.0           # local floor height is refined per 1 m x 1 m tile (handles slight tilt)
RAY_BINS = 720             # 0.5 deg angular bins for free-space rays

L_OCC, L_FREE_GROUND, L_FREE_RAY = 0.9, 0.45, 0.3
L_MIN, L_MAX = -2.0, 3.5   # clamping = settled cells stop changing
OCC_T, FREE_T = 0.7, -0.4

FOOTPRINT_R = 0.30         # cells under the robot are always free
LETHAL_R = 0.30            # planner keeps the robot centre this far from obstacles
SOFT_R = 0.70              # extra cost below this distance keeps paths centred
SMOOTH_R = 0.45            # clearance required for smoothing shortcuts

NAV_VMAX = 0.40            # m/s
NAV_VMIN = 0.15            # below ~0.15 m/s Go2 mostly steps in place
NAV_WMAX = 0.8             # rad/s
LOOKAHEAD = 0.5            # m
GOAL_TOL = 0.12            # m
YAW_TOL = 0.10             # rad
BODY_HALF_LEN, BODY_HALF_WIDTH = 0.33, 0.15
FRONT_STOP = 0.35          # min stop distance ahead of the nose; grows with speed
ROTATE_CLEARANCE = 0.40    # in-place turns need this radius free
BLOCKED_TIMEOUT = 12.0
PROGRESS_TIMEOUT = 25.0
REPLAN_INTERVAL = 1.5

# Scan-to-map matching corrects odometry drift (Go2 Air has no lidar odometry:
# sportmodestate = leg kinematics + IMU, whose yaw slowly drifts).
MATCH_ENABLED = True
MATCH_YAW = math.radians(2.0)   # search window per integration step
MATCH_XY = 0.15                 # m
MATCH_SIGMA = 0.06              # likelihood-field width (m)
MATCH_MIN_MAP = 250             # occupied cells needed in the window
MATCH_MIN_POINTS = 60           # obstacle cells needed in the scan
MATCH_MIN_SCORE = 0.30          # best score must reach this to be trusted
MATCH_MAX_POINTS = 700
MATCH_PRIOR = 0.01              # weight of the motion prior (score units)

UNKNOWN, FREE, OCCUPIED, INFLATED = 0, 1, 2, 3
PALETTE = [(0, 0, 0, 0), (36, 58, 70, 235), (236, 242, 246, 255), (92, 66, 128, 235)]


class PlanError(ValueError):
    """Planning failed for a reason that is shown to the user."""


def dilate(mask, diagonal):
    out = mask.copy()
    out[1:, :] |= mask[:-1, :]
    out[:-1, :] |= mask[1:, :]
    out[:, 1:] |= mask[:, :-1]
    out[:, :-1] |= mask[:, 1:]
    if diagonal:
        out[1:, 1:] |= mask[:-1, :-1]
        out[:-1, :-1] |= mask[1:, 1:]
        out[1:, :-1] |= mask[:-1, 1:]
        out[:-1, 1:] |= mask[1:, :-1]
    return out


def distance_steps(occupied, max_steps):
    """Approximate distance (in cells) to the nearest occupied cell.

    Alternating 4- and 8-neighbour dilation gives an octagonal metric that is
    within a few percent of Euclidean, using only numpy. Cells farther than
    ``max_steps`` get ``max_steps + 1``.
    """
    dist = np.full(occupied.shape, max_steps + 1, np.int16)
    dist[occupied] = 0
    reached = occupied.copy()
    for step in range(1, max_steps + 1):
        grown = dilate(reached, diagonal=step % 2 == 0)
        dist[grown & ~reached] = step
        reached = grown
    return dist


def encode_png(image, palette):
    """Encode a uint8 index image as a palette PNG (no extra dependencies)."""
    height, width = image.shape
    raw = np.zeros((height, width + 1), np.uint8)
    raw[:, 1:] = image

    def chunk(kind, data):
        return (struct.pack(">I", len(data)) + kind + data
                + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF))

    header = struct.pack(">IIBBBBB", width, height, 8, 3, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", header)
            + chunk(b"PLTE", bytes(c for rgba in palette for c in rgba[:3]))
            + chunk(b"tRNS", bytes(rgba[3] for rgba in palette))
            + chunk(b"IDAT", zlib.compress(raw.tobytes(), 6)) + chunk(b"IEND", b""))


def estimate_floor(z):
    """Floor height = the densest 5 cm height layer (histogram mode).

    A low percentile is pulled down by mirror points under glossy floors; the
    floor layer itself is by far the most populated horizontal layer near the
    robot, while walls and reflections spread over many layers.
    """
    if z.size < 50:
        return None
    lo = float(z.min())
    idx = ((z - lo) / 0.05).astype(np.int64)
    counts = np.bincount(idx)
    k = int(np.argmax(counts))
    centre = lo + (k + 0.5) * 0.05
    layer = z[np.abs(z - centre) < 0.075]
    return float(np.median(layer))


def floor_bottom(points, cells, n, res, floor):
    """Per-point floor reference = bottom of the floor layer.

    Each 5 cm column's lowest voxel is a floor sample (if it is near the
    global floor layer); a 1 m tile uses the 20th percentile of its columns,
    which follows slight tilt and ignores sparse points below the floor.
    Returns (per-point reference, global bottom).
    """
    cols, inv = np.unique(cells, return_inverse=True)
    colmin = np.full(cols.size, np.inf, np.float32)
    np.minimum.at(colmin, inv, points[:, 2])
    cand = np.abs(colmin - floor) < 0.3
    if cand.sum() < 10:
        return np.full(len(points), floor, np.float32), floor
    bottom = float(np.percentile(colmin[cand], 20))
    cy, cx = np.divmod(cols, n)
    per_tile = max(1, int(round(FLOOR_TILE / res)))
    tile = (cy // per_tile) * (n // per_tile + 1) + cx // per_tile
    ct, cm = tile[cand], colmin[cand]
    order = np.lexsort((cm, ct))
    keys, starts, counts = np.unique(ct[order], return_index=True, return_counts=True)
    tile_bottom = cm[order][starts + (counts * 0.2).astype(np.int64)]
    good = counts >= 5
    keys, tile_bottom = keys[good], tile_bottom[good]
    ref_col = np.full(cols.size, bottom, np.float32)
    if keys.size:
        pos = np.clip(np.searchsorted(keys, tile), 0, keys.size - 1)
        hit = keys[pos] == tile
        ref_col[hit] = tile_bottom[pos[hit]]
    return ref_col[inv], bottom


def wrap_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class OccupancyMap:
    def __init__(self, size_m=SIZE_M, res=RES):
        self.res = res
        self.n = int(round(size_m / res))
        self.x0 = self.y0 = -size_m / 2
        self.lock = threading.Lock()
        r = int(math.ceil(FOOTPRINT_R / res))
        oy, ox = np.mgrid[-r:r + 1, -r:r + 1]
        keep = (ox * ox + oy * oy) * res * res <= FOOTPRINT_R ** 2
        self._footprint = (oy[keep], ox[keep])
        self.version = 0
        self.reset()

    def reset(self):
        with self.lock:
            self.L = np.zeros((self.n, self.n), np.float32)
            self.bbox = None            # [ix0, iy0, ix1, iy1] of touched cells
            self.floor = None
            self.known = 0
            self.updated_at = 0.0
            self.rendered_at = 0.0
            self.dirty = False
            self.last_pose = None
            self.png = None
            self.png_meta = None
            self.scan = np.zeros((0, 2), np.float32)
            self.scan_list = []
            self.scan_at = 0.0
            self.status = "等待点云与里程计"
            self.pose_source = None
            self.last_frame = None
            self.floor_layer = None
            # map = R(dyaw) * odom + (dx, dy); grows as odometry drift is corrected
            self.corr = (0.0, 0.0, 0.0)
            self.map_pose = None
            self.match_info = {}
            # Ring buffer of integrated frames (time, pose, 2D obstacle scan) for offline drift analysis.
            self.session = collections.deque(maxlen=1500)
            self.frame_stats = {}
            self.version += 1

    # ── geometry helpers ──
    def cells(self, xy, keep_mask=False):
        """Flat indices of the cells containing ``xy`` (points outside dropped).

        A tiny bias keeps points that sit exactly on a cell border (voxel
        lattices are on the same 5 cm pitch) in one cell instead of jittering
        between two, which otherwise leaves empty stripes in the map.
        """
        ix = np.floor((xy[:, 0] - self.x0) / self.res + 1e-3).astype(np.int64)
        iy = np.floor((xy[:, 1] - self.y0) / self.res + 1e-3).astype(np.int64)
        ok = (ix >= 0) & (ix < self.n) & (iy >= 0) & (iy < self.n)
        flat = iy[ok] * self.n + ix[ok]
        return (flat, ok) if keep_mask else flat

    def centers(self, flat):
        iy, ix = np.divmod(flat, self.n)
        return np.column_stack(((ix + 0.5) * self.res + self.x0,
                                (iy + 0.5) * self.res + self.y0)).astype(np.float32)

    # ── odom → map correction ──
    def to_map(self, pose, corr=None):
        """(x, y, z, yaw) in odom → same in the corrected map frame."""
        dx, dy, dyaw = self.corr if corr is None else corr
        c, s = math.cos(dyaw), math.sin(dyaw)
        x, y, z, yaw = pose
        return (c * x - s * y + dx, s * x + c * y + dy, z, wrap_angle(yaw + dyaw))

    @staticmethod
    def apply(xy, corr):
        dx, dy, dyaw = corr
        c, s = math.cos(dyaw), math.sin(dyaw)
        out = np.empty((len(xy), 2), np.float32)
        out[:, 0] = c * xy[:, 0] - s * xy[:, 1] + dx
        out[:, 1] = s * xy[:, 0] + c * xy[:, 1] + dy
        return out

    def _align_lattice(self, xy):
        """Put Go2's voxel lattice on cell centres (a one-off shift of <= res/2).

        The robot's voxels lie on a regular 5 cm lattice. If that lattice hits
        cell borders, every point is ambiguous between two cells and the
        matcher prefers a half-cell shift; aligning once removes both effects.
        """
        if len(xy) < 50:
            return
        shift = []
        for axis, origin in ((0, self.x0), (1, self.y0)):
            frac = ((xy[:, axis] - origin) / self.res) % 1.0
            ang = 2 * math.pi * frac
            c, s = float(np.cos(ang).mean()), float(np.sin(ang).mean())
            if math.hypot(c, s) < 0.9:          # not a lattice (noisy data): leave it
                shift.append(0.0)
                continue
            phase = (math.atan2(s, c) / (2 * math.pi)) % 1.0
            v = (0.5 - phase) % 1.0
            if abs(v - 0.5) < 0.05:
                v = 0.5        # points on cell borders: move them into the cell cells() already assigns
            elif v > 0.5:
                v -= 1.0
            shift.append(v * self.res)
        self.corr = (shift[0], shift[1], 0.0)

    def _match(self, obst_xy, odom_pose, odom_motion):
        """Correlative scan-to-map matching around the predicted correction.

        Searches a small rotation/translation window about the robot and keeps
        the best pose on a likelihood field of the current map's obstacles.
        A light prior towards the prediction keeps featureless directions
        (e.g. along a straight corridor) from wandering.
        """
        pred = self.corr
        info = {"used": False}
        self.match_info = info
        if len(obst_xy) < MATCH_MIN_POINTS:
            info["reason"] = "扫描障碍太少"
            return
        mx, my, _, _ = self.to_map(odom_pose, pred)
        r = int((MAX_RANGE + MATCH_XY + 0.3) / self.res)
        cx, cy = int((mx - self.x0) / self.res), int((my - self.y0) / self.res)
        ix0, iy0 = max(0, cx - r), max(0, cy - r)
        ix1, iy1 = min(self.n, cx + r + 1), min(self.n, cy + r + 1)
        window = self.L[iy0:iy1, ix0:ix1]
        occ = window > OCC_T
        # Only score points that land on already-observed cells: the leading
        # edge of a scan is new territory and must not pull the pose back.
        known = np.abs(window) > 0.3
        if int(occ.sum()) < MATCH_MIN_MAP:
            info["reason"] = "地图障碍太少"
            return
        k = int(math.ceil(3 * MATCH_SIGMA / self.res))
        d = distance_steps(occ, k).astype(np.float32) * self.res
        like = np.exp(-d * d / (2 * MATCH_SIGMA ** 2))
        like[d > k * self.res] = 0.0
        knownf = (known | occ).astype(np.float32)
        like = (like * knownf).astype(np.float32)
        h, w = like.shape
        # Short-term odometry is good: allow a small correction per step that
        # grows with how far / how much the robot moved since the last one.
        dist_moved, yaw_moved = odom_motion
        sig_t = 0.03 + 0.05 * dist_moved
        sig_y = math.radians(0.4) + 0.03 * yaw_moved
        # One raw point per cell. Raw (not rounded) coordinates keep sub-cell
        # misalignment visible; Go2 voxels sit on cell centres, so a static
        # robot scores best at zero shift (rounding to cell corners did not).
        moved = self.apply(obst_xy, pred)
        flat, inside = self.cells(moved, keep_mask=True)
        _, first = np.unique(flat, return_index=True)
        pts = moved[inside][first]
        if len(pts) > MATCH_MAX_POINTS:
            pts = pts[np.linspace(0, len(pts) - 1, MATCH_MAX_POINTS).astype(np.int64)]
        rel = pts - (mx, my)
        gx0, gy0 = self.x0 + ix0 * self.res, self.y0 + iy0 * self.res

        def search(yaws, offs, centre):
            best = (-1.0, 0.0, 0.0, 0.0, 0.0)
            tx, ty = np.meshgrid(offs + centre[0], offs + centre[1])
            tx, ty = tx.ravel(), ty.ravel()
            prior_t = (tx * tx + ty * ty) / (sig_t ** 2)
            n_pts = len(rel)
            for th in yaws + centre[2]:
                c, s = math.cos(th), math.sin(th)
                px = c * rel[:, 0] - s * rel[:, 1] + (mx - gx0)
                py = s * rel[:, 0] + c * rel[:, 1] + (my - gy0)
                # Bilinear lookup: sub-cell misalignment still changes the score,
                # so small drift is corrected continuously instead of in jumps.
                fx = (px[None, :] + tx[:, None]) / self.res - 0.5
                fy = (py[None, :] + ty[:, None]) / self.res - 0.5
                x0 = np.floor(fx).astype(np.int64); y0 = np.floor(fy).astype(np.int64)
                ax = (fx - x0).astype(np.float32); ay = (fy - y0).astype(np.float32)
                num = np.zeros(fx.shape, np.float32); den = np.zeros(fx.shape, np.float32)
                for oy, wy in ((0, 1 - ay), (1, ay)):
                    for ox, wx in ((0, 1 - ax), (1, ax)):
                        xx, yy = x0 + ox, y0 + oy
                        ok = (xx >= 0) & (xx < w) & (yy >= 0) & (yy < h)
                        xx, yy = np.clip(xx, 0, w - 1), np.clip(yy, 0, h - 1)
                        wgt = wx * wy * ok
                        num += wgt * like[yy, xx]
                        den += wgt * knownf[yy, xx]
                raw = num.sum(axis=1) / np.maximum(den.sum(axis=1), 0.7 * n_pts)
                adj = raw - MATCH_PRIOR * (prior_t + (th / sig_y) ** 2)
                i = int(np.argmax(adj))
                if adj[i] > best[0]:
                    best = (float(adj[i]), float(raw[i]), float(tx[i]), float(ty[i]), float(th))
            return best

        coarse = search(np.radians(np.arange(-2.0, 2.01, 0.5)), np.arange(-MATCH_XY, MATCH_XY + 1e-6, 0.05),
                        (0.0, 0.0, 0.0))
        fine = search(np.radians(np.arange(-0.375, 0.376, 0.125)), np.arange(-0.0375, 0.0376, 0.0125),
                      coarse[2:])
        base = search(np.zeros(1), np.zeros(1), (0.0, 0.0, 0.0))
        adj, raw, tx, ty, th = fine
        info.update({"score": round(raw, 3), "base": round(base[1], 3),
                     "delta": [round(tx, 3), round(ty, 3), round(math.degrees(th), 2)]})
        if raw < MATCH_MIN_SCORE:
            info["reason"] = "匹配度太低"
            return
        if adj <= base[0]:
            info.update({"used": True, "delta": [0.0, 0.0, 0.0]})
            return
        # new map point = R(th) (p_pred - c) + c + t, with p_pred = R0 p + t0
        c, s = math.cos(th), math.sin(th)
        dx0, dy0, dyaw0 = pred
        ndx = c * (dx0 - mx) - s * (dy0 - my) + mx + tx
        ndy = s * (dx0 - mx) + c * (dy0 - my) + my + ty
        self.corr = (ndx, ndy, wrap_angle(dyaw0 + th))
        info["used"] = True

    @staticmethod
    def classify(values):
        out = np.zeros(values.shape, np.uint8)
        out[values < FREE_T] = FREE
        out[values > OCC_T] = OCCUPIED
        return out

    def should_integrate(self, pose, now):
        if self.last_pose is None:
            return True
        dx = math.hypot(pose[0] - self.last_pose[0], pose[1] - self.last_pose[1])
        dyaw = abs(wrap_angle(pose[3] - self.last_pose[3]))
        elapsed = now - self.updated_at
        # A static robot looking at a static scene changes nothing; refresh
        # slowly anyway so moved obstacles are still noticed.
        return (elapsed >= 0.25 and (dx > 0.03 or dyaw > 0.035)) or elapsed >= 0.8

    # ── per-frame processing ──
    def process(self, points, pose, now=None):
        """Extract the 2D scan every frame; integrate into the map when useful.

        ``points`` is an (N, 3) array in the odom frame, ``pose`` is
        (x, y, z, yaw). Returns True when the map classification changed.
        """
        now = time.monotonic() if now is None else now
        points = np.asarray(points, np.float32).reshape(-1, 3)
        x, y, z, yaw = pose
        with self.lock:
            self.last_frame = (points, (x, y, z, yaw))
            d2 = (points[:, 0] - x) ** 2 + (points[:, 1] - y) ** 2
            points = points[d2 < MAX_RANGE ** 2]
            d2 = d2[d2 < MAX_RANGE ** 2]
            point_cells, inside = self.cells(points[:, :2], keep_mask=True)
            points = points[inside]
            near = points[d2[inside] < 3.0 ** 2, 2]
            layer = estimate_floor(near)
            if layer is None:
                if self.floor is None:
                    self.status = "机身 3 m 内只有 {} 个体素，无法估计地面（本帧共 {} 个）".format(
                        near.size, len(points))
                    return False
                layer = self.floor_layer
            self.floor_layer = layer if self.floor is None else 0.7 * self.floor_layer + 0.3 * layer
            ref, bottom = floor_bottom(points, point_cells, self.n, self.res, self.floor_layer)
            self.floor = bottom if self.floor is None else 0.7 * self.floor + 0.3 * bottom
            self.status = "建图中"
            rel = points[:, 2] - ref
            obstacles = points[(rel >= OBST_BAND[0]) & (rel <= OBST_BAND[1])][:, :2]
            ground = points[(rel > GROUND_BAND[0]) & (rel < GROUND_BAND[1])][:, :2]
            integrate = self.should_integrate(pose, now)
            if self.bbox is None and self.last_pose is None:
                self._align_lattice(points[:, :2])
            if integrate and MATCH_ENABLED and self.bbox is not None:
                lp = self.last_pose
                motion = ((math.hypot(x - lp[0], y - lp[1]), abs(wrap_angle(yaw - lp[3])))
                          if lp is not None else (1.0, 1.0))
                self._match(obstacles, pose, motion)
            # Everything below works in the corrected map frame.
            obstacles = self.apply(obstacles, self.corr)
            ground = self.apply(ground, self.corr)
            x, y, _, myaw = self.to_map(pose)
            self.map_pose = (x, y, myaw)
            robot_cell = self.cells(np.array([[x, y]], np.float32))
            fp = np.zeros(0, np.int64)
            if robot_cell.size:
                ry, rx = divmod(int(robot_cell[0]), self.n)
                fy, fx = ry + self._footprint[0], rx + self._footprint[1]
                ok = (fx >= 0) & (fx < self.n) & (fy >= 0) & (fy < self.n)
                fp = fy[ok] * self.n + fx[ok]
            occ = np.unique(self.cells(obstacles))
            occ = occ[~np.isin(occ, fp)]          # never mark the robot's own body
            self.scan = self.centers(occ)
            self.frame_stats = {"points": int(len(points)), "ground": int(len(ground)),
                                "obstacle": int(len(obstacles))}
            step = max(1, math.ceil(len(self.scan) / 6000))
            self.scan_list = np.round(self.scan[::step], 2).tolist()
            self.scan_at = now
            if not integrate:
                return False
            return self._integrate(occ, ground, fp, (x, y, myaw), now, pose)

    def _rays(self, x, y):
        """Cells between the robot and the nearest obstacle in each direction."""
        if not len(self.scan):
            return np.zeros(0, np.int64)
        dx, dy = self.scan[:, 0] - x, self.scan[:, 1] - y
        r = np.hypot(dx, dy)
        keep = r > 0.2
        if not keep.any():
            return np.zeros(0, np.int64)
        r, theta = r[keep], np.arctan2(dy[keep], dx[keep])
        bins = ((theta + math.pi) / (2 * math.pi) * RAY_BINS).astype(np.int64) % RAY_BINS
        order = np.lexsort((r, bins))
        first = np.r_[True, bins[order][1:] != bins[order][:-1]]
        nearest = order[first]
        length = r[nearest] - 2 * self.res      # stop just short of the hit
        theta = theta[nearest]
        t = np.arange(int(length.max() / self.res) + 1, dtype=np.float32) * self.res
        mask = t[None, :] < length[:, None]
        xs = x + np.cos(theta)[:, None] * t[None, :]
        ys = y + np.sin(theta)[:, None] * t[None, :]
        return np.unique(self.cells(np.column_stack((xs[mask], ys[mask]))))

    def _integrate(self, occ, ground, fp, pose, now, odom_pose):
        x, y, _ = pose
        ground_cells = np.unique(self.cells(ground))
        free_ground = ground_cells[~np.isin(ground_cells, occ, assume_unique=True)]
        ray_free = self._rays(x, y)
        ray_free = ray_free[~np.isin(ray_free, occ, assume_unique=True)]
        touched = np.unique(np.concatenate((occ, free_ground, ray_free, fp)))
        if not touched.size:
            return False
        flat = self.L.reshape(-1)
        before = self.classify(flat[touched])
        flat[ray_free] -= L_FREE_RAY
        flat[free_ground] -= L_FREE_GROUND
        flat[occ] += L_OCC
        flat[fp] = np.minimum(flat[fp], FREE_T - 0.6)
        flat[touched] = np.clip(flat[touched], L_MIN, L_MAX)
        after = self.classify(flat[touched])
        self.known += int((after != UNKNOWN).sum()) - int((before != UNKNOWN).sum())
        iy, ix = np.divmod(touched, self.n)
        box = [int(ix.min()), int(iy.min()), int(ix.max()), int(iy.max())]
        if self.bbox is None:
            self.bbox = box
        else:
            self.bbox = [min(self.bbox[0], box[0]), min(self.bbox[1], box[1]),
                         max(self.bbox[2], box[2]), max(self.bbox[3], box[3])]
        self.last_pose = tuple(odom_pose)       # integration cadence follows odometry motion
        self.updated_at = now
        self.session.append((now, (x, y, pose[2]), self.scan.copy()))
        changed = bool((before != after).any())
        self.dirty = self.dirty or changed
        if self.dirty and now - self.rendered_at >= 0.5:
            self._render(now)
        return changed

    def _render(self, now):
        ix0, iy0, ix1, iy1 = self.bbox
        ix0, iy0 = max(0, ix0 - 2), max(0, iy0 - 2)
        ix1, iy1 = min(self.n - 1, ix1 + 2), min(self.n - 1, iy1 + 2)
        crop = self.classify(self.L[iy0:iy1 + 1, ix0:ix1 + 1])
        occupied = crop == OCCUPIED
        if occupied.any():
            lethal = distance_steps(occupied, int(round(LETHAL_R / self.res)))
            crop[(lethal <= int(round(LETHAL_R / self.res))) & (crop == FREE)] = INFLATED
        self.png = base64.b64encode(encode_png(crop, PALETTE)).decode("ascii")
        self.png_meta = {"origin": [round(self.x0 + ix0 * self.res, 3), round(self.y0 + iy0 * self.res, 3)],
                         "resolution": self.res, "width": int(crop.shape[1]), "height": int(crop.shape[0])}
        self.rendered_at = now
        self.dirty = False
        self.version += 1

    def info(self):
        return {"floor": None if self.floor is None else round(self.floor, 3),
                "status": self.status,
                "pose_source": self.pose_source,
                "frame": self.frame_stats,
                "correction": [round(self.corr[0], 3), round(self.corr[1], 3),
                               round(math.degrees(self.corr[2]), 2)],
                "match": self.match_info,
                "known_m2": round(self.known * self.res * self.res, 1),
                "resolution": self.res}

    def snapshot(self, start_xy, margin_m=1.0):
        """Copy the part of the grid needed for planning (map ∪ start + margin)."""
        with self.lock:
            if self.bbox is None or self.known == 0:
                raise PlanError("地图为空，请先遥控机器狗建图")
            m = int(margin_m / self.res)
            sx = int((start_xy[0] - self.x0) / self.res)
            sy = int((start_xy[1] - self.y0) / self.res)
            ix0 = max(0, min(self.bbox[0], sx) - m)
            iy0 = max(0, min(self.bbox[1], sy) - m)
            ix1 = min(self.n - 1, max(self.bbox[2], sx) + m)
            iy1 = min(self.n - 1, max(self.bbox[3], sy) + m)
            grid = self.L[iy0:iy1 + 1, ix0:ix1 + 1].copy()
        return grid, self.x0 + ix0 * self.res, self.y0 + iy0 * self.res, self.res


def plan_path(grid, ox, oy, res, start, goal):
    """A* on a 2x coarsened grid with inflation, then shortcut + resample.

    ``grid`` is a log-odds crop whose cell (0, 0) has its corner at (ox, oy).
    Returns an (M, 2) array of waypoints from start to goal.
    """
    lethal_steps = int(round(LETHAL_R / res))
    soft_steps = int(round(SOFT_R / res))
    smooth_steps = int(round(SMOOTH_R / res))
    h, w = grid.shape
    h2, w2 = (h + 1) // 2, (w + 1) // 2
    padded = np.zeros((h2 * 2, w2 * 2), np.float32)
    padded[:h, :w] = grid
    occupied = padded > OCC_T
    free = padded < FREE_T
    dist = distance_steps(occupied, soft_steps)

    def blocks(a):
        return a.reshape(h2, 2, w2, 2)

    c_free = dilate(blocks(free).any(axis=(1, 3)), True)   # close 1-cell gaps
    c_dist = blocks(dist).min(axis=(1, 3))
    trav = c_free & (c_dist > lethal_steps)
    soft = np.clip((soft_steps - c_dist) / float(soft_steps - lethal_steps), 0, 1)

    def fine(p):
        return int((p[1] - oy) / res), int((p[0] - ox) / res)

    def coarse(p):
        fy, fx = fine(p)
        return fy // 2, fx // 2

    gy, gx = fine(goal)
    if not (0 <= gy < h and 0 <= gx < w):
        raise PlanError("目标点超出已建地图范围")
    if occupied[gy, gx] or dist[gy, gx] == 0:
        raise PlanError("目标点在障碍物上")
    if dist[gy, gx] <= lethal_steps:
        raise PlanError("目标点离障碍物太近（< {:.2f} m）".format(LETHAL_R))
    gcy, gcx = gy // 2, gx // 2
    if not c_free[gcy, gcx]:
        raise PlanError("目标点在未探索区域")
    trav[gcy, gcx] = True
    scy, scx = coarse(start)
    if not (0 <= scy < h2 and 0 <= scx < w2):
        raise PlanError("机器狗当前位置不在地图内")
    # The robot physically stands here: let it leave the inflated zone.
    r = int(math.ceil(0.4 / (2 * res)))
    y0, y1 = max(0, scy - r), min(h2, scy + r + 1)
    x0, x1 = max(0, scx - r), min(w2, scx + r + 1)
    trav[y0:y1, x0:x1] |= c_dist[y0:y1, x0:x1] > 0

    trav_list = trav.ravel().tolist()
    mult = (1.0 + 3.0 * soft).ravel().tolist()
    start_i, goal_i = scy * w2 + scx, gcy * w2 + gcx
    moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)),
             (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]

    def heuristic(i):
        dy, dx = abs(i // w2 - gcy), abs(i % w2 - gcx)
        return (dx + dy) + (math.sqrt(2) - 2) * min(dx, dy)

    g_score = {start_i: 0.0}
    came = {}
    heap = [(heuristic(start_i), 0.0, start_i)]
    expanded = 0
    while heap:
        _, g, cur = heapq.heappop(heap)
        if cur == goal_i:
            break
        if g > g_score.get(cur, math.inf):
            continue
        expanded += 1
        if expanded > 400000:
            raise PlanError("规划超时，地图过大")
        cy, cx = divmod(cur, w2)
        for dy, dx, length in moves:
            ny, nx = cy + dy, cx + dx
            if not (0 <= ny < h2 and 0 <= nx < w2):
                continue
            nxt = ny * w2 + nx
            if not trav_list[nxt]:
                continue
            if dy and dx and not (trav_list[cy * w2 + nx] and trav_list[ny * w2 + cx]):
                continue                       # no corner cutting
            cost = g + length * 0.5 * (mult[cur] + mult[nxt])
            if cost < g_score.get(nxt, math.inf):
                g_score[nxt] = cost
                came[nxt] = cur
                heapq.heappush(heap, (cost + heuristic(nxt), cost, nxt))
    else:
        raise PlanError("找不到可通行路径")
    if goal_i not in came and goal_i != start_i:
        raise PlanError("找不到可通行路径")

    cells = [goal_i]
    while cells[-1] != start_i:
        cells.append(came[cells[-1]])
    cells.reverse()
    pts = [(ox + (2 * (c % w2) + 1) * res, oy + (2 * (c // w2) + 1) * res) for c in cells]
    pts[0], pts[-1] = tuple(start[:2]), tuple(goal[:2])

    def clearance(p):
        fy, fx = fine(p)
        if not (0 <= fy < dist.shape[0] and 0 <= fx < dist.shape[1]):
            return 0
        return int(dist[fy, fx])

    clear = [clearance(p) for p in pts]

    def segment_ok(a, b, need):
        n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / res) + 1)
        for t in np.linspace(0, 1, n):
            p = (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)
            cy, cx = coarse(p)
            if not (0 <= cy < h2 and 0 <= cx < w2) or not trav[cy, cx] or clearance(p) < need:
                return False
        return True

    smooth = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = min(len(pts) - 1, i + 60)
        while j > i + 1:
            need = min(smooth_steps, min(clear[i:j + 1]))
            if segment_ok(pts[i], pts[j], need):
                break
            j -= 1
        smooth.append(pts[j])
        i = j

    poly = np.array(smooth, np.float64)
    seg = np.hypot(*np.diff(poly, axis=0).T)
    total = float(seg.sum())
    if total < 1e-6:
        return poly[-1:].copy()
    s = np.concatenate(([0.0], np.cumsum(seg)))
    samples = np.append(np.arange(0.0, total, 0.1), total)
    return np.column_stack((np.interp(samples, s, poly[:, 0]), np.interp(samples, s, poly[:, 1])))


class Navigator:
    """Pure-pursuit follower plus its own stop checks. No I/O."""

    ACTIVE = ("planning", "following", "blocked")

    def __init__(self):
        self.goal_id = 0
        self.reset()

    def reset(self):
        self.state = "idle"
        self.message = ""
        self.goal = None
        self.path = None
        self.path_list = []
        self.idx = 0
        self.command = (0.0, 0.0, 0.0)
        self.turning = False
        self.final = False
        self.blocked_since = None
        self.best = math.inf
        self.best_at = 0.0
        self.remaining = None
        self.last_plan = 0.0
        self.debug = {}

    @property
    def active(self):
        return self.state in self.ACTIVE

    def set_goal(self, x, y, yaw, now):
        self.reset()
        self.goal_id += 1
        self.goal = (float(x), float(y), wrap_angle(float(yaw)))
        self.state = "planning"
        self.message = "规划路径中"
        self.best_at = now
        return self.goal_id

    def _stop(self, state, message):
        self.state = state
        self.message = message
        self.command = (0.0, 0.0, 0.0)

    def cancel(self, message="已取消导航"):
        if self.active:
            self._stop("canceled", message)

    def fail(self, message):
        if self.active:
            self._stop("failed", message)

    def plan_done(self, goal_id, path, error, now):
        if goal_id != self.goal_id or not self.active:
            return
        self.last_plan = now
        if error:
            if self.state == "planning":
                self.fail(error)
            else:
                self.message = "重新规划失败：" + error
            return
        self.path = np.asarray(path, np.float64)
        self.path_list = np.round(self.path[::2], 2).tolist() + [np.round(self.path[-1], 2).tolist()]
        self.idx = 0
        self.turning = False
        self.best, self.best_at = math.inf, now     # progress is measured per path
        if self.state == "planning":
            self.state = "following"
            self.message = ""

    def snapshot(self):
        return {"state": self.state, "message": self.message, "debug": self.debug,
                "goal": None if self.goal is None else [round(v, 3) for v in self.goal],
                "path": self.path_list if self.active else [],
                "remaining": None if self.remaining is None else round(self.remaining, 2)}

    def tick(self, now, pose, obstacles, odom_age, scan_age):
        """Update ``self.command`` (vx, vy, wz). Returns True if a replan is wanted."""
        if not self.active:
            self.command = (0.0, 0.0, 0.0)
            return False
        if pose is None or odom_age > 0.5 or scan_age > 1.0:
            self.command = (0.0, 0.0, 0.0)
            if pose is None or odom_age > 2.0 or scan_age > 3.0:
                self.fail("位姿或点云中断，导航已停止")
            return False
        if self.state == "planning" or self.path is None:
            self.command = (0.0, 0.0, 0.0)
            return False
        x, y, yaw = pose
        gx, gy, gyaw = self.goal
        d_goal = math.hypot(gx - x, gy - y)
        # Progress along the path (not straight-line: detours move away first).
        along = float(np.hypot(*np.diff(self.path[self.idx:], axis=0).T).sum()) if len(self.path) > 1 else 0.0
        self.remaining = along + math.hypot(self.path[self.idx][0] - x, self.path[self.idx][1] - y)
        if self.remaining < self.best - 0.1:
            self.best, self.best_at = self.remaining, now
        elif not self.final and now - self.best_at > PROGRESS_TIMEOUT:
            self.fail("{:.0f} 秒内没有沿路径前进，导航已停止".format(PROGRESS_TIMEOUT))
            return False

        local = np.zeros((0, 2))
        world_near = np.zeros((0, 2))
        if obstacles is not None and len(obstacles):
            rel = np.asarray(obstacles, np.float64) - (x, y)
            near = (rel ** 2).sum(axis=1) < 3.5 ** 2
            world_near = np.asarray(obstacles, np.float64)[near]
            c, s = math.cos(yaw), math.sin(yaw)
            rel = rel[near]
            local = np.column_stack((c * rel[:, 0] + s * rel[:, 1], -s * rel[:, 0] + c * rel[:, 1]))
            inside = (np.abs(local[:, 0]) < BODY_HALF_LEN) & (np.abs(local[:, 1]) < BODY_HALF_WIDTH)
            local = local[~inside]
        rotate_blocked = int(((local ** 2).sum(axis=1) < ROTATE_CLEARANCE ** 2).sum()) >= 3

        if self.final or d_goal < GOAL_TOL:
            self.final = True
            err = wrap_angle(gyaw - yaw)
            if abs(err) < YAW_TOL:
                self._stop("arrived", "已到达目标")
                return False
            if rotate_blocked:
                self._stop("arrived", "已到达目标位置，周围空间不足，未调整朝向")
                return False
            wz = math.copysign(min(NAV_WMAX, max(0.2, 1.5 * abs(err))), err)
            self.command = (0.0, 0.0, wz)
            self.debug = {"phase": "final_turn", "yaw_err": round(err, 3)}
            self.state, self.message = "following", "调整朝向"
            return False

        want_replan = False
        ahead = self.path[self.idx:self.idx + 30]
        if len(world_near) and len(ahead):
            d = np.sqrt(((ahead[:, None, :] - world_near[None, :, :]) ** 2).sum(axis=2)).min()
            want_replan = d < LETHAL_R - 0.05

        window = self.path[self.idx:self.idx + 40]
        self.idx += int(np.argmin(np.hypot(window[:, 0] - x, window[:, 1] - y)))
        target = self.path[-1]
        for point in self.path[self.idx:]:
            if math.hypot(point[0] - x, point[1] - y) >= LOOKAHEAD:
                target = point
                break
        dx, dy = target[0] - x, target[1] - y
        c, s = math.cos(yaw), math.sin(yaw)
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        alpha = math.atan2(ly, lx)
        if self.turning and abs(alpha) < 0.3:
            self.turning = False
        elif not self.turning and abs(alpha) > 0.9:
            self.turning = True

        if self.turning:
            blocked = rotate_blocked
            command = (0.0, 0.0, math.copysign(min(NAV_WMAX, max(0.3, 1.2 * abs(alpha))), alpha))
            self.debug = {"phase": "turn_in_place", "alpha": round(alpha, 3),
                          "blocked_by": "rotate_clearance" if blocked else None}
        else:
            kappa = 2.0 * math.sin(alpha) / LOOKAHEAD
            # Gentle curvature slowdown, with a floor so the gait actually translates.
            v = NAV_VMAX / (1.0 + 0.6 * abs(kappa))
            v = min(v, 0.6 * d_goal + 0.1)
            v = max(v, NAV_VMIN if d_goal > 0.3 else 0.10)
            wz = max(-NAV_WMAX, min(NAV_WMAX, v * kappa))
            stop = max(FRONT_STOP, 0.35 + 0.5 * v)       # 0.55 m ahead of the nose at 0.4 m/s
            reach = BODY_HALF_LEN + min(stop, d_goal + 0.05)
            front = (local[:, 0] > BODY_HALF_LEN - 0.03) & (local[:, 0] < reach) & (np.abs(local[:, 1]) < BODY_HALF_WIDTH + 0.06)
            blocked = int(front.sum()) >= 3
            command = (v, 0.0, wz)
            self.debug = {"phase": "pursuit", "alpha": round(alpha, 3), "kappa": round(kappa, 3),
                          "front_points": int(front.sum()), "blocked_by": "front" if blocked else None}

        if blocked:
            self.command = (0.0, 0.0, 0.0)
            if self.blocked_since is None:
                self.blocked_since = now
            elif now - self.blocked_since > BLOCKED_TIMEOUT:
                self.fail("前方受阻超过 {:.0f} 秒，导航已停止".format(BLOCKED_TIMEOUT))
                return False
            self.state, self.message = "blocked", "前方受阻，等待重新规划"
            return now - self.last_plan >= REPLAN_INTERVAL
        self.blocked_since = None
        self.state, self.message = "following", ""
        self.command = command
        self.debug["command"] = [round(c, 3) for c in command]
        return want_replan and now - self.last_plan >= REPLAN_INTERVAL
