"""Offline checks for the minimal 2D SLAM / planner / follower. No robot."""
import base64
import math
import struct
import unittest
import zlib
from unittest.mock import Mock

import numpy as np

from go2_web import slam
from go2_web.__main__ import RobotBridge
from go2_web.slam import Navigator, OccupancyMap, PlanError, plan_path

FLOOR = -0.30          # odom z of the floor (robot body at z = 0)


def floor_points(x0, x1, y0, y1, step=0.05):
    xs, ys = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1, step))
    return np.column_stack((xs.ravel(), ys.ravel(), np.full(xs.size, FLOOR)))


def wall(x0, y0, x1, y1, step=0.05):
    # round(): exact 5 cm spacing like Go2's voxel lattice (int() lost a point to float error)
    n = int(round(math.hypot(x1 - x0, y1 - y0) / step)) + 1
    t = np.linspace(0, 1, n)
    pts = []
    for z in np.arange(FLOOR + 0.2, FLOOR + 0.6, 0.05):
        pts.append(np.column_stack((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, np.full(n, z))))
    return np.vstack(pts)


def room(extra=()):
    """4 m x 4 m room centred at the origin, plus optional obstacles."""
    parts = [floor_points(-2, 2, -2, 2), wall(-2, -2, 2, -2), wall(-2, 2, 2, 2),
             wall(-2, -2, -2, 2), wall(2, -2, 2, 2), *extra]
    return np.vstack(parts).astype(np.float32)


def cls_at(m, x, y):
    ix = int((x - m.x0) / m.res)
    iy = int((y - m.y0) / m.res)
    return int(m.classify(m.L[iy, ix:ix + 1])[0])


def build(points, pose=(0, 0, 0, 0), frames=3):
    m = OccupancyMap(size_m=12)
    for k in range(frames):
        m.process(points, pose, now=k * 1.0)
    return m


class MappingTests(unittest.TestCase):
    def test_floor_free_walls_occupied_outside_unknown(self):
        m = build(room())
        self.assertAlmostEqual(m.floor, FLOOR, delta=0.03)
        self.assertEqual(cls_at(m, 1.0, 0.5), slam.FREE)
        self.assertEqual(cls_at(m, 2.0, 0.3), slam.OCCUPIED)
        self.assertEqual(cls_at(m, 3.0, 0.0), slam.UNKNOWN)
        self.assertGreater(m.info()["known_m2"], 10)
        self.assertTrue(len(m.scan_list) > 0)

    def test_removed_obstacle_is_cleared(self):
        box = wall(0.8, -0.3, 0.8, 0.3)
        m = build(room([box]))
        self.assertEqual(cls_at(m, 0.8, 0.0), slam.OCCUPIED)
        for k in range(12):
            m.process(room(), (0, 0, 0, 0), now=10 + k)
        self.assertEqual(cls_at(m, 0.8, 0.0), slam.FREE)

    def test_static_scene_settles_and_skips(self):
        m = build(room(), frames=6)
        version = m.version
        self.assertFalse(m.should_integrate((0, 0, 0, 0), m.updated_at + 0.3))
        self.assertTrue(m.should_integrate((0.1, 0, 0, 0), m.updated_at + 0.3))
        # Same scene again: classification does not change, no new PNG.
        self.assertFalse(m.process(room(), (0, 0, 0, 0), now=m.updated_at + 1.0))
        self.assertEqual(m.version, version)

    def test_robot_footprint_never_occupied(self):
        body = wall(0.1, -0.1, 0.1, 0.1)          # voxels "inside" the robot
        m = build(room([body]))
        self.assertEqual(cls_at(m, 0.1, 0.0), slam.FREE)

    def test_png_is_valid(self):
        m = build(room())
        raw = base64.b64decode(m.png)
        self.assertEqual(raw[:8], b"\x89PNG\r\n\x1a\n")
        width, height = struct.unpack(">II", raw[16:24])
        self.assertEqual((width, height), (m.png_meta["width"], m.png_meta["height"]))
        idat = raw.index(b"IDAT")
        size = struct.unpack(">I", raw[idat - 4:idat])[0]
        rows = zlib.decompress(raw[idat + 4:idat + 4 + size])
        self.assertEqual(len(rows), height * (width + 1))

    def test_reset_clears(self):
        m = build(room())
        v = m.version
        m.reset()
        self.assertIsNone(m.png)
        self.assertEqual(m.known, 0)
        self.assertGreater(m.version, v)


class FloorRobustnessTests(unittest.TestCase):
    def scene(self):
        pts = room([wall(0.8, -0.3, 0.8, 0.3)])
        walls = pts[pts[:, 2] > FLOOR + 0.1]
        mirror = walls.copy()
        mirror[:, 2] = 2 * FLOOR - mirror[:, 2]             # glossy floor reflections
        overhead = floor_points(-1.5, 1.5, -1.5, 1.5, 0.1)
        overhead[:, 2] = FLOOR + 0.85                         # table top / shelf above robot + 0.2 m
        return np.vstack((pts, mirror, overhead)).astype(np.float32)

    def test_glossy_floor_reflections_and_overhead(self):
        m = build(self.scene())
        self.assertAlmostEqual(m.floor, FLOOR, delta=0.03)
        self.assertEqual(cls_at(m, 0.4, 0.5), slam.FREE)       # floor stays free
        self.assertEqual(cls_at(m, -1.0, -1.0), slam.FREE)     # under the overhead plane
        self.assertEqual(cls_at(m, 0.8, 0.0), slam.OCCUPIED)   # the real box
        free = (m.classify(m.L) == slam.FREE).sum()
        occ = (m.classify(m.L) == slam.OCCUPIED).sum()
        self.assertGreater(free, 8 * occ)

    def test_tilted_floor_far_cells_stay_free(self):
        pts = room().copy()
        tilt = math.tan(math.radians(1.5))
        pts[:, 2] += pts[:, 0] * tilt                         # 1.5 deg slope along x
        m = build(pts)
        self.assertEqual(cls_at(m, 1.7, 0.0), slam.FREE)
        self.assertEqual(cls_at(m, -1.7, 0.0), slam.FREE)
        self.assertEqual(cls_at(m, 2.0, 0.3), slam.OCCUPIED)


def seg(a, b, step=0.05):
    n = int(round(math.dist(a, b) / step)) + 1
    t = np.linspace(0, 1, n)[:, None]
    return np.asarray(a, float) + (np.asarray(b, float) - np.asarray(a, float)) * t


def observe(wall_xy, floor_xy, p, rng, radius=5.0):
    w = wall_xy[np.hypot(*(wall_xy - p).T) < radius]
    f = floor_xy[np.hypot(*(floor_xy - p).T) < radius]
    zs = np.arange(FLOOR + 0.35, FLOOR + 0.61, 0.05)
    W = np.column_stack((np.repeat(w, len(zs), 0), np.tile(zs, len(w))))
    P = np.vstack((W, np.column_stack((f, np.full(len(f), FLOOR)))))
    P[:, :2] += rng.normal(0, 0.015, (len(P), 2))
    return P


def drive(wall_xy, floor_xy, way, drift_deg_per_step, scale=1.0, step=0.2, match=True):
    """Dead-reckoned odometry with yaw drift; returns (map, true poses, odom poses)."""
    slam.MATCH_ENABLED = match
    try:
        rng = np.random.default_rng(3)
        pts = np.vstack([seg(a, b, step)[:-1] for a, b in zip(way, way[1:])] + [np.array([way[-1]])])
        yaws = np.arctan2(np.gradient(pts[:, 1]), np.gradient(pts[:, 0]))
        m = OccupancyMap(size_m=40)
        a, odom = 0.0, pts[0].copy()
        true, est = [], []
        for k, (p, yaw) in enumerate(zip(pts, yaws)):
            if k:
                move = pts[k] - pts[k - 1]
                c, s_ = math.cos(a), math.sin(a)
                odom = odom + scale * np.array([c * move[0] - s_ * move[1], s_ * move[0] + c * move[1]])
                a += math.radians(drift_deg_per_step)
            Ra = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
            P = observe(wall_xy, floor_xy, p, rng)
            P[:, :2] = (P[:, :2] - p) @ Ra.T + odom
            pose = (odom[0], odom[1], 0.0, yaw + a)
            m.process(P, pose, now=k * 0.5)
            true.append((p[0], p[1], yaw))
            est.append(m.to_map(pose))
        return m, np.array(true), np.array(est)
    finally:
        slam.MATCH_ENABLED = True


def l_corridor():
    walls = [((-1, -1), (9, -1)), ((9, -1), (9, 9)), ((-1, 1), (7, 1)), ((7, 1), (7, 9)),
             ((-1, -1), (-1, 1)), ((7, 9), (9, 9))]
    pts = [seg(a, b) for a, b in walls]
    for cx, cy in [(2.5, -0.85), (5.0, 0.85), (8.85, 4.0), (7.15, 6.5)]:
        pts.append(np.array([(cx + dx, cy + dy) for dx in np.arange(-0.1, 0.11, 0.05)
                             for dy in np.arange(-0.1, 0.11, 0.05)]))
    fx, fy = np.meshgrid(np.arange(-1, 9, 0.1), np.arange(-1, 9, 0.1))
    f = np.column_stack((fx.ravel(), fy.ravel()))
    inside = ((f[:, 1] > -1) & (f[:, 1] < 1)) | ((f[:, 0] > 7) & (f[:, 0] < 9))
    return np.vstack(pts), f[inside]


class DriftCorrectionTests(unittest.TestCase):
    """Go2 Air odometry (legs + IMU) has slowly drifting yaw; scan matching must undo it."""

    def wall_precision(self, m, wall_xy):
        cls = m.classify(m.L)
        oy, ox = np.nonzero(cls == slam.OCCUPIED)
        truth = np.zeros_like(cls, bool)
        ti = ((wall_xy - (m.x0, m.y0)) / m.res).astype(int)
        truth[ti[:, 1], ti[:, 0]] = True
        return float((slam.distance_steps(truth, 3)[oy, ox] <= 2).mean())

    def test_yaw_drift_ghosting_removed(self):
        walls, floor = l_corridor()
        way = [(0, 0), (8, 0), (8, 8), (8, 0), (0, 0)]
        m_off, true, est_off = drive(walls, floor, way, 0.1, match=False)
        m_on, true, est_on = drive(walls, floor, way, 0.1, match=True)
        drift = abs(math.degrees(slam.wrap_angle(est_off[-1, 3] - true[-1, 2])))
        self.assertGreater(drift, 10)                                 # the test really drifts
        self.assertLess(self.wall_precision(m_off, walls), 0.6)       # ghost walls without matching
        self.assertGreater(self.wall_precision(m_on, walls), 0.95)
        yaw_err = np.degrees(np.abs([slam.wrap_angle(e[3] - t[2]) for e, t in zip(est_on, true)]))
        pos_err = np.hypot(est_on[:, 0] - true[:, 0], est_on[:, 1] - true[:, 1])
        self.assertLess(yaw_err.max(), 1.0)
        self.assertLess(pos_err.max(), 0.15)

    def test_featureless_corridor_not_pulled_back(self):
        walls = np.vstack([seg((-1, -1), (20, -1)), seg((-1, 1), (20, 1))])
        fx, fy = np.meshgrid(np.arange(-1, 20, 0.1), np.arange(-0.9, 0.95, 0.1))
        floor = np.column_stack((fx.ravel(), fy.ravel()))
        m, true, est = drive(walls, floor, [(0, 0), (16, 0)], 0.1, scale=1.05)
        # lateral position and yaw are observable from the walls ...
        self.assertLess(abs(est[-1, 1]), 0.1)
        self.assertLess(abs(math.degrees(est[-1, 3])), 1.0)
        # ... distance along a featureless corridor is not: it must follow odometry, not shrink.
        self.assertAlmostEqual(est[-1, 0], 16 * 1.05, delta=0.3)

    def test_static_robot_correction_stays_put(self):
        m = OccupancyMap(size_m=12)
        pts = room([wall(0.8, -0.3, 0.8, 0.3), wall(-1.0, 0.5, -0.5, 1.0)])
        rng = np.random.default_rng(4)
        for k in range(15):
            noisy = pts.copy()
            noisy[:, :2] += rng.normal(0, 0.01, (len(pts), 2))
            m.process(noisy, (0.0, 0.0, 0.0, 0.0), now=k * 1.0)
        dx, dy, dyaw = m.corr
        self.assertLess(math.hypot(dx, dy), 0.05)
        self.assertLess(abs(math.degrees(dyaw)), 0.3)


class PlannerTests(unittest.TestCase):
    def setUp(self):
        # Wall across the room with a gap at the top: forces a detour.
        self.m = build(room([wall(0, -2, 0, 0.9)]))

    def plan(self, start, goal):
        grid, ox, oy, res = self.m.snapshot(start)
        return plan_path(grid, ox, oy, res, start, goal)

    def test_path_goes_around_wall_with_clearance(self):
        path = self.plan((-1.2, -1.0), (1.2, -1.0))
        np.testing.assert_allclose(path[0], (-1.2, -1.0))
        np.testing.assert_allclose(path[-1], (1.2, -1.0))
        self.assertGreater(path[:, 1].max(), 0.9)          # went through the gap
        wall_pts = wall(0, -2, 0, 0.9)[:, :2]
        d = np.sqrt(((path[:, None, :] - wall_pts[None]) ** 2).sum(axis=2)).min()
        self.assertGreater(d, slam.LETHAL_R - 0.06)
        steps = np.hypot(*np.diff(path, axis=0).T)
        self.assertLessEqual(steps.max(), 0.11)

    def test_goal_errors(self):
        with self.assertRaisesRegex(PlanError, "未探索|超出"):
            self.plan((-1.2, -1.0), (3.0, 0.0))
        with self.assertRaisesRegex(PlanError, "障碍物"):
            self.plan((-1.2, -1.0), (0.0, -1.0))
        with self.assertRaisesRegex(PlanError, "太近"):
            self.plan((-1.2, -1.0), (1.8, -1.0))

    def test_empty_map(self):
        with self.assertRaisesRegex(PlanError, "地图为空"):
            OccupancyMap(size_m=12).snapshot((0, 0))


def simulate(nav, pose, obstacles=None, ticks=600, dt=0.1):
    x, y, yaw = pose
    for k in range(ticks):
        nav.tick(k * dt, (x, y, yaw), obstacles, 0.0, 0.0)
        vx, _, wz = nav.command
        x += vx * math.cos(yaw) * dt
        y += vx * math.sin(yaw) * dt
        yaw = slam.wrap_angle(yaw + wz * dt)
        assert abs(vx) <= slam.NAV_VMAX + 1e-9 and abs(wz) <= slam.NAV_WMAX + 1e-9
        if not nav.active:
            break
    return x, y, yaw


class NavigatorTests(unittest.TestCase):
    def test_follows_path_and_final_heading(self):
        m = build(room([wall(0, -2, 0, 0.9)]))
        grid, ox, oy, res = m.snapshot((-1.2, -1.0))
        path = plan_path(grid, ox, oy, res, (-1.2, -1.0), (1.2, -1.0))
        nav = Navigator()
        gid = nav.set_goal(1.2, -1.0, math.pi / 2, 0.0)
        nav.plan_done(gid, path, None, 0.0)
        x, y, yaw = simulate(nav, (-1.2, -1.0, 0.0), obstacles=m.scan)
        self.assertEqual(nav.state, "arrived", nav.message)
        self.assertLess(math.hypot(x - 1.2, y + 1.0), slam.GOAL_TOL + 0.02)
        self.assertLess(abs(slam.wrap_angle(yaw - math.pi / 2)), slam.YAW_TOL + 0.02)

    def test_stops_for_obstacle_ahead_and_asks_replan(self):
        nav = Navigator()
        gid = nav.set_goal(3.0, 0.0, 0.0, 0.0)
        nav.plan_done(gid, np.column_stack((np.arange(0, 3.01, 0.1), np.zeros(31))), None, 0.0)
        blocker = np.column_stack((np.full(9, 0.55), np.linspace(-0.1, 0.1, 9)))
        want = nav.tick(5.0, (0.0, 0.0, 0.0), blocker, 0.0, 0.0)
        self.assertEqual(nav.command, (0.0, 0.0, 0.0))
        self.assertEqual(nav.state, "blocked")
        self.assertTrue(want)
        for t in np.arange(5.1, 5.0 + slam.BLOCKED_TIMEOUT + 1, 0.5):
            nav.tick(t, (0.0, 0.0, 0.0), blocker, 0.0, 0.0)
        self.assertEqual(nav.state, "failed")

    def test_stale_data_stops(self):
        nav = Navigator()
        gid = nav.set_goal(2.0, 0.0, 0.0, 0.0)
        nav.plan_done(gid, np.column_stack((np.arange(0, 2.01, 0.1), np.zeros(21))), None, 0.0)
        nav.tick(0.1, (0, 0, 0), None, 0.0, 0.0)
        self.assertGreater(nav.command[0], 0)
        nav.tick(0.2, (0, 0, 0), None, 0.8, 0.0)       # odom 0.8 s old
        self.assertEqual(nav.command, (0.0, 0.0, 0.0))
        self.assertTrue(nav.active)
        nav.tick(0.3, (0, 0, 0), None, 2.5, 0.0)
        self.assertEqual(nav.state, "failed")

    def test_stale_plan_result_ignored(self):
        nav = Navigator()
        old = nav.set_goal(1, 0, 0, 0.0)
        nav.set_goal(2, 0, 0, 0.0)
        nav.plan_done(old, np.zeros((2, 2)), None, 0.0)
        self.assertEqual(nav.state, "planning")


class BridgeNavTests(unittest.IsolatedAsyncioTestCase):
    def make_bridge(self):
        bridge = RobotBridge()
        bridge.ready = Mock(return_value=True)
        bridge.conn = Mock()
        bridge.send = Mock()
        bridge.mode = "normal"
        bridge.connected = bridge.armed = True
        bridge.map = build(room())
        bridge.odom = {"position": [-1.0, 0.0, 0.0], "quaternion": [0, 0, 0, 1]}
        bridge.odom_at = 1e12            # "fresh" far into the future
        bridge.map.scan_at = 1e12
        return bridge

    async def test_goal_plans_and_drives_with_direct_move(self):
        bridge = self.make_bridge()
        bridge.avoid = True              # navigation must still bypass the joystick path
        bridge.deadline = 1e12
        bridge.start_nav(1.0, 0.0, 0.0)
        await bridge.plan_task
        self.assertEqual(bridge.nav.state, "following", bridge.nav.message)
        task = asyncio.create_task(bridge.watchdog())
        await asyncio.sleep(0.35)
        task.cancel()
        names = [c.args[0] for c in bridge.send.call_args_list]
        self.assertIn("Move", names)
        moves = [c.args[1] for c in bridge.send.call_args_list if c.args[0] == "Move"]
        self.assertTrue(any(m["x"] > 0 for m in moves))
        wireless = [c for c in bridge.conn.datachannel.pub_sub.publish_without_callback.call_args_list
                    if "wirelesscontroller" in str(c.args[0])]
        self.assertEqual(wireless, [])

    async def test_disarm_and_heartbeat_loss_cancel(self):
        bridge = self.make_bridge()
        bridge.start_nav(1.0, 0.0, 0.0)
        await bridge.plan_task
        bridge.disarm()
        self.assertEqual(bridge.nav.state, "canceled")
        bridge.armed = True
        bridge.start_nav(1.0, 0.0, 0.0)
        await bridge.plan_task
        bridge.deadline = 0              # page stopped sending heartbeats
        task = asyncio.create_task(bridge.watchdog())
        await asyncio.sleep(0.12)
        task.cancel()
        self.assertEqual(bridge.nav.state, "canceled")

    async def test_sport_state_pose_fallback(self):
        """Go2 Air: no robot_pose, so sportmodestate position + IMU drive mapping."""
        import time as _time
        bridge = RobotBridge()
        bridge.state = {"position": [0.5, -0.2, 0.0], "imu_state": {"quaternion": [
            math.cos(0.4 / 2), 0.0, 0.0, math.sin(0.4 / 2)]}}       # w, x, y, z
        bridge.state_at = _time.monotonic()
        pose, age, source = bridge.pose_source()
        self.assertEqual(source, "sportmodestate")
        self.assertAlmostEqual(pose[3], 0.4, places=6)
        bridge.map_frame(room())
        await asyncio.sleep(0.2)                    # worker thread
        self.assertIsNotNone(bridge.map.floor)
        self.assertGreater(bridge.map.known, 0)
        self.assertEqual(bridge.map.info()["pose_source"], "sportmodestate")
        bridge.state_at -= 5                       # stale pose: mapping pauses with a reason
        bridge.map_frame(room())
        self.assertIn("没有实时位姿", bridge.map.status)

    async def test_requires_fresh_odom_and_map(self):
        bridge = self.make_bridge()
        bridge.odom_at = 0
        with self.assertRaisesRegex(ValueError, "位姿"):
            bridge.start_nav(1.0, 0.0, 0.0)
        bridge.odom_at = 1e12
        bridge.map.reset()
        with self.assertRaisesRegex(ValueError, "地图为空"):
            bridge.start_nav(1.0, 0.0, 0.0)


import asyncio  # noqa: E402  (used by the async tests above)

if __name__ == "__main__":
    unittest.main()
