import argparse
import asyncio
import base64
import contextlib
import ipaddress
import json
import math
import os
import re
import socket as pysocket
import time
from pathlib import Path

import numpy as np
from aiohttp import web, WSMsgType
from unitree_webrtc_connect import (
    UnitreeWebRTCConnection, WebRTCConnectionMethod, RTC_TOPIC, SPORT_CMD,
    DATA_CHANNEL_TYPE,
)
from unitree_webrtc_connect.constants import OBSTACLES_AVOID_API
from unitree_webrtc_connect.multicast_scanner import discover_ip_sn

from go2_web.slam import Navigator, OccupancyMap, PlanError, plan_path

STATIC = Path(__file__).parent / "static"
# 后端速度上限 (x m/s, y m/s, yaw rad/s)。避障模式下按同一比例换算成摇杆量。
DRIVE_LIMITS = (0.6, 0.3, 1.0)


class VisualizationDecoder:
    """Keep the original voxel payload for unitree_ui's browser mesh decoder."""

    def __init__(self, decoder):
        self.decoder = decoder

    def decode(self, compressed_data, metadata):
        result = self.decoder.decode(compressed_data, metadata)
        result["compressed"] = base64.b64encode(compressed_data).decode("ascii")
        return result


class RobotBridge:
    def __init__(self):
        self.conn = None
        self.connected = False
        self.armed = False
        self.owner = None
        self.command = (0.0, 0.0, 0.0)
        self.deadline = 0
        self.moving = False
        self.state = {}
        self.motors = []
        self.state_at = 0
        self.points = []
        self.points_at = 0
        self.point_version = 0
        self.voxel = None
        self.odom = None
        self.odom_at = 0
        self.lidar_topic = RTC_TOPIC["ULIDAR_ARRAY"]
        self.lidar_meta = {}
        self.lidar_error = ""
        self.jpeg = None
        self.video_at = 0
        self.video_version = 0
        self.mode = "未知"
        # 避障开关：True 时走 rt/wirelesscontroller（模拟摇杆，经避障服务过滤），
        # False 时走 SPORT_MOD Move（直接速度指令，无避障）。None 表示状态未知。
        self.avoid = None
        self.robot_ip = None
        self.ip_source = ""
        # 在线 2D 建图 + 导航（只在本次开机内有效，关机即丢弃）
        self.map = OccupancyMap()
        self.map_busy = False
        self.nav = Navigator()
        self.plan_task = None
        self.nav_moving = False
        self.lock = asyncio.Lock()
        self.error = ""

    def ready(self):
        return (self.connected and self.conn is not None
                and self.conn.pc.connectionState == "connected"
                and self.conn.datachannel.channel.readyState == "open")

    def send(self, name, parameter=None):
        if not self.ready():
            raise ValueError("机器人未连接或数据通道不可用")
        payload = {"header": {"identity": {"id": time.time_ns() % 2147483647,
                    "api_id": SPORT_CMD[name]}},
                   "parameter": json.dumps(parameter) if parameter else ""}
        self.conn.datachannel.pub_sub.publish_without_callback(
            RTC_TOPIC["SPORT_MOD"], payload, DATA_CHANNEL_TYPE["REQUEST"])

    def stop(self):
        self.command = (0.0, 0.0, 0.0)
        self.deadline = 0
        if self.ready():
            if self.avoid:
                self.joystick(0.0, 0.0, 0.0)
            self.send("Move", {"x": 0, "y": 0, "z": 0})
            self.send("StopMove")
        self.moving = False

    def joystick(self, x, y, z):
        """把速度指令按后端上限换算成摇杆量，发到 rt/wirelesscontroller。

        摇杆约定与 unitree_ui 一致：ly 前为正，lx 右为正，rx 右转为正；
        而速度指令 y 左为正、z 左转（逆时针）为正，因此 y、z 取反。
        """
        def ratio(value, limit):
            return round(max(-1.0, min(1.0, value / limit)), 3)
        self.conn.datachannel.pub_sub.publish_without_callback(
            RTC_TOPIC["WIRELESS_CONTROLLER"],
            {"lx": ratio(-y, DRIVE_LIMITS[1]), "ly": ratio(x, DRIVE_LIMITS[0]),
             "rx": ratio(-z, DRIVE_LIMITS[2]), "ry": 0.0, "keys": 0})

    async def query_avoid(self):
        response = await self.rpc(RTC_TOPIC["OBSTACLES_AVOID"],
                                  {"api_id": OBSTACLES_AVOID_API["SWITCH_GET"]})
        enabled = json.loads(response["data"]["data"]).get("enable")
        if not isinstance(enabled, bool):
            raise ValueError("避障状态返回格式无法识别")
        self.avoid = enabled
        return enabled

    async def set_avoid(self, enable):
        # 切换通道前先停车并锁定，避免两个通道的指令交叠。
        self.disarm()
        await self.rpc(RTC_TOPIC["OBSTACLES_AVOID"],
                       {"api_id": OBSTACLES_AVOID_API["SWITCH_SET"],
                        "parameter": {"enable": bool(enable)}})
        try:
            actual = await self.query_avoid()
        except (ValueError, KeyError, TypeError, asyncio.TimeoutError):
            self.avoid = None
            raise ValueError("避障开关已发送，但无法读回实际状态，请勿依赖避障")
        if actual != bool(enable):
            raise ValueError("机器人未按要求{}避障".format("开启" if enable else "关闭"))
        return actual

    def disarm(self):
        self.armed = False
        self.nav.cancel("控制已锁定，导航已停止")
        self.stop()

    POSE_MAX_AGE = 0.5

    def pose_source(self):
        """Return ((x, y, z, yaw), age_s, source) or (None, inf, None).

        Prefer rt/utlidar/robot_pose. Go2 Air does not publish it, so fall
        back to rt/lf/sportmodestate position + IMU quaternion, which is what
        unitree_ui also uses to place the robot in the voxel map frame.
        """
        now = time.monotonic()
        if self.odom and now - self.odom_at <= self.POSE_MAX_AGE:
            x, y, z = self.odom["position"]
            qx, qy, qz, qw = self.odom["quaternion"]
            yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            return (x, y, z, yaw), now - self.odom_at, "robot_pose"
        state = self.state if isinstance(self.state, dict) else {}
        position = state.get("position")
        imu = state.get("imu_state") or {}
        yaw = None
        quat = imu.get("quaternion")
        if isinstance(quat, (list, tuple)) and len(quat) == 4:
            qw, qx, qy, qz = quat                     # sport state order is w, x, y, z
            yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        elif isinstance(imu.get("rpy"), (list, tuple)) and len(imu["rpy"]) >= 3:
            yaw = imu["rpy"][2]
        if (isinstance(position, (list, tuple)) and len(position) >= 3 and yaw is not None
                and self.state_at and now - self.state_at <= self.POSE_MAX_AGE):
            values = [position[0], position[1], position[2], yaw]
            if all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
                pose = tuple(float(v) for v in values)
                self.check_pose_jump(pose, self.state_at)
                return pose, now - self.state_at, "sportmodestate"
        ages = [now - t for t in (self.odom_at, self.state_at) if t]
        return None, (min(ages) if ages else math.inf), None

    def check_pose_jump(self, pose, stamp):
        """Warn when the estimator's position teleports (e.g. reset on stand/mode change)."""
        last = getattr(self, "_jump_ref", None)
        self._jump_ref = (pose, stamp)
        if last is None or stamp == last[1]:
            return
        dt = stamp - last[1]
        dist = math.hypot(pose[0] - last[0][0], pose[1] - last[0][1])
        if dt < 0.3 and dist > 0.3:
            self.pose_jumps = getattr(self, "pose_jumps", 0) + 1
            print("[slam] 位姿跳变：{:.2f} s 内移动 {:.2f} m（({:.2f}, {:.2f}) → ({:.2f}, {:.2f})），"
                  "可能是状态估计器重置".format(dt, dist, last[0][0], last[0][1], pose[0], pose[1]), flush=True)

    def pose2d(self):
        return self.pose_source()[0]

    def slam_pose(self):
        pose = self.map_pose()
        return None if pose is None else [round(pose[0], 3), round(pose[1], 3), round(pose[3], 4)]

    def map_pose(self):
        """Robot pose in the drift-corrected SLAM map frame, (x, y, z, yaw)."""
        pose = self.pose2d()
        return None if pose is None else self.map.to_map(pose)

    def map_frame(self, points):
        """Feed one voxel frame to the map, off the event loop when possible."""
        pose, age, source = self.pose_source()
        if pose is None:
            self.set_map_status("没有实时位姿（robot_pose 和 sportmodestate 都没有新数据），暂停建图"
                                if math.isfinite(age) else
                                "没有收到位姿（robot_pose / sportmodestate），无法建图")
            return
        self.map.pose_source = source
        if self.map_busy:
            return
        self.map_busy = True

        def work():
            try:
                self.map.process(points, pose)
            except Exception as exc:   # surface instead of silently losing it
                import traceback
                traceback.print_exc()
                self.map.status = "建图出错：{}: {}".format(type(exc).__name__, exc)
            finally:
                self.map_busy = False
                self.set_map_status(self.map.status)

        try:
            asyncio.get_running_loop().run_in_executor(None, work)
        except RuntimeError:        # no loop (offline checks): run inline
            work()

    def set_map_status(self, text):
        self.map.status = text
        if text != getattr(self, "_logged_map_status", None):
            self._logged_map_status = text
            print("[slam]", text, flush=True)

    def start_nav(self, x, y, yaw):
        self.nav_log("收到目标 x={:.2f} y={:.2f} 朝向 {:.0f}°".format(x, y, math.degrees(yaw)))
        if self.pose2d() is None:
            raise ValueError("没有实时位姿，无法导航")
        if self.map.known == 0:
            raise ValueError("地图为空，请先遥控机器狗走一圈建图")
        self.nav.set_goal(x, y, yaw, time.monotonic())
        self.schedule_plan(force=True)

    def schedule_plan(self, force=False):
        if self.plan_task is not None and not self.plan_task.done():
            return
        self.plan_task = asyncio.ensure_future(self._plan(self.nav.goal_id))

    async def _plan(self, goal_id):
        pose, goal = self.map_pose(), self.nav.goal
        started = time.monotonic()
        try:
            if pose is None or goal is None:
                raise PlanError("没有实时位姿，无法规划")
            grid, ox, oy, res = self.map.snapshot(pose[:2])
            path = await asyncio.to_thread(plan_path, grid, ox, oy, res, pose[:2], goal[:2])
        except PlanError as exc:
            self.nav_log("规划失败：{}".format(exc))
            self.nav.plan_done(goal_id, None, str(exc), time.monotonic())
        except Exception as exc:              # never leave the navigator stuck in "planning"
            import traceback
            traceback.print_exc()
            self.nav_log("规划出错：{}: {}".format(type(exc).__name__, exc))
            self.nav.plan_done(goal_id, None, "规划出错：{}".format(type(exc).__name__), time.monotonic())
        else:
            length = float(np.hypot(*np.diff(path, axis=0).T).sum()) if len(path) > 1 else 0.0
            self.nav_log("规划完成：{} 个路径点，长 {:.2f} m，用时 {:.0f} ms".format(
                len(path), length, (time.monotonic() - started) * 1000))
            self.nav.plan_done(goal_id, path, None, time.monotonic())

    def nav_log(self, text):
        print("[nav]", text, flush=True)

    def nav_tick(self):
        now = time.monotonic()
        if self.mode not in ("normal", "mcf"):
            self.nav.cancel("当前运动模式不支持导航")
            return
        pose, pose_age, _ = self.pose_source()
        pose = None if pose is None else self.map.to_map(pose)
        want = self.nav.tick(now, None if pose is None else (pose[0], pose[1], pose[3]),
                             self.map.scan, pose_age,
                             now - self.map.scan_at if self.map.scan_at else math.inf)
        if want:
            self.schedule_plan()
        key = (self.nav.state, self.nav.message, (self.nav.debug or {}).get("phase"),
               (self.nav.debug or {}).get("blocked_by"))
        if key != getattr(self, "_nav_log_key", None):
            self._nav_log_key = key
            self.nav_log("状态 {} {} | {}".format(self.nav.state, self.nav.message, self.nav.debug))

    async def rpc(self, topic, options):
        if not self.ready():
            raise ValueError("请先连接机器人")
        response = await asyncio.wait_for(
            self.conn.datachannel.pub_sub.publish_request_new(topic, options), 4)
        code = response.get("data", {}).get("header", {}).get("status", {}).get("code")
        if code != 0:
            raise ValueError("机器人拒绝或未确认操作（返回码 {}）".format(code))
        return response

    def on_state(self, message):
        data = message.get("data", {})
        # Sport mode state drives the telemetry card.  Keep it separate from
        # lowstate: the two topics arrive at different rates and lowstate does
        # not contain the same velocity/position fields.
        if isinstance(data, dict) and data:
            self.state = data
        self.state_at = time.monotonic()

    def on_lowstate(self, message):
        data = message.get("data", {})
        motors = data.get("motor_state", []) if isinstance(data, dict) else []
        if isinstance(motors, list):
            self.motors = [{"q": float(m.get("q", 0)), "dq": float(m.get("dq", 0)),
                            "tau": float(m.get("tau_est", m.get("tau", 0))),
                            "temp": m.get("temperature", m.get("temperature_ntc", None))}
                           for m in motors[:12] if isinstance(m, dict)]

    def on_odom(self, message):
        pose = message.get("data", {}).get("pose", {}).get("pose", {})
        position, orientation = pose.get("position", {}), pose.get("orientation", {})
        values = [position.get(k) for k in ("x", "y", "z")]
        values += [orientation.get(k) for k in ("x", "y", "z", "w")]
        if all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
            self.odom = {"position": values[:3], "quaternion": values[3:]}
            self.odom_at = time.monotonic()

    def on_points(self, message):
        if message.get("topic") != self.lidar_topic:
            return
        now = time.monotonic()
        if now - self.points_at < 0.16:
            return
        outer = message.get("data", {})
        data = outer.get("data", {})
        self.lidar_meta = {key: outer.get(key) for key in ("origin", "resolution", "frame_id", "width", "src_size") if key in outer}
        if isinstance(data, dict) and "decode_error" in data:
            self.lidar_error = "压缩点云解码失败：" + str(data["decode_error"])
            return
        self.voxel = ({"data": data["compressed"], **self.lidar_meta}
                      if isinstance(data, dict) and data.get("compressed") else None)
        raw_points = data.get("points") if isinstance(data, dict) else None
        has_points = raw_points is not None and np.asarray(raw_points).size > 0
        if not has_points and isinstance(data, dict) and data.get("positions") is not None:
            raw_points = data.get("positions")
            has_points = np.asarray(raw_points).size > 0
        if not has_points and isinstance(data, (bytes, bytearray)):
            self.lidar_error = "该点云话题返回了未适配的二进制格式"
            return
        if not has_points and self.voxel is None:
            self.lidar_error = "该点云话题暂未返回可显示的点坐标"
            return
        points = np.asarray(raw_points if has_points else [], dtype=np.float32).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        # 体素格点与地图栅格的半格错位由 OccupancyMap 在第一帧自动对齐
        self.map_frame(points)
        if len(points) > 16000:
            points = points[::math.ceil(len(points) / 16000)]
        self.points = np.round(points, 3).tolist()
        self.lidar_error = ""
        self.points_at = now
        self.point_version += 1

    async def video(self, track):
        while True:
            frame = await track.recv()
            now = time.monotonic()
            if now - self.video_at < 0.10:
                continue
            image = frame.to_image()
            image.thumbnail((960, 540))
            self.jpeg = await asyncio.to_thread(self.encode, image)
            self.video_at = now
            self.video_version += 1

    @staticmethod
    def encode(image):
        import io
        output = io.BytesIO()
        image.save(output, format="JPEG", quality=72)
        return base64.b64encode(output.getvalue()).decode()

    async def connect(self):
        async with self.lock:
            if self.connected:
                return
            ip = os.environ.get("UNITREE_ROBOT_IP", "").strip()
            key = os.environ.get("UNITREE_AES_KEY", "").strip()
            if not re.fullmatch(r"[a-fA-F0-9]{32}", key):
                raise ValueError("启动服务前请设置 32 位 UNITREE_AES_KEY")
            if ip:
                try:
                    ipaddress.IPv4Address(ip)
                except ipaddress.AddressValueError:
                    raise ValueError("UNITREE_ROBOT_IP 不是有效的 IPv4 地址")
                self.ip_source = "环境变量"
            else:
                ip = await self.discover()
                self.ip_source = "自动发现"
            self.robot_ip = ip
            # 每次连接（通常即每次开机）从空白地图开始，里程计原点也随之重置
            self.nav.reset()
            self.map.reset()
            self.error = ""
            self.conn = UnitreeWebRTCConnection(WebRTCConnectionMethod.LocalSTA,
                        ip=ip, aes_128_key=key, device_type="Go2")
            try:
                connect_task = asyncio.create_task(self.conn.connect())
                try:
                    while not hasattr(self.conn, "video") and not connect_task.done():
                        await asyncio.sleep(0)
                    if hasattr(self.conn, "video"):
                        self.conn.video.add_track_callback(self.video)
                    await asyncio.wait_for(connect_task, 35)
                finally:
                    if not connect_task.done():
                        connect_task.cancel()
                        with contextlib.suppress(asyncio.CancelledError):
                            await connect_task
                self.connected = True
                self.conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LF_SPORT_MOD_STATE"], self.on_state)
                self.conn.datachannel.pub_sub.subscribe(RTC_TOPIC["LOW_STATE"], self.on_lowstate)
                self.conn.datachannel.pub_sub.subscribe(RTC_TOPIC["ROBOTODOM"], self.on_odom)
                self.conn.datachannel.set_decoder("native")
                self.conn.datachannel.decoder = VisualizationDecoder(self.conn.datachannel.decoder)
                await asyncio.wait_for(self.conn.datachannel.disableTrafficSaving(True), 4)
                self.conn.datachannel.pub_sub.publish_without_callback(RTC_TOPIC["ULIDAR_SWITCH"], "on")
                self.conn.datachannel.pub_sub.subscribe(self.lidar_topic, self.on_points)
                self.conn.video.switchVideoChannel(True)
                try:
                    response = await self.rpc(RTC_TOPIC["MOTION_SWITCHER"], {"api_id": 1001})
                    self.mode = json.loads(response["data"]["data"]).get("name", "未知")
                except (ValueError, KeyError, asyncio.TimeoutError):
                    self.mode = "未知"
                try:
                    await self.query_avoid()
                except (ValueError, KeyError, TypeError, asyncio.TimeoutError):
                    self.avoid = None
            except BaseException:
                await self.close()
                raise

    async def discover(self):
        """局域网组播发现 Go2（与手机 App 相同的 231.1.1.1 查询），返回 IP。

        设置了 UNITREE_ROBOT_SN 时只接受该序列号；否则要求恰好发现一台。
        discover_ip_sn 是阻塞 socket 调用，放到线程里避免卡住事件循环。
        """
        sn = os.environ.get("UNITREE_ROBOT_SN", "").strip() or None
        try:
            found = await asyncio.to_thread(discover_ip_sn, 3, "Go2", sn)
        except OSError as exc:
            raise ValueError("自动发现失败（{}），可改用 UNITREE_ROBOT_IP 指定地址".format(
                exc.strerror or type(exc).__name__))
        candidates = {}
        for serial, address in found.items():
            with contextlib.suppress(ValueError):
                candidates[serial] = str(ipaddress.IPv4Address(address))
        if sn:
            if sn not in candidates:
                raise ValueError("局域网中未发现指定序列号的 Go2（共发现 {} 台），"
                                 "请确认机器狗已开机并与本机在同一网络".format(len(candidates)))
            return candidates[sn]
        if not candidates:
            raise ValueError("局域网中未发现 Go2。固件 1.1.15 及以上只回应带序列号的查询，"
                             "请设置 UNITREE_ROBOT_SN 后重启服务；或设置 UNITREE_ROBOT_IP")
        if len(candidates) > 1:
            masked = "、".join("…" + serial[-4:] for serial in sorted(candidates))
            raise ValueError("发现多台 Go2（序列号尾号 {}），请设置 UNITREE_ROBOT_SN 指定".format(masked))
        return next(iter(candidates.values()))

    async def close(self):
        self.disarm()
        self.connected = False
        if self.conn:
            await self.conn.disconnect()
        self.conn = None
        self.avoid = None
        self.nav.reset()
        self.map.reset()
        self.state = {}
        self.points = []
        self.voxel = None
        self.odom = None
        self.odom_at = 0
        self.motors = []
        self.point_version += 1
        self.lidar_meta = {}
        self.lidar_error = ""
        self.jpeg = None
        self.state_at = self.points_at = self.video_at = 0
        self.mode = "未知"

    async def watchdog(self):
        last_move = 0.0
        while True:
            # 模拟摇杆按 unitree_ui 的 20 Hz 发送；直接速度指令保持 10 Hz。
            await asyncio.sleep(0.05)
            if self.connected and not self.ready():
                self.armed = False
                self.connected = False
                self.error = "机器人连接中断，请断开后重连；无法保证停止指令已送达"
            if not self.armed or time.monotonic() > self.deadline:
                # 页面心跳（前端每 100 ms 的 move）丢失也会停止导航
                self.nav.cancel("控制已锁定或页面心跳丢失，导航已停止")
                if self.moving:
                    self.stop()
                continue
            if self.nav.active or self.nav_moving:
                # 导航直接下发速度指令（不经避障服务），只靠实时规划和自身停车检查避障
                if time.monotonic() - last_move >= 0.095:
                    self.nav_tick()
                    x, y, z = self.nav.command
                    if any(self.nav.command):
                        self.send("Move", {"x": x, "y": y, "z": z})
                        self.nav_moving = self.moving = True
                    elif self.nav_moving:
                        self.send("Move", {"x": 0, "y": 0, "z": 0})
                        self.send("StopMove")
                        self.nav_moving = self.moving = False
                    last_move = time.monotonic()
                continue
            x, y, z = self.command
            if any(self.command):
                if self.avoid:
                    self.joystick(x, y, z)
                elif time.monotonic() - last_move >= 0.095:
                    self.send("Move", {"x": x, "y": y, "z": z})
                    last_move = time.monotonic()
                self.moving = True
            elif self.moving:
                self.stop()

    def snapshot(self):
        now = time.monotonic()
        return {"connected": self.ready(), "armed": self.armed, "state": self.state,
                "mode": self.mode, "error": self.error, "avoid": self.avoid,
                "slam": {**self.map.info(), "pose": self.slam_pose()}, "nav": self.nav.snapshot(),
                "robot_ip": self.robot_ip if self.ready() else None, "ip_source": self.ip_source, "motors": self.motors,
                "lidar_topic": self.lidar_topic, "lidar_meta": self.lidar_meta,
                "lidar_error": self.lidar_error,
                "odom": self.odom if now - self.odom_at < 2 else None,
                "ages": {"state": now-self.state_at if self.state_at else None,
                         "video": now-self.video_at if self.video_at else None,
                         "lidar": now-self.points_at if self.points_at else None}}


PORT = 8765
LOCAL_HOSTS = ("127.0.0.1:{}".format(PORT), "localhost:{}".format(PORT))


def lan_addresses():
    """本机在局域网中的 IPv4 地址（私有网段，排除回环）。"""
    found = set()
    with contextlib.suppress(OSError):
        for info in pysocket.getaddrinfo(pysocket.gethostname(), None, pysocket.AF_INET):
            found.add(info[4][0])
    # UDP connect 不发包，只用于让系统选出默认路由所在网卡的地址。
    for target in (os.environ.get("UNITREE_ROBOT_IP", ""), "192.168.0.1", "10.0.0.1"):
        with contextlib.suppress(OSError, ValueError), \
                pysocket.socket(pysocket.AF_INET, pysocket.SOCK_DGRAM) as probe:
            ipaddress.IPv4Address(target)
            probe.connect((target, 9))
            found.add(probe.getsockname()[0])
    result = []
    for text in found:
        with contextlib.suppress(ValueError):
            address = ipaddress.IPv4Address(text)
            if (address.is_private or address.is_link_local) and not address.is_loopback:
                result.append(address)
    return [str(a) for a in sorted(result)]


def host_allowed(host, lan):
    if host in LOCAL_HOSTS:
        return True
    if not lan:
        return False
    name, _, port = host.rpartition(":")
    if port != str(PORT):
        return False
    try:
        address = ipaddress.IPv4Address(name)
    except ValueError:
        # 允许 Bonjour 主机名（如 MacBook.local），拒绝其他域名以防 DNS 重绑定。
        return name.endswith(".local")
    return address.is_private or address.is_link_local


@web.middleware
async def local_only(request, handler):
    lan = request.app.get("lan", False)
    if not host_allowed(request.host, lan):
        raise web.HTTPForbidden(text="仅允许本机或局域网访问" if lan else "仅允许本机访问")
    origin = request.headers.get("Origin")
    # 只接受同源页面（地址栏里的主机就是本服务），阻止其他网站跨站发控制指令。
    if origin and origin != "http://" + request.host:
        raise web.HTTPForbidden(text="不允许跨站请求")
    return await handler(request)


async def no_cache(request, response):
    # 本地开发页面：强制浏览器每次用 ETag 重新验证，避免 Safari 等按
    # Last-Modified 启发式缓存旧的 app.js / ES 模块，导致改动不生效。
    # 用 on_response_prepare 而非中间件，FileResponse 发送时会重建头部。
    if not isinstance(response, web.WebSocketResponse):
        response.headers["Cache-Control"] = "no-cache"


async def socket(request):
    bridge = request.app["bridge"]
    ws = web.WebSocketResponse(heartbeat=5, max_msg_size=4096)
    await ws.prepare(request)
    # A browser refresh can leave the previous socket around briefly. Let the
    # newest local page take ownership so a stale tab cannot block the UI.
    if bridge.owner is not None and not bridge.owner.closed:
        bridge.disarm()
        await bridge.owner.close(code=4000, message=b"replaced by newer local console")
    bridge.owner = ws

    async def stream():
        video_version = point_version = map_version = -1
        while not ws.closed:
            await ws.send_json({"type": "status", **bridge.snapshot()})
            if bridge.jpeg and bridge.video_version != video_version:
                video_version = bridge.video_version
                await ws.send_json({"type": "video", "jpeg": bridge.jpeg})
            if bridge.point_version != point_version:
                point_version = bridge.point_version
                # 2D 过滤后的障碍物扫描替代原始 3D 点云；3D 体素仍供跟随视角使用
                await ws.send_json({"type": "points", "topic": bridge.lidar_topic,
                                    "scan": bridge.map.scan_list, "voxel": bridge.voxel})
            if bridge.map.version != map_version:
                map_version = bridge.map.version
                await ws.send_json({"type": "map", "png": bridge.map.png,
                                    **(bridge.map.png_meta or {})})
            await asyncio.sleep(0.12)

    sender = asyncio.create_task(stream())
    try:
        async for message in ws:
            if message.type != WSMsgType.TEXT:
                continue
            try:
                data = json.loads(message.data)
                action = data.get("action")
                if action == "connect":
                    if bridge.conn and not bridge.ready():
                        await bridge.close()
                    await bridge.connect()
                elif action == "disconnect":
                    await bridge.close()
                elif action == "lidar_topic":
                    if not bridge.ready():
                        raise ValueError("请先连接机器人")
                    topics = {"compressed": RTC_TOPIC["ULIDAR_ARRAY"], "voxel": RTC_TOPIC["ULIDAR"]}
                    selected = data.get("source")
                    if selected not in topics:
                        raise ValueError("未知点云数据源")
                    current = "compressed" if bridge.lidar_topic == topics["compressed"] else "voxel"
                    if selected == current:
                        raise ValueError("已经在使用该点云数据源")
                    bridge.conn.datachannel.pub_sub.unsubscribe(bridge.lidar_topic)
                    bridge.conn.datachannel.pub_sub.subscriptions.pop(bridge.lidar_topic, None)
                    bridge.lidar_topic = topics[selected]
                    bridge.nav.cancel("点云数据源已切换，导航已取消")
                    bridge.map.reset()
                    bridge.points = []
                    bridge.voxel = None
                    bridge.point_version += 1
                    bridge.lidar_meta = {}
                    bridge.lidar_error = ""
                    bridge.points_at = 0
                    bridge.conn.datachannel.pub_sub.subscribe(bridge.lidar_topic, bridge.on_points)
                elif action == "arm":
                    if not bridge.ready():
                        raise ValueError("请先连接机器人")
                    bridge.armed = True
                elif action in ("disarm", "stop"):
                    bridge.disarm()
                elif action == "move":
                    if not bridge.armed or not bridge.ready() or bridge.mode not in ("normal", "mcf"):
                        raise ValueError("运动控制尚未解锁，或机器人当前模式不支持移动")
                    values = [float(data.get(k, 0)) for k in ("x", "y", "z")]
                    if not all(math.isfinite(v) for v in values):
                        raise ValueError("无效速度")
                    bridge.command = tuple(max(-limit, min(limit, v)) for v, limit in zip(values, DRIVE_LIMITS))
                    bridge.deadline = time.monotonic() + 0.35
                    if any(bridge.command) and bridge.nav.active:
                        bridge.nav.cancel("手动操作接管，导航已取消")
                elif action == "nav_goal":
                    if not bridge.armed or not bridge.ready() or bridge.mode not in ("normal", "mcf"):
                        bridge.nav_log("拒绝目标：控制{}，模式 {}".format("已解锁" if bridge.armed else "未解锁", bridge.mode))
                        raise ValueError("导航目标未发送：请先解锁控制，并处于普通或 MCF 运动模式（当前 {}）".format(bridge.mode))
                    values = [float(data.get(k)) for k in ("x", "y", "yaw")]
                    if not all(math.isfinite(v) for v in values):
                        raise ValueError("无效导航目标")
                    bridge.start_nav(*values)
                    bridge.deadline = time.monotonic() + 0.35
                elif action == "nav_cancel":
                    bridge.nav.cancel("已取消导航")
                elif action == "map_reset":
                    bridge.nav.cancel("地图已清空，导航已取消")
                    bridge.map.reset()
                elif action == "avoid":
                    if not bridge.ready():
                        raise ValueError("请先连接机器人")
                    enable = data.get("enable")
                    if not isinstance(enable, bool):
                        raise ValueError("无效避障开关")
                    await bridge.set_avoid(enable)
                    await ws.send_json({"type": "notice", "message":
                        "避障已开启：手动驾驶改用模拟摇杆，速度由避障服务过滤" if enable
                        else "避障已关闭：手动驾驶改用直接速度指令，不会避障"})
                    continue
                elif action == "mode":
                    if not bridge.ready():
                        raise ValueError("请先连接机器人")
                    bridge.disarm()
                    mode = data.get("mode")
                    if mode not in ("normal", "ai"):
                        raise ValueError("未知运动模式")
                    await bridge.rpc(RTC_TOPIC["MOTION_SWITCHER"],
                                     {"api_id": 1002, "parameter": {"name": mode}})
                    bridge.mode = mode
                elif action == "pose":
                    if not bridge.ready():
                        raise ValueError("请先连接机器人")
                    name = data.get("name")
                    if name not in ("StandUp", "StandDown", "BalanceStand"):
                        raise ValueError("不支持该姿态")
                    bridge.disarm()
                    await bridge.rpc(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD[name]})
                else:
                    raise ValueError("未知操作")
                if action != "move":
                    await ws.send_json({"type": "notice", "message": "操作已处理：" + action})
            except Exception as exc:
                # Never echo third-party exception messages that could contain credentials.
                text = str(exc) if isinstance(exc, ValueError) else "操作失败（{}），请检查网络或固件支持".format(type(exc).__name__)
                await ws.send_json({"type": "error", "message": text})
    finally:
        if bridge.owner is ws:
            bridge.disarm()
            bridge.owner = None
        sender.cancel()
        with contextlib.suppress(asyncio.CancelledError, ConnectionError):
            await sender
    return ws


async def lifecycle(app):
    task = asyncio.create_task(app["bridge"].watchdog())
    yield
    task.cancel()
    with contextlib.suppress(asyncio.CancelledError):
        await task
    await app["bridge"].close()


async def debug_nav(request):
    """Read-only navigation/mapping state for troubleshooting."""
    bridge = request.app["bridge"]
    pose, age, source = bridge.pose_source()
    return web.json_response({
        "armed": bridge.armed, "mode": bridge.mode, "connected": bridge.ready() if bridge.conn else False,
        "pose": pose, "pose_age": None if not math.isfinite(age) else round(age, 3), "pose_source": source,
        "nav": bridge.nav.snapshot(), "slam": bridge.map.info()})


async def debug_slam_session(request):
    """Recent integrated frames: t, pose (x, y, yaw) and 2D obstacle scans. Read-only."""
    import io
    session = list(request.app["bridge"].map.session)
    if not session:
        raise web.HTTPNotFound(text="还没有建图记录")
    times = np.array([s[0] for s in session], np.float64)
    poses = np.array([s[1] for s in session], np.float64)
    counts = np.array([len(s[2]) for s in session], np.int64)
    scans = np.concatenate([s[2] for s in session]) if counts.sum() else np.zeros((0, 2), np.float32)
    buffer = io.BytesIO()
    np.savez_compressed(buffer, t=times, poses=poses, counts=counts, scans=scans,
                        pose_jumps=np.int64(getattr(request.app["bridge"], "pose_jumps", 0)))
    return web.Response(body=buffer.getvalue(), content_type="application/octet-stream")


async def debug_slam_frame(request):
    """Latest raw voxel frame + pose as .npz, for offline analysis. Read-only."""
    import io
    frame = request.app["bridge"].map.last_frame
    if frame is None:
        raise web.HTTPNotFound(text="还没有可用的点云帧")
    points, pose = frame
    m = request.app["bridge"].map
    buffer = io.BytesIO()
    np.savez_compressed(buffer, points=points, pose=np.asarray(pose, np.float64),
                        floor=np.float64(m.floor if m.floor is not None else np.nan))
    return web.Response(body=buffer.getvalue(), content_type="application/octet-stream")


def create_app(lan=False):
    app = web.Application(middlewares=[local_only])
    app["lan"] = lan
    app["bridge"] = RobotBridge()
    app.cleanup_ctx.append(lifecycle)
    app.on_response_prepare.append(no_cache)
    app.router.add_get("/", lambda request: web.FileResponse(STATIC / "index.html"))
    app.router.add_get("/ws", socket)
    app.router.add_get("/debug/slam_frame", debug_slam_frame)
    app.router.add_get("/debug/nav", debug_nav)
    app.router.add_get("/debug/slam_session", debug_slam_session)
    app.router.add_static("/static/", STATIC)
    # Keep the same public asset URL used by unitree_ui/Vite.
    app.router.add_static("/models/", STATIC / "models")
    return app


def main():
    parser = argparse.ArgumentParser(prog="python -m go2_web", description="Go2 本机控制台")
    parser.add_argument("--lan", action="store_true",
                        help="对局域网开放（无鉴权，同一网络的设备都能控制机器人）")
    args = parser.parse_args()
    print("本机访问:   http://127.0.0.1:{}".format(PORT))
    if args.lan:
        addresses = lan_addresses()
        for address in addresses:
            print("局域网访问: http://{}:{}".format(address, PORT))
        if not addresses:
            print("局域网访问: 未找到局域网 IPv4 地址，请检查网络连接")
        print("警告：已对局域网开放且无鉴权，同一网络中的任何设备都能控制机器人。"
              "仅在可信网络中使用；新打开的页面会接管控制，旧页面自动断开。")
    else:
        print("局域网访问: 未开启（需要 iPad 等设备访问时，加 --lan 启动）")
    robot_ip = os.environ.get("UNITREE_ROBOT_IP", "").strip()
    print("机器人地址: " + (robot_ip + "（来自 UNITREE_ROBOT_IP）" if robot_ip
          else "点击「连接机器人」时在局域网自动发现" +
          ("（按 UNITREE_ROBOT_SN 定向查询）" if os.environ.get("UNITREE_ROBOT_SN", "").strip()
           else "（未设置 UNITREE_ROBOT_SN：固件 1.1.15 及以上不会回应）")))
    web.run_app(create_app(lan=args.lan), host="0.0.0.0" if args.lan else "127.0.0.1",
                port=PORT, print=None)


if __name__ == "__main__":
    main()
