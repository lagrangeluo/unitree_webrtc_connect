"""Avoidance switch: joystick mapping and channel selection, without a robot."""
import asyncio
import json
import unittest
from unittest.mock import AsyncMock, Mock

from go2_web.__main__ import RobotBridge
from unitree_webrtc_connect import RTC_TOPIC


class AvoidSwitchTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.bridge = RobotBridge()
        self.bridge.ready = Mock(return_value=True)
        self.bridge.conn = Mock()
        self.publish = self.bridge.conn.datachannel.pub_sub.publish_without_callback
        self.bridge.send = Mock()

    def joystick_payloads(self):
        return [c.args[1] for c in self.publish.call_args_list
                if c.args[0] == RTC_TOPIC["WIRELESS_CONTROLLER"]]

    def test_joystick_mapping_matches_unitree_ui_convention(self):
        # forward full, left strafe half, left turn (CCW) full, then clamp
        self.bridge.joystick(0.6, 0.15, 1.0)
        self.bridge.joystick(9, -9, -9)
        first, clamped = self.joystick_payloads()
        self.assertEqual(first, {"lx": -0.5, "ly": 1.0, "rx": -1.0, "ry": 0.0, "keys": 0})
        self.assertEqual(clamped, {"lx": 1.0, "ly": 1.0, "rx": 1.0, "ry": 0.0, "keys": 0})

    async def run_watchdog(self, seconds=0.3):
        task = asyncio.create_task(self.bridge.watchdog())
        await asyncio.sleep(seconds)
        task.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await task

    async def test_avoid_on_uses_wireless_controller_only(self):
        self.bridge.connected = self.bridge.armed = True
        self.bridge.avoid = True
        self.bridge.command = (0.3, 0.0, 0.0)
        self.bridge.deadline = float("inf")
        await self.run_watchdog()
        self.assertGreaterEqual(len(self.joystick_payloads()), 4)
        self.assertNotIn("Move", [c.args[0] for c in self.bridge.send.call_args_list])

    async def test_avoid_off_uses_sport_move_only(self):
        self.bridge.connected = self.bridge.armed = True
        self.bridge.avoid = False
        self.bridge.command = (0.3, 0.0, 0.0)
        self.bridge.deadline = float("inf")
        await self.run_watchdog()
        self.assertEqual(self.joystick_payloads(), [])
        self.assertIn("Move", [c.args[0] for c in self.bridge.send.call_args_list])

    async def test_set_avoid_disarms_and_confirms(self):
        self.bridge.armed = True
        self.bridge.rpc = AsyncMock(side_effect=[
            {}, {"data": {"data": json.dumps({"enable": True})}}])
        self.assertTrue(await self.bridge.set_avoid(True))
        self.assertFalse(self.bridge.armed)
        self.assertTrue(self.bridge.avoid)

    async def test_set_avoid_mismatch_is_error(self):
        self.bridge.rpc = AsyncMock(side_effect=[
            {}, {"data": {"data": json.dumps({"enable": False})}}])
        with self.assertRaises(ValueError):
            await self.bridge.set_avoid(True)
        self.assertFalse(self.bridge.avoid)


if __name__ == "__main__":
    unittest.main()
