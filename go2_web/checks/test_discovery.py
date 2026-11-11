"""Robot IP auto-discovery selection logic, with discover_ip_sn mocked."""
import os
import unittest
from unittest.mock import patch

from go2_web.__main__ import RobotBridge

TARGET = "go2_web.__main__.discover_ip_sn"


class DiscoveryTests(unittest.IsolatedAsyncioTestCase):
    def setUp(self):
        self.env = patch.dict(os.environ, {}, clear=False)
        self.env.start()
        os.environ.pop("UNITREE_ROBOT_SN", None)

    def tearDown(self):
        self.env.stop()

    async def test_single_robot_is_used(self):
        with patch(TARGET, return_value={"SN0001": "192.168.1.20"}):
            self.assertEqual(await RobotBridge().discover(), "192.168.1.20")

    async def test_none_found_is_error(self):
        with patch(TARGET, return_value={}):
            with self.assertRaisesRegex(ValueError, "未发现"):
                await RobotBridge().discover()

    async def test_multiple_without_sn_masks_serials(self):
        with patch(TARGET, return_value={"AAAA1111": "192.168.1.2", "BBBB2222": "192.168.1.3"}):
            with self.assertRaises(ValueError) as ctx:
                await RobotBridge().discover()
        self.assertIn("…1111", str(ctx.exception))
        self.assertNotIn("AAAA", str(ctx.exception))

    async def test_sn_selects_robot(self):
        os.environ["UNITREE_ROBOT_SN"] = "BBBB2222"
        with patch(TARGET, return_value={"AAAA1111": "192.168.1.2", "BBBB2222": "192.168.1.3"}) as scan:
            self.assertEqual(await RobotBridge().discover(), "192.168.1.3")
        self.assertEqual(scan.call_args.args[2], "BBBB2222")

    async def test_invalid_ip_reply_is_ignored(self):
        with patch(TARGET, return_value={"SN0001": "not-an-ip"}):
            with self.assertRaises(ValueError):
                await RobotBridge().discover()

    async def test_socket_error_is_reported(self):
        with patch(TARGET, side_effect=OSError(48, "Address already in use")):
            with self.assertRaisesRegex(ValueError, "自动发现失败"):
                await RobotBridge().discover()
