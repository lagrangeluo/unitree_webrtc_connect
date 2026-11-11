"""Exercise the WebSocket control handler without connecting to a robot."""
import unittest
from unittest.mock import Mock

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from go2_web.__main__ import RobotBridge, socket


class DriveLimitsTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.bridge = RobotBridge()
        self.bridge.send = Mock()
        self.bridge.ready = Mock(return_value=True)
        self.bridge.mode = "mcf"
        app = web.Application()
        app["bridge"] = self.bridge
        app.router.add_get("/ws", socket)
        self.client = TestClient(TestServer(app))
        await self.client.start_server()
        self.ws = await self.client.ws_connect("/ws")

    async def asyncTearDown(self):
        await self.client.close()

    async def receive_type(self, kind):
        while True:
            message = await self.ws.receive_json(timeout=2)
            if message["type"] == kind:
                return message

    async def test_scaled_commands_and_backend_caps(self):
        for mode in ("normal", "mcf"):
            self.bridge.mode = mode
            for values, expected in (
                ((.30, .15, .50), (.30, .15, .50)),
                ((.60, .30, 1.0), (.60, .30, 1.0)),
                ((9, -9, 9), (.60, -.30, 1.0)),
            ):
                self.bridge.armed = True
                await self.ws.send_json(dict(action="move", **dict(zip("xyz", values))))
                # A following acknowledged action is a barrier for move processing.
                await self.ws.send_json({"action": "arm"})
                await self.receive_type("notice")
                self.assertEqual(self.bridge.command, expected)

    async def test_locked_control_rejects_motion(self):
        self.bridge.disarm()
        await self.ws.send_json({"action": "move", "x": .6})
        await self.receive_type("error")
        self.assertEqual(self.bridge.command, (0, 0, 0))
