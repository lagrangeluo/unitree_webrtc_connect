"""Offline regression checks; no robot connection or movement."""
import base64
import json
import struct
import unittest

import lz4.block
import numpy as np

from go2_web.__main__ import RobotBridge, VisualizationDecoder
from unitree_webrtc_connect.webrtc_datachannel import WebRTCDataChannel
from unitree_webrtc_connect.lidar.lidar_decoder_native import LidarDecoder

COMPRESSED = 'rt/utlidar/voxel_map_compressed'
VOXEL = 'rt/utlidar/voxel_map'


def packet(topic, payload, modern):
    header = json.dumps({'topic': topic, 'data': {
        'origin': [1, 2, 3], 'resolution': .05, 'src_size': 2048,
        'frame_id': 'test_frame',
    }}).encode()
    if modern:
        return struct.pack('<HHII', 2, 0, len(header), 0) + header + payload
    return struct.pack('<HH', len(header), 0) + header + payload


class LidarPipelineTests(unittest.TestCase):
    def setUp(self):
        self.parser = object.__new__(WebRTCDataChannel)
        self.parser.decoder = LidarDecoder()
        raw = bytearray(2048)
        raw[0] = 0b11000000
        self.payload = lz4.block.compress(bytes(raw), store_size=False)

    def test_both_wire_formats_to_web_json(self):
        for modern in (False, True):
            with self.subTest(modern=modern):
                message = self.parser.deal_array_buffer(packet(COMPRESSED, self.payload, modern))
                bridge = RobotBridge()
                bridge.on_points(message)
                np.testing.assert_allclose(bridge.points, [[1, 2, 3], [1.05, 2, 3]])
                self.assertEqual(bridge.point_version, 1)
                self.assertEqual(bridge.lidar_error, '')
                json.dumps({'type': 'points', 'points': bridge.points})
                json.dumps(bridge.snapshot())

    def test_unknown_payload_not_passed_to_compressed_decoder(self):
        for modern in (False, True):
            message = self.parser.deal_array_buffer(packet(VOXEL, b'unknown bytes', modern))
            self.assertEqual(message['data']['data'], b'unknown bytes')

    def test_visualization_preserves_full_compressed_map(self):
        self.parser.decoder = VisualizationDecoder(LidarDecoder())
        message = self.parser.deal_array_buffer(packet(COMPRESSED, self.payload, True))
        bridge = RobotBridge()
        bridge.on_points(message)
        self.assertEqual(base64.b64decode(bridge.voxel['data']), self.payload)
        self.assertEqual(bridge.voxel['origin'], [1, 2, 3])
        self.assertEqual(bridge.voxel['resolution'], .05)
        json.dumps(bridge.voxel)

    def test_empty_map_replaces_previous_frame(self):
        self.parser.decoder = VisualizationDecoder(LidarDecoder())
        bridge = RobotBridge()
        bridge.on_points(self.parser.deal_array_buffer(packet(COMPRESSED, self.payload, True)))
        bridge.points_at = 0
        empty = lz4.block.compress(bytes(2048), store_size=False)
        bridge.on_points(self.parser.deal_array_buffer(packet(COMPRESSED, empty, True)))
        self.assertEqual(bridge.points, [])
        self.assertEqual(bridge.point_version, 2)
        self.assertEqual(bridge.lidar_error, '')
        self.assertEqual(base64.b64decode(bridge.voxel['data']), empty)

    def test_odom_validated_and_expires(self):
        bridge = RobotBridge()
        pose = {'position': {'x': 1, 'y': 2, 'z': .3},
                'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}}
        bridge.on_odom({'data': {'pose': {'pose': pose}}})
        self.assertEqual(bridge.snapshot()['odom'], {
            'position': [1, 2, .3], 'quaternion': [0, 0, 0, 1]})
        bridge.on_odom({'data': {'pose': {'pose': {'position': {'x': float('nan')}}}}})
        self.assertEqual(bridge.odom['position'], [1, 2, .3])
        bridge.odom_at -= 3
        self.assertIsNone(bridge.snapshot()['odom'])

    def test_old_topic_frames_ignored_after_switch(self):
        bridge = RobotBridge()
        bridge.lidar_topic = VOXEL
        message = self.parser.deal_array_buffer(packet(COMPRESSED, self.payload, True))
        bridge.on_points(message)
        self.assertEqual(bridge.points, [])
        self.assertEqual(bridge.point_version, 0)

    def test_decode_errors_are_delivered_to_ui(self):
        for modern in (False, True):
            with self.assertLogs(level='ERROR'):
                message = self.parser.deal_array_buffer(packet(COMPRESSED, b'broken', modern))
            bridge = RobotBridge()
            bridge.on_points(message)
            self.assertIn('解码失败', bridge.lidar_error)
            self.assertEqual(bridge.points, [])


if __name__ == '__main__':
    unittest.main()
