import json
import re
import runpy
import threading
import unittest
from urllib.error import HTTPError
from unittest.mock import Mock, patch
from urllib.request import Request, urlopen

from arm_helpers import position_to_angle
import arm_settings as settings

gui_module = runpy.run_path(
    "scripts/08_robot_arm_gui.py", run_name="robot_arm_gui_test"
)
visualizer_module = runpy.run_path(
    "scripts/04_forward_kinematics.py", run_name="kinematics_visualizer_test"
)
page_module = runpy.run_path(
    "scripts/kinematics_page.py", run_name="kinematics_page_test"
)


class KinematicsPageTests(unittest.TestCase):
    def test_script_06_angle_equation_matches_live_raw_angle_reading(self):
        for joint in settings.JOINT_NAMES:
            raw = settings.ZERO_TICKS[joint] + 113 * settings.DIRECTIONS[joint]
            expected = (
                (raw - settings.ZERO_TICKS[joint])
                * 360
                / (settings.DIRECTIONS[joint] * settings.TICKS_PER_REVOLUTION)
            )
            self.assertAlmostEqual(position_to_angle(joint, raw), expected)

    def test_script_04_opens_visualizer_and_keeps_server_alive(self):
        class FakePacketHandler:
            @staticmethod
            def ReadPos(servo_id):
                return settings.ZERO_TICKS[
                    next(
                        name
                        for name, configured_id in settings.SERVO_IDS.items()
                        if configured_id == servo_id
                    )
                ], 0, 0

            @staticmethod
            def getTxRxResult(_result):
                return "communication error"

            @staticmethod
            def getRxPacketError(_error):
                return "servo error"

        server = Mock()
        server.server_address = ("127.0.0.1", 43210)
        server.serve_forever.side_effect = KeyboardInterrupt
        page_server = Mock(return_value=server)
        port = Mock()
        packet_handler = FakePacketHandler()
        with patch.dict(
            visualizer_module["main"].__globals__,
            {
                "connect": Mock(return_value=(port, packet_handler)),
                "create_page_server": page_server,
            },
        ):
            with patch.object(
                visualizer_module["webbrowser"],
                "open_new_tab",
                return_value=True,
            ) as open_page:
                visualizer_module["main"]()

        open_page.assert_called_once_with("http://127.0.0.1:43210/#live-pi")
        server_reader = page_server.call_args.args[0]
        pose_payload = server_reader()
        arm_settings_module = visualizer_module["main"].__globals__["settings"]
        self.assertEqual(
            pose_payload["positions_raw"],
            arm_settings_module.ZERO_TICKS,
        )
        self.assertEqual(pose_payload["zero_ticks"], arm_settings_module.ZERO_TICKS)
        self.assertEqual(pose_payload["directions"], arm_settings_module.DIRECTIONS)
        server.serve_forever.assert_called_once()
        server.server_close.assert_called_once()
        port.closePort.assert_called_once()

    def test_live_pose_endpoint_returns_raw_values_and_angle_calibration(self):
        expected_raw = {"base_yaw": 2048, "shoulder": 2050}
        calibration = {
            "positions_raw": expected_raw,
            "zero_ticks": {"base_yaw": 2048, "shoulder": 2048},
            "directions": {"base_yaw": 1, "shoulder": 1},
            "ticks_per_revolution": 4096,
        }
        server = page_module["create_page_server"](lambda: dict(calibration))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            host, port = server.server_address
            with urlopen(f"http://{host}:{port}/api/pose", timeout=2) as response:
                payload = json.loads(response.read().decode("utf-8"))
            self.assertEqual(payload["positions_raw"], expected_raw)
            self.assertEqual(payload["zero_ticks"], calibration["zero_ticks"])
            self.assertEqual(payload["directions"], calibration["directions"])
            self.assertEqual(
                payload["ticks_per_revolution"],
                calibration["ticks_per_revolution"],
            )
            self.assertIn("sampled_at", payload)
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

    def test_live_torque_release_endpoint_calls_all_servo_release_action(self):
        release = Mock()
        server = page_module["create_page_server"](lambda: {}, release)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            host, port = server.server_address
            request = Request(
                f"http://{host}:{port}/api/torque/release",
                data=b"",
                method="POST",
            )
            with urlopen(request, timeout=2) as response:
                payload = json.loads(response.read().decode("utf-8"))
            self.assertEqual(payload["status"], "released")
            release.assert_called_once_with()
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

    def test_live_torque_release_endpoint_surfaces_servo_errors(self):
        def fail_release():
            raise RuntimeError("Servo 3 torque release failed")

        server = page_module["create_page_server"](lambda: {}, fail_release)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            host, port = server.server_address
            request = Request(
                f"http://{host}:{port}/api/torque/release",
                data=b"",
                method="POST",
            )
            with self.assertRaises(HTTPError) as error:
                urlopen(request, timeout=2)
            self.assertEqual(error.exception.code, 503)
            payload = json.loads(error.exception.read().decode("utf-8"))
            self.assertIn("Servo 3 torque release failed", payload["error"])
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)

    def test_gui_serves_reference_page_only_on_loopback(self):
        window_class = gui_module["ArmWindow"]
        window = window_class.__new__(window_class)
        window.webpage_server = None
        window.webpage_thread = None
        window.port = None
        window.status = Mock()
        window.root = Mock()
        window.show_error = Mock()
        with patch.object(
            gui_module["webbrowser"], "open_new_tab", return_value=True
        ) as open_page:
            try:
                window.open_kinematics_page()
                open_page.assert_called_once()
                url = open_page.call_args.args[0]
                self.assertTrue(url.startswith("http://127.0.0.1:"))
                with urlopen(url, timeout=2) as response:
                    page = response.read().decode("utf-8")
                self.assertIn("Robot Arm Kinematics Lab", page)
                asset_path = re.search(
                    r'<script[^>]+src="([^"]+)"', page
                ).group(1)
                asset_url = (
                    f"http://127.0.0.1:"
                    f"{window.webpage_server.server_address[1]}{asset_path}"
                )
                with urlopen(asset_url, timeout=2) as response:
                    javascript = response.read().decode("utf-8")
                self.assertIn("J5 ref", javascript)
                self.assertNotIn("60 mm demonstration offset", javascript)
                self.assertEqual(window.show_error.call_count, 0)
            finally:
                window.close()


if __name__ == "__main__":
    unittest.main()
