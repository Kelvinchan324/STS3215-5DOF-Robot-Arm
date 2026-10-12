"""Serve the bundled CAD kinematics webpage on the local machine only."""

from datetime import datetime, timezone
from functools import partial
import http.server
import json
import threading
from pathlib import Path

PROJECT_FOLDER = Path(__file__).resolve().parents[1]
KINEMATICS_PAGE = (
    PROJECT_FOLDER.parent
    / "robot-arm-kinematics-raspberry-pi-2026-10-09"
    / "webserial-react"
    / "dist"
)


def create_page_server(live_pose_reader=None, live_torque_releaser=None):
    if not KINEMATICS_PAGE.is_dir():
        raise FileNotFoundError(
            f"Kinematics webpage build not found: {KINEMATICS_PAGE}"
        )
    if live_pose_reader is None:
        handler = partial(
            http.server.SimpleHTTPRequestHandler,
            directory=str(KINEMATICS_PAGE),
        )
    else:
        reader_lock = threading.Lock()

        class LivePoseRequestHandler(http.server.SimpleHTTPRequestHandler):
            def do_GET(self):
                if self.path.split("?", 1)[0] != "/api/pose":
                    return super().do_GET()
                try:
                    with reader_lock:
                        payload = live_pose_reader()
                    payload["sampled_at"] = datetime.now(timezone.utc).isoformat()
                    payload["cad_zero_alignment_confirmed"] = False
                    body = json.dumps(payload).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                except Exception as error:
                    body = json.dumps({"error": str(error)}).encode("utf-8")
                    self.send_response(503)
                    self.send_header("Content-Type", "application/json")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)

            def do_POST(self):
                if (
                    self.path.split("?", 1)[0] != "/api/torque/release"
                    or live_torque_releaser is None
                ):
                    self.send_error(404)
                    return
                try:
                    with reader_lock:
                        live_torque_releaser()
                    body = json.dumps(
                        {"status": "released", "message": "Torque released on all configured servos."}
                    ).encode("utf-8")
                    self.send_response(200)
                except Exception as error:
                    body = json.dumps({"error": str(error)}).encode("utf-8")
                    self.send_response(503)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        handler = partial(
            LivePoseRequestHandler,
            directory=str(KINEMATICS_PAGE),
        )
    return http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
