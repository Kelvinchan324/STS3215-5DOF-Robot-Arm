#!/usr/bin/env python3
"""Serve the local 3D arm viewer; optionally read current motor positions."""
import argparse
from functools import partial
import http.server
import json
from pathlib import Path
import sys
import threading

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live", action="store_true", help="connect to motors for read-only XYZ")
    parser.add_argument("--port", type=int, default=8080)
    args = parser.parse_args()
    config = json.loads((ROOT / "kinematics_config.json").read_text())
    motor_port = None
    server = None
    lock = threading.Lock()
    try:
        if args.live:
            from arm_helpers import connect, read_servo
            motor_port, packet = connect()

        class Handler(http.server.SimpleHTTPRequestHandler):
            def do_GET(self):
                path = self.path.split("?", 1)[0]
                if path not in ("/api/pose", "/kinematics_config.json"):
                    return super().do_GET()
                status = 200
                try:
                    if path == "/kinematics_config.json":
                        payload = config
                    elif not args.live:
                        status, payload = 503, {"error": "Start the server with --live to read motors."}
                    else:
                        with lock:
                            raw = {j["servo_id"]: read_servo(packet, j["servo_id"]) for j in config["joints"]}
                        payload = {"raw_by_id": raw}
                except Exception as error:
                    status, payload = 503, {"error": str(error)}
                body = json.dumps(payload).encode()
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        server = http.server.ThreadingHTTPServer(
            ("127.0.0.1", args.port), partial(Handler, directory=str(ROOT / "web/arm-viewer")))
        print(f"Open http://localhost:{server.server_port} on the Raspberry Pi.", flush=True)
        print("Read-only live motors enabled." if args.live else "Simulation mode. Use --live for motor readings.")
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if server is not None:
            server.server_close()
        if motor_port is not None:
            motor_port.closePort()


if __name__ == "__main__":
    main()
