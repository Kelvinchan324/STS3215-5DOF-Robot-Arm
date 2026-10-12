#!/usr/bin/env python3
"""Print current CAD-derived XYZ without changing torque or motor targets."""

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from cad_forward_kinematics import position_from_raw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "kinematics_config.json")
    parser.add_argument("--tcp-home", type=float, nargs=3, metavar=("X", "Y", "Z"),
                        help="gripper centre in the absolute CAD home frame, mm")
    parser.add_argument("--frame", choices=("base", "cad"), default="base")
    parser.add_argument("--raw", type=int, nargs=5, help="offline J1..J5 counts")
    parser.add_argument("--watch", type=float, metavar="SECONDS", help="repeat read interval")
    args = parser.parse_args()
    if args.watch is not None and (not math_is_positive_finite(args.watch) or args.raw):
        parser.error("--watch requires a positive finite interval and hardware readings")
    port = None
    try:
        config = json.loads(args.config.read_text())
        if args.raw is None:
            from arm_helpers import connect, read_servo
            port, handler = connect()
        while True:
            raw = (dict(zip((j["servo_id"] for j in config["joints"]), args.raw))
                   if args.raw is not None else
                   {j["servo_id"]: read_servo(handler, j["servo_id"]) for j in config["joints"]})
            result = position_from_raw(raw, config, args.tcp_home, args.frame)
            print(json.dumps(result), flush=True)
            if args.watch is None:
                break
            time.sleep(args.watch)
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        print(f"Could not calculate XYZ: {error}", file=sys.stderr)
        return 1
    finally:
        if port is not None:
            port.closePort()
    return 0


def math_is_positive_finite(value):
    import math
    return math.isfinite(value) and value > 0


if __name__ == "__main__":
    sys.exit(main())
