#!/usr/bin/env python3
"""Launch the interactive CAD-derived robot-arm kinematics visualization."""

import sys
import webbrowser
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from kinematics_page import create_page_server
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import arm_settings as settings
from arm_helpers import connect, disable_all_torque, read_all


def main():
    print(
        f"Connecting to {settings.SERIAL_PORT} at {settings.BAUD_RATE} baud "
        "to read the current arm pose (no movement commands will be sent)."
    )
    port, packet_handler = connect()
    server = None
    try:
        def read_live_pose():
            positions = read_all(packet_handler)
            return {
                "positions_raw": positions,
                "zero_ticks": settings.ZERO_TICKS,
                "directions": settings.DIRECTIONS,
                "ticks_per_revolution": settings.TICKS_PER_REVOLUTION,
            }

        read_live_pose()
        server = create_page_server(
            read_live_pose,
            lambda: disable_all_torque(packet_handler),
        )
        host, port_number = server.server_address
        url = f"http://{host}:{port_number}/#live-pi"
        try:
            browser_opened = webbrowser.open_new_tab(url)
        except webbrowser.Error as error:
            print(f"Could not open a browser automatically: {error}")
            browser_opened = False
        if browser_opened:
            print("Opened the live Robot Arm Kinematics Lab.")
        else:
            print("Open this address in a browser on the Raspberry Pi:")
        print(f"Visualization URL: {url}")
        print("Live pose is read every 0.5 seconds. Press Ctrl+C to stop.")
        print(
            "The visualization uses Pi ZERO_TICKS/DIRECTIONS. "
            "CAD-home alignment is not confirmed."
        )
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nLive kinematics visualization stopped.")
    finally:
        if server is not None:
            server.server_close()
        port.closePort()


if __name__ == "__main__":
    main()
