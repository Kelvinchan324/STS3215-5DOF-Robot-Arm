#!/usr/bin/env python3
"""A small GUI to read positions and command named joint angles."""

import sys
import threading
import tkinter as tk
from tkinter import messagebox, ttk
import webbrowser
from pathlib import Path

PROJECT_FOLDER = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_FOLDER))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import arm_settings as settings
from kinematics_page import create_page_server
from arm_helpers import (
    angle_to_position,
    connect,
    disable_all_torque,
    move_all_sync,
    positions_to_angles,
    read_all,
)

class ArmWindow:
    def __init__(self, root):
        self.root = root
        self.root.title("STS3215 robot arm")
        self.port = None
        self.packet_handler = None
        self.webpage_server = None
        self.webpage_thread = None
        self.estopped = False
        self.values = {name: tk.StringVar(value="0") for name in settings.JOINT_NAMES}
        ttk.Label(root, text="Enter joint angles in degrees").grid(
            row=0, column=0, columnspan=3, padx=10, pady=10
        )
        for row, name in enumerate(settings.JOINT_NAMES, start=1):
            ttk.Label(root, text=name.replace("_", " ")).grid(
                row=row, column=0, padx=8, pady=4, sticky="w"
            )
            ttk.Entry(root, textvariable=self.values[name], width=12).grid(
                row=row, column=1, padx=8, pady=4
            )
        ttk.Button(root, text="Connect", command=self.connect).grid(
            row=6, column=0, padx=8, pady=10
        )
        ttk.Button(root, text="Read positions", command=self.read).grid(
            row=6, column=1, padx=8, pady=10
        )
        ttk.Button(
            root,
            text="Open CAD kinematics webpage",
            command=self.open_kinematics_page,
        ).grid(row=7, column=0, columnspan=2, padx=8, pady=5, sticky="ew")
        ttk.Button(root, text="Move joints", command=self.move).grid(
            row=8, column=0, columnspan=2, padx=8, pady=5, sticky="ew"
        )
        ttk.Button(
            root,
            text="EMERGENCY STOP — release torque",
            command=self.emergency_stop,
        ).grid(row=9, column=0, columnspan=2, padx=8, pady=5, sticky="ew")
        self.status = tk.StringVar(value="Not connected")
        ttk.Label(root, textvariable=self.status).grid(
            row=10, column=0, columnspan=3, padx=8, pady=8, sticky="w"
        )
        self.root.bind_all("<space>", self._space_emergency_stop)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def open_kinematics_page(self):
        try:
            if self.webpage_server is None:
                self.webpage_server = create_page_server()
                self.webpage_thread = threading.Thread(
                    target=self.webpage_server.serve_forever,
                    daemon=True,
                )
                self.webpage_thread.start()
            host, port = self.webpage_server.server_address
            url = f"http://{host}:{port}/"
            if webbrowser.open_new_tab(url):
                self.status.set(
                    "CAD kinematics webpage opened in your browser "
                    "(simulation by default)"
                )
            else:
                self.status.set(f"Open the CAD kinematics webpage at {url}")
        except Exception as error:
            self.show_error(error)

    def _space_emergency_stop(self, _event):
        self.emergency_stop()
        return "break"

    def connect(self):
        try:
            self.port, self.packet_handler = connect()
            self.estopped = False
            self.status.set(f"Connected to {settings.SERIAL_PORT}")
        except Exception as error:
            self.show_error(error)

    def read(self):
        if not self.packet_handler:
            self.show_error(RuntimeError("Press Connect first"))
            return
        if self.estopped:
            self.show_error(RuntimeError("Emergency stop is active; reconnect to resume"))
            return
        try:
            positions = read_all(self.packet_handler)
            angles = positions_to_angles(positions)
            for name, angle in angles.items():
                self.values[name].set(f"{angle:.1f}")
            self.status.set("Current joint angles loaded")
        except Exception as error:
            self.show_error(error)

    def move(self):
        if not self.packet_handler:
            self.show_error(RuntimeError("Press Connect first"))
            return
        if self.estopped:
            self.show_error(RuntimeError("Emergency stop is active; reconnect to resume"))
            return
        try:
            targets = {
                name: angle_to_position(name, float(self.values[name].get()))
                for name in settings.JOINT_NAMES
            }
            if not messagebox.askyesno("Confirm move", "Move all five joints now?"):
                return
            move_all_sync(self.packet_handler, targets)
            self.status.set("Move commands sent")
        except Exception as error:
            self.show_error(error)

    def emergency_stop(self):
        if not self.packet_handler:
            self.status.set("Emergency stop unavailable: connect to the controller first")
            return
        try:
            disable_all_torque(self.packet_handler)
            self.estopped = True
            self.status.set("EMERGENCY STOP: torque released. Reconnect to resume.")
        except Exception as error:
            self.show_error(error)

    def show_error(self, error):
        self.status.set(f"Error: {error}")
        messagebox.showerror("Robot arm", str(error), parent=self.root)

    def close(self):
        if self.webpage_server:
            self.webpage_server.shutdown()
            self.webpage_server.server_close()
            self.webpage_thread.join(timeout=1)
        if self.port:
            self.port.closePort()
        self.root.destroy()


def main():
    root = tk.Tk()
    ArmWindow(root)
    root.mainloop()


if __name__ == "__main__":
    main()
