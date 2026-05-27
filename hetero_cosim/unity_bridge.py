"""Unity UDP bridge — shared helper for all hetero experiments.

Each step the experiment script calls `send_state(...)` with the 4 agents'
poses; the helper formats them into ASCII messages and shoots them over UDP
to a Unity host (auto-resolved from environment + WSL networking mode).

Message format (matches run_hetero_three_uav_one_ugv.py):
    UAV,<idx>,x,y,z,roll,pitch,yaw
    UGV,0,x,y,z,roll,pitch,yaw

Disable / configure via environment variables:
    UNITY_UDP    "0" -> no socket created, send_state is a no-op
                 "1" -> enabled (default)
    UNITY_HOST   IP / hostname; if unset, auto-detect
    UNITY_PORT   default 5006
"""

import os
import socket
import subprocess

import numpy as np
import pybullet as p


def _resolve_unity_host():
    env_ip = os.environ.get("UNITY_HOST")
    if env_ip:
        return env_ip
    try:
        mode = subprocess.check_output(
            ["wslinfo", "--networking-mode"],
            stderr=subprocess.DEVNULL, timeout=2).decode().strip()
        if mode == "mirrored":
            return "127.0.0.1"
    except Exception:
        pass
    try:
        out = subprocess.check_output(
            ["ip", "route", "show", "default"], timeout=2).decode()
        for tok in out.split():
            if tok.count(".") == 3:
                return tok
    except Exception:
        pass
    return "127.0.0.1"


class UnityBridge:
    """Light wrapper around a UDP socket; sends one ASCII line per agent.

    Usage:
        bridge = UnityBridge()
        ...
        bridge.send_uav(idx, pos_xyz, rpy_xyz)
        bridge.send_ugv(0, ugv_pos, ugv_quat)   # quat -> rpy auto
        ...
        bridge.close()
    """

    def __init__(self):
        self.enabled = os.environ.get("UNITY_UDP", "1") == "1"
        self.host = _resolve_unity_host()
        self.port = int(os.environ.get("UNITY_PORT", "5006"))
        self.sock = None
        if self.enabled:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                print(f"[UnityBridge] UDP target: {self.host}:{self.port}")
            except Exception as e:
                print(f"[UnityBridge] failed to create socket: {e}; disabling")
                self.enabled = False

    def send_uav(self, idx, pos, rpy):
        if not self.enabled or self.sock is None:
            return
        msg = (f"UAV,{idx},"
               f"{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},"
               f"{rpy[0]:.4f},{rpy[1]:.4f},{rpy[2]:.4f}")
        try:
            self.sock.sendto(msg.encode(), (self.host, self.port))
        except Exception:
            pass

    def send_ugv(self, idx, pos, quat):
        """quat is a 4-vector (PyBullet convention)."""
        if not self.enabled or self.sock is None:
            return
        rpy = p.getEulerFromQuaternion(quat)
        msg = (f"UGV,{idx},"
               f"{pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f},"
               f"{rpy[0]:.4f},{rpy[1]:.4f},{rpy[2]:.4f}")
        try:
            self.sock.sendto(msg.encode(), (self.host, self.port))
        except Exception:
            pass

    def close(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
