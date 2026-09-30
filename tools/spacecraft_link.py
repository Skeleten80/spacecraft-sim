#!/usr/bin/env python3
"""Minimal client for the spacecraft-sim telemetry link.

The link is newline-delimited JSON over TCP (see TelemetryLink.swift):

    server -> client: {"type":"hello","protocolVersion":1,"scenario":...,"dt":...,"duration":...}
    server -> client: {"type":"frame", ...telemetry...}
    server -> client: {"type":"end","reason":"complete"}
    client -> server: {"type":"command","cmd":"killWheel","index":0}
    client -> server: {"type":"command","cmd":"loadScenario","scenario":"tumble"}

Usage:
    # Mini 1:  spacecraft-cli serve nominal --port 9001 --speed 0
    # Mini 3:  python3 tools/spacecraft_link.py --port 9001
"""

import argparse
import json
import socket


class SpacecraftLink:
    def __init__(self, host="127.0.0.1", port=9001):
        self.sock = socket.create_connection((host, port))
        # Text-mode wrapper gives us clean line iteration.
        self.lines = self.sock.makefile("r", encoding="utf-8")
        self.hello = self._read()
        assert self.hello["type"] == "hello", self.hello
        print(f"connected: scenario={self.hello['scenario']} "
              f"dt={self.hello['dt']} duration={self.hello['duration']}")

    def _read(self):
        line = self.lines.readline()
        if not line:
            raise ConnectionError("server closed the connection")
        return json.loads(line)

    def frames(self):
        """Yield telemetry frames until the server sends 'end'."""
        while True:
            msg = self._read()
            if msg["type"] == "frame":
                yield msg
            elif msg["type"] == "end":
                print(f"run ended: {msg['reason']}")
                return
            # 'hello' may re-appear after a loadScenario command.

    def kill_wheel(self, index):
        self.sock.sendall(
            (json.dumps({"type": "command", "cmd": "killWheel",
                         "index": index}) + "\n").encode())

    def load_scenario(self, name):
        self.sock.sendall(
            (json.dumps({"type": "command", "cmd": "loadScenario",
                         "scenario": name}) + "\n").encode())

    def set_speed(self, sim_seconds_per_real_second):
        self.sock.sendall(
            (json.dumps({"type": "command", "cmd": "setSpeed",
                         "value": sim_seconds_per_real_second}) + "\n").encode())

    def close(self):
        self.sock.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9001)
    ap.add_argument("--kill-wheel", type=int, default=None,
                    help="inject a wheel failure as soon as we connect")
    args = ap.parse_args()

    link = SpacecraftLink(args.host, args.port)
    if args.kill_wheel is not None:
        link.kill_wheel(args.kill_wheel)
        print(f"sent killWheel({args.kill_wheel})")

    n = 0
    last = None
    for last in link.frames():
        n += 1
    link.close()

    print(f"received {n} frames")
    if last:
        print(f"final pointing error : {last['pointErrDeg']:.3f} deg")
        print(f"final wheel loading  : {last['wheelSaturation'] * 100:.1f} %")
        print(f"thruster pulses      : {last['thrusterFirings']}")


if __name__ == "__main__":
    main()
