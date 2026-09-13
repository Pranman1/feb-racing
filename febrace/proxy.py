#!/usr/bin/env python3
"""Multi-car bridge proxy: one simulator, one devkit container per car.

    python3 proxy.py --port 4570 --devkits 4568 4569

The simulator connects here and sends one Bridge message with V1..VN fields. Each
devkit gets only its own car, renamed to V1, so every racing stack keeps believing it
drives roboracer_1. Commands coming back (V1 Throttle/Steering/Reset) are renamed to the
car's real id and merged into one reply for the simulator. Nothing else: no stewarding,
no redaction. Runs inside the devkit image (python-socketio 4 + gevent + websocket-client).
"""
import argparse
import re
import threading

import socketio
from gevent import pywsgi
from geventwebsocket.handler import WebSocketHandler

VEHICLE_KEY = re.compile(r"^V(\d+) (.*)$")
COMMANDS = ("Throttle", "Steering", "Reset")


class Proxy:
    def __init__(self, devkit_ports):
        self.ports = devkit_ports
        self.lock = threading.Lock()
        self.commands = {f"V{i + 1} {c}": ("False" if c == "Reset" else "0.0000") for i in range(len(devkit_ports)) for c in COMMANDS}
        self.server = socketio.Server(async_mode="gevent")
        self.server.on("connect", lambda sid, env: print("simulator connected", flush=True))
        self.server.on("Bridge", self.from_simulator)
        self.clients = [self.connect_devkit(i, port) for i, port in enumerate(devkit_ports)]

    def connect_devkit(self, index, port):
        client = socketio.Client(reconnection=True)
        client.on("Bridge", lambda data: self.from_devkit(index, data))
        client.connect(f"http://127.0.0.1:{port}", transports=["websocket"])
        print(f"devkit {index + 1} connected on port {port}", flush=True)
        return client

    def from_simulator(self, sid, data):
        for index, client in enumerate(self.clients):
            own = {}
            for key, value in data.items():
                m = VEHICLE_KEY.match(key)
                if m is None:
                    own[key] = value
                elif int(m.group(1)) == index + 1:
                    own["V1 " + m.group(2)] = value
            if client.eio.state == "connected":
                client.emit("Bridge", own)
        with self.lock:
            reply = dict(self.commands)
        self.server.emit("Bridge", reply, to=sid)

    def from_devkit(self, index, data):
        with self.lock:
            for key, value in (data or {}).items():
                m = VEHICLE_KEY.match(key)
                if m is not None and m.group(2) in COMMANDS:
                    self.commands[f"V{index + 1} {m.group(2)}"] = value

    def serve(self, port):
        app = socketio.WSGIApp(self.server)
        print(f"proxy listening on {port} for {len(self.ports)} cars", flush=True)
        pywsgi.WSGIServer(("", port), app, handler_class=WebSocketHandler).serve_forever()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=4570, help="port the simulator connects to")
    ap.add_argument("--devkits", type=int, nargs="+", required=True, help="devkit bridge ports, car 1 first")
    args = ap.parse_args()
    Proxy(args.devkits).serve(args.port)


if __name__ == "__main__":
    main()
