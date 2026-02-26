# -*- coding: utf-8 -*-
"""
Client ZMQ pour contrôle distant de l'Alicat.
Usage :
 python alicat_zmq_client.py --host 192.168.1.10 --port 5560
"""

import zmq
import argparse


class AlicatZmqClient:
    def __init__(self, host='localhost', port=5560, timeout=2000):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, timeout)
        self.socket.setsockopt(zmq.SNDTIMEO, timeout)
        self.socket.connect(f"tcp://{host}:{port}")

    def _send(self, msg: dict) -> dict:
        self.socket.send_json(msg)
        return self.socket.recv_json()

    def ping(self):
        return self._send({"cmd": "ping"})

    def get_pressure(self) -> float:
        r = self._send({"cmd": "get_pressure"})
        return r.get("pressure")

    def get_setpoint(self) -> float:
        r = self._send({"cmd": "get_setpoint"})
        return r.get("setpoint")

    def set_setpoint(self, value: float) -> dict:
        return self._send({"cmd": "set_setpoint", "value": value})

    def close(self):
        self.socket.close()
        self.context.term()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=5560)
    args = parser.parse_args()

    client = AlicatZmqClient(host=args.host, port=args.port)

    print("Ping:", client.ping())
    print("Pression actuelle:", client.get_pressure(), "Bar")
    print("Setpoint actuel:", client.get_setpoint(), "Bar")

    print("Envoi setpoint 10 Bar...")
    print(client.set_setpoint(10.0))