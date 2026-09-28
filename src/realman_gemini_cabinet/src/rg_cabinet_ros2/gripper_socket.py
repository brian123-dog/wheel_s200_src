from __future__ import annotations

import json
import socket
import time
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class SocketJsonGripperConfig:
    ip: str = "192.168.1.18"
    port: int = 8080
    device_id: int = 1
    pos_open: int = 0
    pos_close: int = 9000
    timeout_s: float = 2.0
    modbus_port: int = 1
    baudrate: int = 115200
    modbus_timeout_s: int = 2
    voltage_type: int = 3
    speed: int = 100
    force: int = 50
    acceleration: int = 1000
    deacceleration: int = 1000
    init_on_connect: bool = True


class SocketJsonGripperClient:
    def __init__(self, config: SocketJsonGripperConfig) -> None:
        self.config = config
        self._sock: socket.socket | None = None
        self._initialized = False

    def close(self) -> None:
        if self._sock is None:
            return
        try:
            self._sock.close()
        finally:
            self._sock = None
            self._initialized = False

    def initialize(self) -> None:
        if self._initialized:
            return
        self._send({"command": "set_tool_voltage", "voltage_type": self.config.voltage_type})
        time.sleep(0.5)
        self._send(
            {
                "command": "set_modbus_mode",
                "port": self.config.modbus_port,
                "baudrate": self.config.baudrate,
                "timeout": self.config.modbus_timeout_s,
            }
        )
        time.sleep(0.5)
        self._write_register(257, 1)
        self._write_register(256, 1)
        time.sleep(1.0)
        self._write_register(260, self.config.speed)
        self._write_register(261, self.config.force)
        self._write_register(262, self.config.acceleration)
        self._initialized = True

    def set_normalized_position(self, position: float) -> dict[str, Any]:
        initialized_before = self._initialized
        if self.config.init_on_connect:
            self.initialize()
        target = self._normalized_to_raw(position)
        high = (target >> 8) & 0xFF
        low = target & 0xFF
        position_payload = {
            "command": "write_registers",
            "port": self.config.modbus_port,
            "device": self.config.device_id,
            "address": 258,
            "num": 2,
            "data": [0, 0, high, low],
        }
        position_response = self._send(position_payload)
        time.sleep(0.02)
        trigger_payload = self._write_register(264, 1)
        return {
            "normalized_position": min(1.0, max(0.0, float(position))),
            "raw_target": target,
            "pos_open": self.config.pos_open,
            "pos_close": self.config.pos_close,
            "position_payload": position_payload,
            "position_response": position_response,
            "trigger_payload": trigger_payload["payload"],
            "trigger_response": trigger_payload["response"],
            "initialized_before": initialized_before,
            "initialized_after": self._initialized,
        }

    def _normalized_to_raw(self, position: float) -> int:
        closed = min(1.0, max(0.0, float(position)))
        span = self.config.pos_close - self.config.pos_open
        return int(round(self.config.pos_open + closed * span))

    def _write_register(self, address: int, data: int) -> dict[str, Any]:
        payload = {
            "command": "write_single_register",
            "port": self.config.modbus_port,
            "device": self.config.device_id,
            "address": int(address),
            "data": int(data),
        }
        response = self._send(payload)
        return {"payload": payload, "response": response}

    def _send(self, payload: dict[str, Any]) -> dict[str, Any]:
        data = (json.dumps(payload) + "\r\n").encode("utf-8")
        try:
            sock = self._connect()
            sock.sendall(data)
            response = self._recv_response(sock)
        except Exception as exc:
            self.close()
            raise RuntimeError(
                f"Gripper socket command {payload.get('command')} failed: {exc}"
            ) from exc
        self._raise_for_error(payload, response)
        return response

    def _connect(self) -> socket.socket:
        if self._sock is None:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(self.config.timeout_s)
            self._sock.connect((self.config.ip, self.config.port))
        return self._sock

    def _recv_response(self, sock: socket.socket) -> dict[str, Any]:
        raw = sock.recv(1024)
        if not raw:
            raise RuntimeError("empty response")
        decoded = raw.decode("utf-8").strip()
        try:
            return json.loads(decoded)
        except json.JSONDecodeError:
            return {"raw": decoded}

    def _raise_for_error(
        self,
        payload: dict[str, Any],
        response: dict[str, Any],
    ) -> None:
        for key in ("ret", "code", "errcode"):
            if key in response and int(response[key]) != 0:
                raise RuntimeError(
                    f"Gripper command {payload.get('command')} failed: {response}"
                )
        if response.get("success") is False:
            raise RuntimeError(f"Gripper command {payload.get('command')} failed: {response}")
