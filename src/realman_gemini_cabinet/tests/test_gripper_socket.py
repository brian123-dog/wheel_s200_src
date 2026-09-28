import unittest
from unittest.mock import patch

from rg_cabinet_ros2.gripper_socket import (
    SocketJsonGripperClient,
    SocketJsonGripperConfig,
)


class RecordingGripperClient(SocketJsonGripperClient):
    def __init__(self, config):
        super().__init__(config)
        self.payloads = []

    def _send(self, payload):
        self.payloads.append(payload)
        return {"ret": 0}


class SocketJsonGripperTests(unittest.TestCase):
    def test_initialization_writes_tool_and_modbus_defaults(self):
        client = RecordingGripperClient(SocketJsonGripperConfig())
        with patch("rg_cabinet_ros2.gripper_socket.time.sleep") as sleep:
            client.initialize()

        self.assertEqual(
            [payload["command"] for payload in client.payloads],
            [
                "set_tool_voltage",
                "set_modbus_mode",
                "write_single_register",
                "write_single_register",
                "write_single_register",
                "write_single_register",
                "write_single_register",
            ],
        )
        self.assertEqual(client.payloads[0], {"command": "set_tool_voltage", "voltage_type": 3})
        self.assertEqual(
            client.payloads[1],
            {"command": "set_modbus_mode", "port": 1, "baudrate": 115200, "timeout": 2},
        )
        written = {
            payload["address"]: payload["data"]
            for payload in client.payloads
            if payload["command"] == "write_single_register"
        }
        self.assertEqual(written[257], 1)
        self.assertEqual(written[256], 1)
        self.assertEqual(written[260], 100)
        self.assertEqual(written[261], 50)
        self.assertEqual(written[262], 1000)
        self.assertEqual(
            [call.args[0] for call in sleep.call_args_list],
            [0.5, 0.5, 1.0],
        )

    def test_set_normalized_position_maps_close_to_high_low_bytes(self):
        client = RecordingGripperClient(
            SocketJsonGripperConfig(
                pos_open=3300,
                pos_close=4700,
                init_on_connect=False,
            )
        )
        with patch("rg_cabinet_ros2.gripper_socket.time.sleep") as sleep:
            debug = client.set_normalized_position(1.0)

        position_payload = client.payloads[0]
        self.assertEqual(position_payload["command"], "write_registers")
        self.assertEqual(position_payload["address"], 258)
        self.assertEqual(position_payload["num"], 2)
        self.assertEqual(position_payload["data"], [0, 0, 18, 92])
        self.assertEqual(client.payloads[1]["address"], 264)
        self.assertEqual(client.payloads[1]["data"], 1)
        self.assertEqual(debug["normalized_position"], 1.0)
        self.assertEqual(debug["raw_target"], 4700)
        self.assertEqual(debug["pos_open"], 3300)
        self.assertEqual(debug["pos_close"], 4700)
        self.assertEqual(debug["position_payload"], position_payload)
        self.assertEqual(debug["position_response"], {"ret": 0})
        self.assertEqual(debug["trigger_payload"], client.payloads[1])
        self.assertEqual(debug["trigger_response"], {"ret": 0})
        self.assertFalse(debug["initialized_before"])
        self.assertFalse(debug["initialized_after"])
        sleep.assert_called_once_with(0.02)

    def test_set_normalized_position_maps_open_to_high_low_bytes(self):
        client = RecordingGripperClient(
            SocketJsonGripperConfig(
                pos_open=3300,
                pos_close=4700,
                init_on_connect=False,
            )
        )
        with patch("rg_cabinet_ros2.gripper_socket.time.sleep"):
            debug = client.set_normalized_position(0.0)

        self.assertEqual(client.payloads[0]["data"], [0, 0, 12, 228])
        self.assertEqual(debug["raw_target"], 3300)

    def test_default_range_maps_open_and_close_to_raw_0_9000(self):
        client = RecordingGripperClient(SocketJsonGripperConfig(init_on_connect=False))
        with patch("rg_cabinet_ros2.gripper_socket.time.sleep"):
            open_debug = client.set_normalized_position(0.0)
            close_debug = client.set_normalized_position(1.0)

        self.assertEqual(open_debug["raw_target"], 0)
        self.assertEqual(open_debug["position_payload"]["data"], [0, 0, 0, 0])
        self.assertEqual(close_debug["raw_target"], 9000)
        self.assertEqual(close_debug["position_payload"]["data"], [0, 0, 35, 40])

    def test_send_appends_crlf_and_reuses_socket(self):
        class FakeSocket:
            def __init__(self):
                self.sent = []
                self.connected_to = None
                self.closed = False

            def settimeout(self, timeout):
                self.timeout = timeout

            def connect(self, address):
                self.connected_to = address

            def sendall(self, data):
                self.sent.append(data)

            def recv(self, size):
                self.recv_size = size
                return b'{"ret":0}'

            def close(self):
                self.closed = True

        fake_socket = FakeSocket()
        with patch(
            "rg_cabinet_ros2.gripper_socket.socket.socket",
            return_value=fake_socket,
        ):
            client = SocketJsonGripperClient(SocketJsonGripperConfig())
            client._send({"command": "noop"})
            client._send({"command": "again"})

        self.assertEqual(fake_socket.connected_to, ("192.168.1.18", 8080))
        self.assertEqual(fake_socket.recv_size, 1024)
        self.assertEqual(fake_socket.sent[0], b'{"command": "noop"}\r\n')
        self.assertEqual(fake_socket.sent[1], b'{"command": "again"}\r\n')

    def test_response_error_raises(self):
        client = SocketJsonGripperClient(SocketJsonGripperConfig())
        with self.assertRaisesRegex(RuntimeError, "failed"):
            client._raise_for_error(
                {"command": "write_single_register"},
                {"ret": 1},
            )

    def test_socket_exception_raises_runtime_error(self):
        class FailingSocket:
            def settimeout(self, timeout):
                pass

            def connect(self, address):
                pass

            def sendall(self, data):
                raise OSError("network down")

            def close(self):
                pass

        with patch(
            "rg_cabinet_ros2.gripper_socket.socket.socket",
            return_value=FailingSocket(),
        ):
            client = SocketJsonGripperClient(SocketJsonGripperConfig())
            with self.assertRaisesRegex(RuntimeError, "network down"):
                client._send({"command": "noop"})


if __name__ == "__main__":
    unittest.main()
