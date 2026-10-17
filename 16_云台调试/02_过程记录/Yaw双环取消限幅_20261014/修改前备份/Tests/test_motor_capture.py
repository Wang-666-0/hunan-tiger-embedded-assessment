"""Offline tests: fake USART1 only. No test imports pyserial or opens a COM port."""
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Tools"))
from motor_capture import Capture, DM_FIELDS, FIRMWARE_ID, ProtocolError, read_command, validate_command
from vesc_decode import DecodeError, decode_vesc


class TickClock:
    def __init__(self):
        self.value = 0

    def __call__(self):
        self.value += 0.005
        return self.value


def vesc_lines(node, command, payload):
    return ([f"D|BEGIN|VESC|{node}|{command}"] +
            [f"D|VESC|{node}|{command}|{len(payload)}|{offset}|{payload[offset:offset + 32].hex()}"
             for offset in range(0, len(payload), 32)] + [f"D|DONE|VESC|{node}|{command}"])


def fw_reply(major=6, minor=6):
    return bytes([0, major, minor]) + b"N630\0" + bytes(range(12)) + b"\0\0\0"


def mc_temp(poles=14, gear=1.0):
    return bytes([91]) + b"".join(struct.pack(">f", value) for value in [-1, 1, -50000, 50000, .01, .95, -1000, 1000, -5, 10]) + bytes([poles]) + struct.pack(">ff", gear, .04)


def status_lines(enabled=0):
    return [f"D|STATUS|10|20|0|0|0|0|1|1500|1000|1000|{enabled}",
            "D|CAN|2|0|205|8|1FFF0064FFFF0000|12|2", "D|DONE|STATUS"]


class FakeSerial:
    def __init__(self, handler):
        self.handler = handler
        self.commands = []
        self.pending = bytearray()

    def write(self, data):
        command = data.decode("ascii").rstrip("\n")
        self.commands.append(command)
        lines = self.handler(command)
        if lines:
            self.pending.extend(("\r\n".join(lines) + "\r\n").encode("ascii"))
        return len(data)

    def read(self, size):
        # USB serial adapter can split a line anywhere, including CR/LF.
        chunk = self.pending[:min(17, size)]
        del self.pending[:len(chunk)]
        return bytes(chunk)


class CaptureTests(unittest.TestCase):
    def capture(self, handler):
        serial = FakeSerial(handler)
        return Capture(serial, clock=TickClock(), progress=lambda _: None), serial

    def test_whitelist_rejects_motion_and_raw_frames(self):
        for command in ("ENABLE", "WRITE DM 7 0", "READ VESC 70 5", "READ VESC 70 30", "READ VESC 70 48",
                        "READ VESC 70 16", "READ VESC 71 12", "READ DM 0", "READ VESC 240 0",
                        "STATUS\nENABLE", "RAW CAN2 200 00000000", " DIAG ON"):
            with self.subTest(command=command), self.assertRaises(ValueError):
                validate_command(command)
        with self.assertRaises(ValueError):
            read_command("VESC", True, 0)

    def test_full_capture_preserves_wire_and_fields(self):
        def handler(command):
            if command == "DIAG ON":
                return ["gimbal:1500,1500", f"D|HELLO|{FIRMWARE_ID}"]
            if command == "STATUS":
                return status_lines()
            if command.startswith("READ DM"):
                rid = int(command.split()[2])
                value = struct.pack("<f", 2.5) if DM_FIELDS[rid][1] == "float32" else struct.pack("<I", 42)
                payload = bytes([2, 0, 0x33, rid]) + value
                return [f"D|BEGIN|DM|{rid}", f"D|DM|{rid}|{payload.hex()}", f"D|DONE|DM|{rid}"]
            if command.startswith("READ VESC"):
                node, cmd = map(int, command.split()[2:])
                payload = fw_reply() if cmd == 0 else mc_temp() if cmd == 91 else bytes([cmd]) + bytes(53) if cmd == 4 else bytes([cmd]) + struct.pack(">I", 123456)
                return vesc_lines(node, cmd, payload)
            return []
        capture, serial = self.capture(handler)
        report = capture.run()
        self.assertTrue(report["complete"])
        self.assertEqual(len(report["dm"]), 14)
        self.assertEqual(report["dm"]["21"]["decoded"]["value"], 2.5)
        self.assertEqual(report["vesc"]["70"]["91"]["decoded"]["fields"]["configured_pole_pairs"], 7)
        self.assertEqual(report["vesc"]["71"]["0"]["decoded"]["uuid_hex"], "000102030405060708090A0B")
        self.assertFalse(report["vesc"]["70"]["14"]["decoded"]["decoded"])
        self.assertEqual(report["passive_can"][0]["c610_feedback"]["torque_feedback_raw"], -1)
        self.assertNotIn("temperature", report["passive_can"][0]["c610_feedback"])
        self.assertEqual(serial.commands[-1], "DIAG OFF")
        for command in serial.commands:
            validate_command(command)
        with tempfile.TemporaryDirectory() as directory:
            capture.save(Path(directory))
            saved = json.loads((Path(directory) / "参数采集报告.json").read_text(encoding="utf-8"))
            self.assertEqual(saved["vesc"]["70"]["91"]["raw_hex"], mc_temp().hex().upper())
            self.assertIn("TX READ VESC 71 17", (Path(directory) / "串口原始记录.txt").read_text(encoding="utf-8"))

    def test_unknown_firmware_identifier_stops_before_can_queries(self):
        capture, serial = self.capture(lambda cmd: ["D|HELLO|OLD_BUILD"] if cmd == "DIAG ON" else [])
        report = capture.run()
        self.assertFalse(report["complete"])
        self.assertEqual(serial.commands, ["DIAG ON", "DIAG OFF"])

    def test_output_enabled_stops_before_queries(self):
        capture, serial = self.capture(lambda cmd: [f"D|HELLO|{FIRMWARE_ID}"] if cmd == "DIAG ON" else status_lines(1) if cmd == "STATUS" else [])
        report = capture.run()
        self.assertFalse(report["complete"])
        self.assertEqual(serial.commands, ["DIAG ON", "STATUS", "DIAG OFF"])

    def test_host_timeout_does_not_send_next_request(self):
        capture, serial = self.capture(lambda cmd: [f"D|HELLO|{FIRMWARE_ID}"] if cmd == "DIAG ON" else status_lines() if cmd == "STATUS" else [])
        report = capture.run()
        self.assertFalse(report["complete"])
        self.assertEqual(serial.commands, ["DIAG ON", "STATUS", "READ DM 7", "DIAG OFF"])
        self.assertIn("host receive timeout", report["failures"][-1]["reason"])

    def test_firmware_timeout_continues_only_after_definite_terminal_error(self):
        capture, serial = self.capture(lambda cmd: [f"D|HELLO|{FIRMWARE_ID}"] if cmd == "DIAG ON" else status_lines() if cmd == "STATUS" else ["D|ERROR|TIMEOUT"] if cmd.startswith("READ") else [])
        report = capture.run()
        self.assertEqual(len(report["failures"]), 24)
        self.assertEqual(len(serial.commands), 28)
        self.assertFalse(report["complete"])

    def test_can_fault_aborts_session(self):
        capture, serial = self.capture(lambda cmd: [f"D|HELLO|{FIRMWARE_ID}"] if cmd == "DIAG ON" else status_lines() if cmd == "STATUS" else ["D|ERROR|CAN_FAULT"] if cmd.startswith("READ") else [])
        report = capture.run()
        self.assertEqual(serial.commands[-2:], ["READ DM 7", "DIAG OFF"])
        self.assertFalse(report["complete"])

    def test_wrong_offset_and_wrong_node_rejected(self):
        for lines in (["D|BEGIN|VESC|70|91", "D|VESC|70|91|50|1|00"],
                      ["D|BEGIN|VESC|70|91", "D|VESC|71|91|50|0|00"],
                      ["D|BEGIN|VESC|70|91", "D|VESC|70|91|50|0|00", "D|DONE|VESC|70|91"]):
            capture, _ = self.capture(lambda _: lines)
            with self.subTest(lines=lines), self.assertRaises(ProtocolError):
                capture.request("VESC", 70, 91)

    def test_dm_response_address_mismatch(self):
        capture, _ = self.capture(lambda _: ["D|BEGIN|DM|7", "D|DM|7|030033072A000000", "D|DONE|DM|7"])
        with self.assertRaises(ProtocolError):
            capture.request("DM", 7)


class DecodeTests(unittest.TestCase):
    def test_official_config_wire_offsets_and_signed_fields(self):
        # Offsets independently checked against the local official serialize functions.
        mc = bytearray(bytes([14]) + struct.pack(">I", 788332866) + bytes(479))
        mc[112] = 255  # hall_table[0] is signed int8: -1 means unused.
        mc[448] = 10
        mc[449:453] = struct.pack(">f", 2.0)
        fields = decode_vesc(bytes(mc), 14)["fields"]
        self.assertEqual(fields["hall_table[0]"], -1)
        self.assertEqual(fields["si_motor_poles"], 10)
        self.assertEqual(fields["si_gear_ratio"], 2)
        app = bytearray(bytes([17]) + struct.pack(">I", 2099347128) + bytes(274))
        app[5] = 70
        app[6:10] = struct.pack(">I", 1000)
        app[20] = 3
        fields = decode_vesc(bytes(app), 17)["fields"]
        self.assertEqual(fields["controller_id"], 70)
        self.assertEqual(fields["timeout_msec"], 1000)
        self.assertEqual(fields["can_baud_rate"], 3)

    def test_config_known_signature_and_unknown_signature(self):
        schema = json.loads((Path(__file__).resolve().parents[1] / "Tools" / "vesc_schema_6_06.json").read_text())
        for kind, command in (("mcconf", 14), ("appconf", 17)):
            profile = schema[kind]
            payload = bytes([command]) + struct.pack(">I", profile["signature"]) + bytes(profile["length"] - 4)
            decoded = decode_vesc(payload, command)
            self.assertTrue(decoded["decoded"])
            self.assertEqual(len(decoded["fields"]), len(profile["fields"]))
            self.assertFalse(decode_vesc(payload[:-1], command)["decoded"])
            unknown = payload[:1] + struct.pack(">I", 0xDEADBEEF) + payload[5:]
            self.assertFalse(decode_vesc(unknown, command)["decoded"])

    def test_firmware_name_is_variable_length(self):
        reply = bytes([0, 6, 5]) + b"CUSTOM_N630_STM32\0" + bytes(range(12))
        fields = decode_vesc(reply, 0)
        self.assertEqual((fields["major"], fields["minor"]), (6, 5))
        self.assertEqual(fields["hardware_name"], "CUSTOM_N630_STM32")
        with self.assertRaises(DecodeError):
            decode_vesc(bytes([0, 6, 6]) + b"unterminated", 0)

    def test_temp_no_defaults_and_rejects_wrong_layout(self):
        reply = mc_temp(poles=0, gear=0)
        fields = decode_vesc(reply, 91)["fields"]
        self.assertIsNone(fields["configured_pole_pairs"])
        self.assertEqual(fields["si_gear_ratio"], 0)
        with self.assertRaises(DecodeError):
            decode_vesc(reply[:-1], 91)
        bad = reply[:42] + struct.pack(">f", float("nan")) + reply[46:]
        with self.assertRaises(DecodeError):
            decode_vesc(bad, 91)

    def test_values_erpm_is_signed_and_not_mechanical_rpm(self):
        reply = bytearray(bytes([4]) + bytes(53))
        reply[23:27] = struct.pack(">i", -28000)
        reply[27:29] = struct.pack(">h", 240)
        fields = decode_vesc(bytes(reply), 4)["fields"]
        self.assertEqual(fields["electrical_rpm"], -28000)
        self.assertEqual(fields["input_voltage_v"], 24)
        self.assertNotIn("mechanical_rpm", fields)


if __name__ == "__main__":
    unittest.main()
