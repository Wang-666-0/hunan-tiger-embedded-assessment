"""Collect motor read replies through Comet USART1 (115200 8N1).

No motor enable, motion, configuration write, flash, zeroing or detection command
is exposed. Close VOFA before running: only one program can own the COM port.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import time

# Optional local dependency installed by the project setup; no network action here.
sys.path.insert(0, str(Path(__file__).resolve().parent / ".deps"))

from vesc_decode import DecodeError, decode_vesc, finite_float

FIRMWARE_ID = "GIMBAL_DIAG_1"
DM_FIELDS = {
    7: ("MST_ID", "uint32"), 8: ("ESC_ID", "uint32"),
    9: ("TIMEOUT", "uint32"), 10: ("CTRL_MODE", "uint32"),
    13: ("hw_ver", "uint32"), 14: ("sw_ver", "uint32"),
    15: ("SN", "uint32"), 16: ("NPP", "uint32"),
    20: ("Gr", "float32"), 21: ("PMAX", "float32"),
    22: ("VMAX", "float32"), 23: ("TMAX", "float32"),
    35: ("can_br", "uint32"), 36: ("sub_ver", "uint32"),
}
VESC_NODES = (70, 71)
VESC_COMMANDS = (0, 91, 14, 17, 4)
STATUS_FIELDS = ("can1_rx", "can2_rx", "can1_error", "can2_error", "can_dropped",
                 "can_fault_epoch", "remote_fresh", "ch1", "ch5", "ch6", "motor_enabled")


class ProtocolError(RuntimeError):
    pass


class DeviceError(RuntimeError):
    pass


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def read_command(kind: str, *values: int) -> str:
    """The only public request builder: whitelist first, then construct text."""
    if any(type(value) is not int for value in values):
        raise ValueError("request arguments must be integers")
    if kind == "DM" and len(values) == 1 and values[0] in DM_FIELDS:
        return f"READ DM {values[0]}"
    if kind == "VESC" and len(values) == 2 and values[0] in VESC_NODES and values[1] in VESC_COMMANDS:
        return f"READ VESC {values[0]} {values[1]}"
    raise ValueError("request is outside the fixed read whitelist")


def validate_command(command: str) -> None:
    allowed = {"DIAG ON", "DIAG OFF", "STATUS"}
    allowed.update(read_command("DM", rid) for rid in DM_FIELDS)
    allowed.update(read_command("VESC", node, cmd) for node in VESC_NODES for cmd in VESC_COMMANDS)
    if command not in allowed:
        raise ValueError("command is outside the fixed read whitelist")


def parse_hex(text: str, length: int | None = None) -> bytes:
    if len(text) % 2 or any(c not in "0123456789abcdefABCDEF" for c in text):
        raise ProtocolError("invalid hexadecimal payload")
    data = bytes.fromhex(text)
    if length is not None and len(data) != length:
        raise ProtocolError("payload length mismatch")
    return data


def decimal(text: str) -> int:
    if not text.isascii() or not text.isdecimal():
        raise ProtocolError("invalid unsigned decimal field")
    return int(text)


def decode_dm(data: bytes, rid: int) -> dict:
    if len(data) != 8 or int.from_bytes(data[:2], "little") != 2 or data[2:4] != bytes((0x33, rid)):
        raise ProtocolError("DM reply address, RID or length mismatch")
    name, typ = DM_FIELDS[rid]
    value = finite_float(data[4:], "<") if typ == "float32" else int.from_bytes(data[4:], "little")
    return {"name": name, "type": typ, "value": value}


class Capture:
    """Serial-independent state machine, also exercised with a fake transport."""
    def __init__(self, transport, timeout: float = 4.0, clock=time.monotonic, progress=print):
        if timeout < 3 or timeout > 30:
            raise ValueError("timeout must be between 3 and 30 seconds")
        self.transport = transport
        self.timeout = timeout
        self.clock = clock
        self.progress = progress
        self.pending = bytearray()
        self.log = []
        self.report = {
            "format": "comet_motor_capture_1", "started_at": now_iso(),
            "firmware": None, "serial": {"baudrate": 115200, "data_bits": 8, "parity": "N", "stop_bits": 1},
            "hardware": {"yaw": "GL60 II, CAN1 ID1", "pitch": "DM3507, CAN1 ID2",
                         "flywheels": "VESC N630, CAN2 ID70/71", "feeder": "C610, CAN2 ID5"},
            "status": [], "passive_can": [], "dm": {}, "vesc": {"70": {}, "71": {}},
            "failures": [], "complete": False,
            "limitations": ["GL60 II configuration cannot be read by a defined CAN parameter command; raw feedback only.",
                            "C610 feedback does not identify the motor model or feeder gear ratio/shots per revolution.",
                            "VESC configured poles/gear ratio require physical confirmation; defaults are never substituted."],
        }

    def send(self, command: str) -> None:
        validate_command(command)
        self.log.append(f"{now_iso()} TX {command}")
        raw = (command + "\n").encode("ascii")
        if self.transport.write(raw) != len(raw):
            raise ProtocolError("serial command was only partially written")

    def line(self, deadline: float) -> list[str]:
        while self.clock() < deadline:
            if b"\n" not in self.pending:
                chunk = self.transport.read(128)
                self.pending.extend(chunk)
                if len(self.pending) > 4096:
                    raise ProtocolError("serial line exceeds maximum length")
            if b"\n" not in self.pending:
                continue
            raw, _, remainder = self.pending.partition(b"\n")
            self.pending = bytearray(remainder)
            text = raw.rstrip(b"\r").decode("ascii", errors="replace")
            self.log.append(f"{now_iso()} RX {text}")
            if not text.startswith("D|"):
                # Old VOFA lines can remain in the USB adapter at mode entry.
                continue
            if "\ufffd" in text:
                raise ProtocolError("diagnostic reply is not ASCII")
            fields = text.split("|")
            if len(fields) >= 3 and fields[1] == "ERROR":
                raise DeviceError("|".join(fields[2:]))
            return fields
        raise TimeoutError("host receive timeout; no further CAN request will be sent")

    def enter(self) -> None:
        self.send("DIAG ON")
        fields = self.line(self.clock() + self.timeout)
        if fields != ["D", "HELLO", FIRMWARE_ID]:
            raise ProtocolError("firmware identifier mismatch; flash the parameter capture build first")
        self.report["firmware"] = FIRMWARE_ID

    def status(self) -> None:
        self.send("STATUS")
        deadline = self.clock() + self.timeout
        seen = False
        while True:
            fields = self.line(deadline)
            if fields == ["D", "DONE", "STATUS"]:
                if not seen:
                    raise ProtocolError("STATUS completion without status data")
                return
            if fields[1] == "STATUS" and len(fields) == 2 + len(STATUS_FIELDS) and not seen:
                status = dict(zip(STATUS_FIELDS, map(decimal, fields[2:])))
                if status["motor_enabled"] != 0:
                    raise ProtocolError("firmware reports motor output enabled")
                if status["remote_fresh"] not in (0, 1):
                    raise ProtocolError("invalid remote freshness flag")
                status["captured_at"] = now_iso()
                self.report["status"].append(status)
                seen = True
            elif fields[1] == "CAN" and len(fields) == 9 and seen:
                bus, extended, dlc = decimal(fields[2]), decimal(fields[3]), decimal(fields[5])
                raw = parse_hex(fields[6], dlc)
                if not fields[4] or any(c not in "0123456789abcdefABCDEF" for c in fields[4]):
                    raise ProtocolError("invalid CAN identifier")
                identifier = int(fields[4], 16)
                if bus not in (1, 2) or extended not in (0, 1) or dlc > 8 or identifier > (0x1FFFFFFF if extended else 0x7FF):
                    raise ProtocolError("invalid CAN frame metadata")
                frame = {"captured_at": now_iso(), "bus": bus, "extended": bool(extended),
                         "id_hex": fields[4], "dlc": dlc, "data_hex": raw.hex().upper(),
                         "count": decimal(fields[7]), "age_ms": decimal(fields[8])}
                # The C610 manual defines the final two bytes as null, not temperature.
                if bus == 2 and not extended and identifier == 0x205 and dlc == 8:
                    frame["c610_feedback"] = {"rotor_angle_raw": int.from_bytes(raw[:2], "big"),
                                               "rotor_rpm": int.from_bytes(raw[2:4], "big", signed=True),
                                               "torque_feedback_raw": int.from_bytes(raw[4:6], "big", signed=True),
                                               "reserved_hex": raw[6:].hex().upper()}
                self.report["passive_can"].append(frame)
            else:
                raise ProtocolError("unexpected STATUS reply")

    def request(self, kind: str, *values: int) -> None:
        command = read_command(kind, *values)
        self.progress(f"正在读取：{command}")
        self.send(command)
        deadline = self.clock() + self.timeout
        key = [kind] + [str(v) for v in values]
        begun = False
        data = bytearray()
        total = None
        while True:
            fields = self.line(deadline)
            if fields == ["D", "BEGIN"] + key and not begun:
                begun = True
                continue
            if fields == ["D", "DONE"] + key:
                if not begun or not data or (kind == "VESC" and len(data) != total):
                    raise ProtocolError("request completed without a full reply")
                break
            if not begun:
                raise ProtocolError("reply arrived without matching BEGIN")
            if kind == "DM" and len(fields) == 4 and fields[1:3] == ["DM", key[1]] and not data:
                data.extend(parse_hex(fields[3], 8))
            elif kind == "VESC" and len(fields) == 7 and fields[1:4] == key:
                size, offset = decimal(fields[4]), decimal(fields[5])
                chunk = parse_hex(fields[6])
                if not 1 <= size <= 2048 or not 1 <= len(chunk) <= 32 or offset != len(data) or offset + len(chunk) > size:
                    raise ProtocolError("invalid VESC chunk offset, length or total size")
                if total is not None and total != size:
                    raise ProtocolError("VESC total length changed within a reply")
                total = size
                data.extend(chunk)
            else:
                raise ProtocolError("unexpected or mismatched request reply")
        entry = {"captured_at": now_iso(), "raw_hex": bytes(data).hex().upper(), "length": len(data)}
        try:
            entry["decoded"] = decode_dm(bytes(data), values[0]) if kind == "DM" else decode_vesc(bytes(data), values[1])
        except DecodeError as exc:
            entry["decode_error"] = str(exc)
        if kind == "DM":
            self.report["dm"][str(values[0])] = entry
        else:
            self.report["vesc"][str(values[0])][str(values[1])] = entry

    def run(self) -> dict:
        try:
            self.enter()
            self.status()
            requests = [("DM", rid) for rid in DM_FIELDS]
            requests += [("VESC", node, cmd) for node in VESC_NODES for cmd in VESC_COMMANDS]
            for request in requests:
                try:
                    self.request(*request)
                except DeviceError as exc:
                    self.report["failures"].append({"request": read_command(*request), "reason": str(exc)})
                    if str(exc) != "TIMEOUT":
                        raise
                    self.progress("设备未回复，本项记录为超时，继续其他只读项目。")
            self.status()
            self.report["complete"] = not self.report["failures"]
        except (DeviceError, ProtocolError, TimeoutError, OSError, KeyboardInterrupt) as exc:
            self.report["failures"].append({"request": "session", "reason": str(exc) or "user interrupted"})
            self.progress(f"采集已停止：{str(exc) or '用户中断'}")
        finally:
            try:
                self.send("DIAG OFF")
            except (OSError, ProtocolError) as exc:
                self.report["failures"].append({"request": "DIAG OFF", "reason": str(exc)})
                self.report["complete"] = False
            self.report["finished_at"] = now_iso()
        return self.report

    def save(self, directory: Path) -> None:
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "参数采集报告.json").write_text(json.dumps(self.report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        (directory / "串口原始记录.txt").write_text("\n".join(self.log) + "\n", encoding="utf-8")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Comet USART1 电机只读参数采集；先关闭 VOFA 串口。")
    parser.add_argument("--list", action="store_true", help="仅列出可用串口，不连接")
    parser.add_argument("--port", help="USART1 USB 转串口端口，例如 COM5")
    parser.add_argument("--output", type=Path, help="报告输出目录")
    parser.add_argument("--timeout", type=float, default=4.0, help="每项接收超时秒数，3~30，默认4")
    args = parser.parse_args(argv)
    try:
        import serial
        from serial.tools import list_ports
    except ImportError:
        dependency_path = Path(__file__).resolve().parent / ".deps"
        print(f'缺少 pyserial，请在 PowerShell 执行：& "{sys.executable}" -m pip install --target "{dependency_path}" pyserial==3.5')
        return 2
    if not 3 <= args.timeout <= 30:
        parser.error("--timeout 必须在3~30秒范围内")
    ports = list(list_ports.comports())
    if args.list or not args.port:
        for port in ports:
            print(f"{port.device}: {port.description}")
        if not ports:
            print("没有检测到串口，请检查 USB 转串口及驱动。")
        if args.list:
            return 0
        if not ports:
            return 2
        args.port = input("输入 USART1 所用串口名称（例如 COM5）：").strip()
    output = args.output or (Path(__file__).resolve().parents[3] / "02_过程记录" / ("参数采集_" + datetime.now().strftime("%Y%m%d_%H%M%S")))
    try:
        with serial.Serial(args.port, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE, timeout=0.05, write_timeout=1,
                           xonxoff=False, rtscts=False, dsrdtr=False) as transport:
            capture = Capture(transport, timeout=args.timeout)
            capture.report["serial"]["port"] = args.port
            capture.run()
            capture.save(output)
    except OSError as exc:
        print(f"串口打开/报告写入失败：{exc}。请关闭 VOFA 并核对串口号。")
        return 2
    print(f"报告已保存到：{output.resolve()}")
    print("请保留 参数采集报告.json 和 串口原始记录.txt，两份文件一起用于补全代码。")
    return 0 if capture.report["complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
