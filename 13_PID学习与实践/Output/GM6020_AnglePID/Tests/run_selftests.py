"""Compile real application C for ARM and execute with mocked HAL/RTOS.

Dependencies: existing EIDE arm-none-eabi-gcc; unicorn in an isolated directory.
This script never opens a real serial port, ST-Link, or motor connection.
"""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import struct
import subprocess
import sys
import tempfile


def run(command):
    result = subprocess.run(command, capture_output=True, text=True, encoding="utf-8", errors="replace")
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    return result.stdout


def load_elf(uc, path):
    elf = path.read_bytes()
    assert elf[:5] == b"\x7fELF\x01", "expected 32-bit ELF"
    phoff = struct.unpack_from("<I", elf, 28)[0]
    size, count = struct.unpack_from("<HH", elf, 42)
    for index in range(count):
        kind, offset, address, _, file_size, _, _, _ = struct.unpack_from("<8I", elf, phoff + index * size)
        if kind == 1 and file_size:
            uc.mem_write(address, elf[offset:offset + file_size])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gcc-dir", type=Path, default=Path.home() / ".eide/tools/gcc_arm/bin")
    parser.add_argument("--unicorn-dir", type=Path, default=Path(tempfile.gettempdir()) / "gm6020-unicorn-tests")
    args = parser.parse_args()
    sys.path.insert(0, str(args.unicorn_dir))
    from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
    from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

    root = Path(__file__).resolve().parent
    app = root.parent / "App"
    output = Path(tempfile.mkdtemp(prefix="angle-pid-arm-selftests-"))
    gcc = args.gcc_dir / "arm-none-eabi-gcc.exe"
    nm = args.gcc_dir / "arm-none-eabi-nm.exe"
    results = []
    for motor_id, mode in [(1, 0), (5, 1)]:
        elf = output / f"test_id{motor_id}_mode{mode}.elf"
        sources = [root / "selftest.c"] + [app / name for name in
                   ("pid.c", "angle_tracker.c", "can_protocol.c", "gm6020_codec.c", "gm6020.c", "motor_control.c", "motor_console.c")]
        command = [str(gcc), "-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=soft", "-std=c99",
                   "-O0", "-g", "-Wall", "-Wextra", "-Werror", "-ffunction-sections", "-fdata-sections",
                   "-I", str(root / "mocks"), "-I", str(app),
                   f"-DGM6020_MOTOR_ID={motor_id}", f"-DGM6020_CONTROL_MODE={mode}",
                   "-nostartfiles", "--specs=nosys.specs", "-Wl,--gc-sections",
                   "-T", str(root / "test.ld"), "-o", str(elf)] + [str(p) for p in sources]
        run(command)
        symbols = {}
        for line in run([str(nm), "-n", "--defined-only", str(elf)]).splitlines():
            parts = line.split()
            if len(parts) == 3:
                symbols[parts[2]] = int(parts[0], 16)

        uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        uc.mem_map(0x08000000, 1024 * 1024)
        uc.mem_map(0x20000000, 128 * 1024)
        load_elf(uc, elf)
        stop_address = 0x080FFFF0
        uc.reg_write(UC_ARM_REG_SP, 0x2001FFF0)
        uc.reg_write(UC_ARM_REG_LR, stop_address | 1)
        # 连续运行用例包含 2000 个控制周期，预留足够指令但保留死循环上界。
        instruction_budget = 100_000_000
        uc.emu_start(symbols["test_run"] | 1, stop_address, count=instruction_budget)

        def value(name):
            return struct.unpack("<I", bytes(uc.mem_read(symbols[name], 4)))[0]

        result = {"motor_id": motor_id, "mode": mode, "checks": value("test_checks"),
                  "failures": value("test_failures"), "failed_line": value("test_failed_line"),
                  "first_failed_stage": value("test_failed_stage"),
                  "stage": value("test_stage"), "finished": value("test_done")}
        lines = list(struct.unpack("<32I", bytes(uc.mem_read(symbols["test_failed_lines"], 128))))
        result["first_failed_lines"] = lines[:min(result["failures"], 32)]
        results.append(result)
        print(json.dumps(result))
        source_lines = (root / "selftest.c").read_text(encoding="utf-8").splitlines()
        for line_number in dict.fromkeys(result["first_failed_lines"]):
            print(f"FAIL {line_number}: {source_lines[line_number - 1]}", file=sys.stderr)
        if result["finished"] != 1:
            print(f"ERROR: ID {motor_id}/mode {mode} did not finish within "
                  f"{instruction_budget:,} instructions (stage {result['stage']}).", file=sys.stderr)
    tested_sources = [root / "selftest.c"] + [app / name for name in
        ("pid.c", "angle_tracker.c", "can_protocol.c", "gm6020_codec.c", "gm6020.c", "motor_control.c", "motor_console.c")]
    tested_sources += sorted((root / "mocks").glob("*.h"))
    tested_sources += sorted(app.glob("*.h"))
    report = {"recorded_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
              "method": "real ARM C with HAL/RTOS mocks, Unicorn; no physical hardware",
              "coverage": ["PID scaling, limits, anti-windup, filtered derivative, invalid inputs",
                           "CAN feedback and all seven command slots",
                           "Retained speed control, continuous run, online gains, measured dt and concurrent requests",
                           "Bidirectional encoder unwrap, exact +/-720 degrees, gap/jump/speed/bounds invalid latch, tick wrap and explicit zero",
                           "Cascade angle control, real multiturn targets, speed limit, mode reset, feedback/epoch faults and stopped low-speed zero",
                           "Angle measured dt, integral gain continuity, repeat target/metrics and once-per-stage braking integral clearing",
                           "Angle settling-band/hold and overshoot telemetry, angle/gain/limit/mode/STOP concurrent requests",
                           "Feedback invalid/reference epoch/overtemperature changes during calculation rejected before publishing, coherent command admission",
                           "Strict S/A/AP/AI/AD/P/I/D/V/Z/STOP/T commands, two/21-channel FireWater and RX recovery"],
              "source_sha256": {str(path.relative_to(root.parent)).replace("\\", "/"):
                                hashlib.sha256(path.read_bytes()).hexdigest()
                                for path in tested_sources},
              "results": results}
    assessment = root.parents[2]
    path = assessment / "02_过程记录" / "角度双环软件自测结果.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("Report:", path)
    return 0 if all(r["finished"] == 1 and r["failures"] == 0 for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
