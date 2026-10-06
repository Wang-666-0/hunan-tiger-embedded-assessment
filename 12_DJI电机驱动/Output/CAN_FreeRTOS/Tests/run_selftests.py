"""Compile real application C for ARM and execute with mocked HAL/RTOS.

Dependencies: existing EIDE arm-none-eabi-gcc; unicorn in an isolated directory.
This script never opens a real serial port, ST-Link, or motor connection.
"""
from pathlib import Path
import argparse
import datetime
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
    output = Path(tempfile.mkdtemp(prefix="gm6020-arm-selftests-"))
    gcc = args.gcc_dir / "arm-none-eabi-gcc.exe"
    nm = args.gcc_dir / "arm-none-eabi-nm.exe"
    results = []
    for motor_id, mode in [(1, 0), (5, 1)]:
        elf = output / f"test_id{motor_id}_mode{mode}.elf"
        sources = [root / "selftest.c"] + [app / name for name in
                   ("can_protocol.c", "gm6020_codec.c", "gm6020.c", "motor_control.c", "motor_console.c")]
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
        uc.emu_start(symbols["test_run"] | 1, stop_address, count=5_000_000)

        def value(name):
            return struct.unpack("<I", bytes(uc.mem_read(symbols[name], 4)))[0]

        result = {"motor_id": motor_id, "mode": mode, "checks": value("test_checks"),
                  "failures": value("test_failures"), "failed_line": value("test_failed_line"),
                  "stage": value("test_stage"), "finished": value("test_done")}
        results.append(result)
        print(json.dumps(result))
    report = {"time": datetime.datetime.now().isoformat(timespec="seconds"),
              "method": "real ARM C with HAL/RTOS mocks, Unicorn; no physical hardware",
              "results": results}
    assessment = root.parents[2]
    path = assessment / "02_过程记录" / "软件自测结果.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("Report:", path)
    return 0 if all(r["finished"] == 1 and r["failures"] == 0 for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
