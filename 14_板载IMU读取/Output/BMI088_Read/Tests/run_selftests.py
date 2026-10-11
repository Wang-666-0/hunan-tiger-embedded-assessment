"""用现有 ARM GCC 编译真实 App 模块，再用 Unicorn 运行；绝不连接硬件。

测试派生产物保存在系统临时目录，报告打印到终端，避免修改用户已有文档。
"""
from pathlib import Path
import argparse
import hashlib
import json
import struct
import subprocess
import sys
import tempfile


def run(command):
    result = subprocess.run(command, capture_output=True, text=True,
                            encoding="utf-8", errors="replace")
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)
    return result.stdout


def load_elf(uc, path):
    data = path.read_bytes()
    if data[:5] != b"\x7fELF\x01":
        raise RuntimeError("expected 32-bit ELF")
    offset = struct.unpack_from("<I", data, 28)[0]
    size, count = struct.unpack_from("<HH", data, 42)
    for index in range(count):
        kind, file_offset, address, _, file_size, _, _, _ = struct.unpack_from(
            "<8I", data, offset + index * size)
        if kind == 1 and file_size:
            uc.mem_write(address, data[file_offset:file_offset + file_size])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gcc-dir", type=Path, default=Path.home() / ".eide/tools/gcc_arm/bin")
    parser.add_argument("--unicorn-dir", type=Path,
                        default=Path(tempfile.gettempdir()) / "gm6020-unicorn-tests")
    args = parser.parse_args()
    sys.path.insert(0, str(args.unicorn_dir))
    from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
    from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC

    root = Path(__file__).resolve().parent
    app = root.parent / "App"
    output = Path(tempfile.mkdtemp(prefix="bmi088-attitude-selftests-"))
    gcc = args.gcc_dir / "arm-none-eabi-gcc.exe"
    nm = args.gcc_dir / "arm-none-eabi-nm.exe"
    groups = [
        ("mahony", [root / "test_mahony.c", app / "mahony.c"]),
        ("bmi088_driver", [root / "test_bmi088_driver.c", app / "bmi088.c"]),
        ("calibration_telemetry", [root / "test_calibration_telemetry.c",
                                    app / "imu_calibration.c", app / "vofa_telemetry.c"]),
    ]
    results = []
    for name, sources in groups:
        elf = output / (name + ".elf")
        command = [str(gcc), "-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=soft", "-std=c99",
                   "-O0", "-g", "-Wall", "-Wextra", "-Werror", "-ffunction-sections", "-fdata-sections",
                   "-I", str(root / "mocks"), "-I", str(app), "-nostartfiles", "--specs=nosys.specs",
                   "-Wl,--gc-sections", "-T", str(root / "test.ld"), "-o", str(elf)]
        command += [str(source) for source in sources] + ["-lm"]
        run(command)
        symbols = {}
        for line in run([str(nm), "-n", "--defined-only", str(elf)]).splitlines():
            fields = line.split()
            if len(fields) == 3:
                symbols[fields[2]] = int(fields[0], 16)
        uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        uc.mem_map(0x08000000, 1024 * 1024)
        uc.mem_map(0x20000000, 128 * 1024)
        load_elf(uc, elf)
        stop = 0x080FFFF0
        uc.reg_write(UC_ARM_REG_SP, 0x2001FFF0)
        uc.reg_write(UC_ARM_REG_LR, stop | 1)
        uc.emu_start(symbols["test_run"] | 1, stop, count=80_000_000)

        def value(symbol):
            return struct.unpack("<I", bytes(uc.mem_read(symbols[symbol], 4)))[0]

        result = {"name": name, "checks": value("test_checks"),
                  "failures": value("test_failures"), "first_failed_line": value("test_failed_line"),
                  "finished": value("test_done"), "returned": uc.reg_read(UC_ARM_REG_PC) == stop,
                  "source_sha256": {str(source.relative_to(root.parent)): hashlib.sha256(
                      source.read_bytes()).hexdigest() for source in sources}}
        results.append(result)
        print(json.dumps(result, ensure_ascii=False))
    print("模拟 ELF 目录：", output)
    return 0 if all(item["finished"] == 1 and item["returned"] and item["failures"] == 0
                    for item in results) else 1


if __name__ == "__main__":
    sys.exit(main())
