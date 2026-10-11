"""现有 ARM GCC 编译真实 App C，再以 Unicorn 执行；不连接硬件。

测试 ELF 保存在临时目录，源代码和测试结果哈希保存在本 Tests 目录。
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
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--gcc-dir", type=Path,
                        default=Path.home() / ".eide/tools/gcc_arm/bin")
    parser.add_argument("--unicorn-dir", type=Path,
                        default=(root / ".deps" if (root / ".deps/unicorn/__init__.py").is_file()
                                 else Path(tempfile.gettempdir()) / "gm6020-unicorn-tests"))
    parser.add_argument("--group", choices=("ibus", "remote_control", "remote_integration", "all"), default="all")
    args = parser.parse_args()
    sys.path.insert(0, str(args.unicorn_dir))
    from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
    from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC

    app = root.parent / "App"
    output = Path(tempfile.mkdtemp(prefix="remote-ibus-arm-selftests-"))
    gcc = args.gcc_dir / "arm-none-eabi-gcc.exe"
    nm = args.gcc_dir / "arm-none-eabi-nm.exe"
    groups = {"ibus": [root / "test_ibus.c", app / "ibus.c"],
              "remote_control": [root / "test_remote_control.c", app / "remote_control.c", app / "ibus.c",
                                 app / "gm6020_codec.c", app / "can_protocol.c"],
              "remote_integration": [root / "test_remote_integration.c", app / "remote_input.c",
                                     app / "motor_console.c", app / "remote_app.c", app / "remote_control.c",
                                     app / "ibus.c", app / "gm6020_codec.c", app / "can_protocol.c"]}
    selected = groups if args.group == "all" else {args.group: groups[args.group]}
    results = []
    for name, sources in selected.items():
        missing = [str(source) for source in sources if not source.is_file()]
        if missing:
            raise RuntimeError("Missing test sources: " + ", ".join(missing))
        tested = sources + [root / "test.ld", root / "run_selftests.py", app / "ibus.h"]
        if name != "ibus":
            tested += [app / "remote_control.h", app / "app_config.h",
                       app / "gm6020.h", app / "can_protocol.h"]
        if name == "remote_integration":
            tested += [app / "remote_input.h", app / "remote_app.h", app / "motor_console.h",
                       app / "motor_control.h", app / "can_debug.h"]
            tested += sorted((root / "mocks_integration").glob("*.h"))
        source_hashes = {str(source.relative_to(root.parent)).replace("\\", "/"):
                         hashlib.sha256(source.read_bytes()).hexdigest()
                         for source in tested if source.is_file()}
        elf = output / (name + ".elf")
        command = [str(gcc), "-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=soft", "-std=c99",
                   "-O0", "-g", "-Wall", "-Wextra", "-Werror", "-ffunction-sections", "-fdata-sections",
                   "-I", str(root / "mocks_integration"), "-I", str(app),
                   "-nostartfiles", "--specs=nosys.specs", "-Wl,--gc-sections",
                   "-T", str(root / "test.ld"), "-o", str(elf)]
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

        # 编译前后校验同一份源文件，避免并发编辑使报告哈希与实际测试不同。
        after_hashes = {str(source.relative_to(root.parent)).replace("\\", "/"):
                        hashlib.sha256(source.read_bytes()).hexdigest()
                        for source in tested if source.is_file()}
        if source_hashes != after_hashes:
            raise RuntimeError("Test sources changed during compilation/execution: " + name)
        result = {"name": name, "checks": value("test_checks"),
                  "failures": value("test_failures"), "first_failed_line": value("test_failed_line"),
                  "stage": value("test_stage"), "finished": value("test_done"),
                  "returned": uc.reg_read(UC_ARM_REG_PC) == stop,
                  "source_hashes_stable": True,
                  "source_sha256": source_hashes}
        results.append(result)
        print(json.dumps(result, ensure_ascii=False))
    success = all(item["finished"] == 1 and item["returned"] and item["failures"] == 0
                  for item in results)
    report = {"recorded_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
              "method": "Real ARM C, GCC -Wall -Wextra -Werror, Unicorn; no physical hardware",
              "compiler": str(gcc), "elf_directory": str(output),
              "results": results, "success": success}
    report_path = root / ("selftest_results.json" if args.group == "all" else
                          "selftest_" + args.group + "_results.json")
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("Report:", report_path)
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
