"""编译正式C逻辑和CAN传输，再用Unicorn运行；不连接串口、CAN或烧录器。"""
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
    parser.add_argument("--gcc-dir", type=Path, default=Path.home() / ".eide/tools/gcc_arm/bin")
    shared_deps = root.parents[3] / "15_遥控使用/Output/Remote_GM6020/Tests/.deps"
    parser.add_argument("--unicorn-dir", type=Path,
                        default=root / ".deps" if (root / ".deps/unicorn").is_dir() else shared_deps)
    args = parser.parse_args()
    sys.path.insert(0, str(args.unicorn_dir))
    from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
    from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC
    app = root.parent / "App"
    output = Path(tempfile.mkdtemp(prefix="gimbal-foundation-selftests-"))
    groups = {
        "ibus": [root / "test_ibus.c", app / "ibus.c"],
        "logic": [root / "test_logic.c"] + [app / (name + ".c") for name in
                    ("app_logic", "remote_mapper", "safety_manager", "target_manager", "motor_backend", "can_frame")],
        "legacy_remote": [root / "test_legacy_remote.c", app / "remote_mapper.c"],
        "dm_diag": [root / "test_dm_diag.c", app / "dm_diag.c", app / "can_frame.c"],
        "vesc_diag": [root / "test_vesc_diag.c", app / "vesc_diag.c"],
        "can_bus": [root / "test_can_bus.c"] + [app / (name + ".c") for name in
                    ("can_bus", "can_frame", "dm_diag", "vesc_diag")],
        "diagnostics": [root / "test_app_diagnostics.c"] + [app / (name + ".c") for name in
                    ("app_diagnostics", "can_frame", "dm_diag", "vesc_diag")],
        "console": [root / "test_app_console.c"] + [app / (name + ".c") for name in
                    ("app_console", "can_frame", "dm_diag", "vesc_diag")],
    }
    results = []
    gcc = args.gcc_dir / "arm-none-eabi-gcc.exe"
    nm = args.gcc_dir / "arm-none-eabi-nm.exe"
    for name, sources in groups.items():
        tested = sources + sorted(app.glob("*.h")) + sorted((root / "mocks").glob("*.h"))
        tested += [root / "test.ld", Path(__file__)]
        hashes = {str(p.relative_to(root.parent)).replace("\\", "/"):
                  hashlib.sha256(p.read_bytes()).hexdigest() for p in tested}
        elf = output / (name + ".elf")
        profile_flags = ["-DAPP_REMOTE_PROFILE_LEGACY=0"] if name == "logic" else []
        run([str(gcc), "-mcpu=cortex-m4", "-mthumb", "-mfloat-abi=soft", "-std=c99", "-O0", "-g"] + profile_flags + [
             "-Wall", "-Wextra", "-Werror", "-ffunction-sections", "-fdata-sections",
             "-I", str(root / "mocks"), "-I", str(app), "-nostartfiles", "--specs=nosys.specs",
             "-Wl,--gc-sections", "-T", str(root / "test.ld"), "-o", str(elf)] +
            [str(p) for p in sources] + ["-lm"])
        symbols = {parts[2]: int(parts[0], 16) for line in
                   run([str(nm), "-n", "--defined-only", str(elf)]).splitlines()
                   if len(parts := line.split()) == 3}
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
        if any(hashlib.sha256(p.read_bytes()).hexdigest() != hashes[str(p.relative_to(root.parent)).replace("\\", "/")]
               for p in tested):
            raise RuntimeError("Test sources changed during execution")
        result = {"name": name, "checks": value("test_checks"), "failures": value("test_failures"),
                  "first_failed_line": value("test_failed_line"), "stage": value("test_stage"),
                  "finished": value("test_done"), "returned": uc.reg_read(UC_ARM_REG_PC) == stop,
                  "source_sha256": hashes}
        results.append(result)
        print(json.dumps({k: v for k, v in result.items() if k != "source_sha256"}, ensure_ascii=False))
    success = all(r["finished"] == 1 and r["returned"] and r["failures"] == 0 for r in results)
    report = {"recorded_at": datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat(timespec="seconds"),
              "method": "ARM GCC -Wall -Wextra -Werror + Unicorn; actual C, HAL/RTOS mocked; no hardware",
              "compiler": str(gcc), "elf_directory": str(output), "results": results, "success": success}
    report_path = root / "selftest_results.json"
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("Report:", report_path)
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
