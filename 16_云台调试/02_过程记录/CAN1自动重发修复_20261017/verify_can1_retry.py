"""Verify the narrowly scoped CAN1 fix with actual initialization and ST HAL C.

No serial, debugger, physical CAN device, or flashing. Simulated registers are
inputs to HAL logic, not a model or proof of physical CAN arbitration/timing.
"""
from pathlib import Path
import datetime
import hashlib
import importlib.util
import json
import re
import struct
import sys
import tempfile

record = Path(__file__).resolve().parent
workspace = record.parents[1]
project = workspace / "Output/Gimbal_Control"
tests = project / "Tests"
sys.path.insert(0, str(tests / ".deps"))
sys.path.insert(0, str(workspace.parent / "15_遥控使用/Output/Remote_GM6020/Tests/.deps"))
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC

spec = importlib.util.spec_from_file_location("selftests", tests / "run_selftests.py")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
before = json.loads((record / "修改前哈希.json").read_text(encoding="utf-8-sig"))
changed = {name for name, expected in before.items() if sha(workspace / name) != expected}
assert changed == {"Output/Gimbal_Control/Core/Src/can.c", "Output/Gimbal_Control/Gimbal_Control.ioc"}, changed

# Validate the two-line policy change against the saved pre-change sources.
old = (record / "修改前备份/can.c").read_text(encoding="utf-8")
new = (project / "Core/Src/can.c").read_text(encoding="utf-8")
expected = old.replace(
    "  hcan1.Init.AutoRetransmission = DISABLE; /* 只读查询只提交一次，不在无ACK时持续重发。 */",
    "  /* CAN1共享运动总线：仲裁让位后由硬件重试；实际总线错误仍由ISR撤权并取消邮箱。 */\n"
    "  hcan1.Init.AutoRetransmission = ENABLE;")
assert new == expected
old_ioc = (record / "修改前备份/Gimbal_Control.ioc").read_text(encoding="utf-8")
assert (project / "Gimbal_Control.ioc").read_text(encoding="utf-8") == old_ioc.replace(
    "CAN1.NART=DISABLE", "CAN1.NART=ENABLE")

source_paths = [record / "test_can_registers.c", project / "Core/Src/can.c",
                project / "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c"]
include_paths = [project / p for p in ("Core/Inc", "App", "Drivers/STM32F4xx_HAL_Driver/Inc",
    "Drivers/CMSIS/Device/ST/STM32F4xx/Include", "Drivers/CMSIS/Include")]
tested = source_paths + [record / "verify_can1_retry.py", project / "Gimbal_Control.ioc"]
for folder in include_paths:
    tested.extend(sorted(folder.glob("*.h")))
source_hashes = {str(p.relative_to(workspace)).replace("\\", "/"): sha(p) for p in tested}
output = Path(tempfile.mkdtemp(prefix="gimbal-can-registers-"))
gcc_dir = Path.home() / ".eide/tools/gcc_arm/bin"
elf = output / "can_registers.elf"
command = [str(gcc_dir / "arm-none-eabi-gcc.exe"), "-mcpu=cortex-m4", "-mthumb",
    "-mfloat-abi=soft", "-std=c99", "-O0", "-g", "-DSTM32F405xx", "-DUSE_HAL_DRIVER",
    "-Wall", "-Wextra", "-Werror", "-ffunction-sections", "-fdata-sections"]
for folder in include_paths:
    command.extend(["-I", str(folder)])
command.extend(["-nostartfiles", "--specs=nosys.specs", "-Wl,--gc-sections", "-T",
    str(tests / "test.ld"), "-o", str(elf)] + [str(p) for p in source_paths])
runner.run(command)
symbols = {v[2]: int(v[0], 16) for line in runner.run([
    str(gcc_dir / "arm-none-eabi-nm.exe"), "-n", "--defined-only", str(elf)]).splitlines()
    if len(v := line.split()) == 3}
uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
uc.mem_map(0x08000000, 1024 * 1024)
uc.mem_map(0x20000000, 128 * 1024)
uc.mem_map(0x40000000, 256 * 1024)
runner.load_elf(uc, elf)
stop = 0x080FFFF0
uc.reg_write(UC_ARM_REG_SP, 0x2001FFF0)
uc.reg_write(UC_ARM_REG_LR, stop | 1)
uc.emu_start(symbols["test_run"] | 1, stop, count=1_000_000)
value = lambda name: struct.unpack("<I", bytes(uc.mem_read(symbols[name], 4)))[0]
register_result = {name: value("test_" + name) for name in
                   ("checks", "failures", "failed_line", "done", "stage")}
register_result["returned"] = uc.reg_read(UC_ARM_REG_PC) == stop
assert register_result["done"] and register_result["returned"] and not register_result["failures"], register_result
for name, expected_hash in source_hashes.items():
    assert sha(workspace / name) == expected_hash, name

regression = json.loads((tests / "selftest_results.json").read_text(encoding="utf-8"))
assert regression["success"] and len(regression["results"]) == 53
for group in regression["results"]:
    assert group["finished"] and group["returned"] and not group["failures"], group["name"]
    for name, expected_hash in group["source_sha256"].items():
        assert sha(project / name) == expected_hash, name
build = (record / "完整构建.log").read_text(encoding="utf-8-sig")
assert "build successfully" in build and not re.search(r"(?:Warning|Error):", build)
result = {
    "recorded_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
    "success": True, "changed_production_files": sorted(changed),
    "can1_auto_retransmission": True, "can2_auto_retransmission": False,
    "register_fixture": register_result, "register_test_sha256": source_hashes,
    "regression_groups": len(regression["results"]),
    "regression_checks": sum(g["checks"] for g in regression["results"]),
    "build_size": re.search(r"Program Size: (Code=\d+ RO-data=\d+ RW-data=\d+ ZI-data=\d+)", build).group(1),
    "build_log_sha256": sha(record / "完整构建.log"),
    "regression_report_sha256": sha(tests / "selftest_results.json"),
    "hardware_verified": False,
    "limitation": "Simulated register inputs verify HAL software branches; physical arbitration, stop timing, remote UART freshness and PID performance require user validation."
}
(record / "selftest_results.json").write_bytes((tests / "selftest_results.json").read_bytes())
(record / "final_verification.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({k: v for k, v in result.items() if k != "register_test_sha256"}, ensure_ascii=False))
