"""Run production CAN init and ST HAL on simulated registers. No hardware access."""
from pathlib import Path
import hashlib
import importlib.util
import json
import struct
import sys
import tempfile

record = Path(__file__).resolve().parent
root = record.parents[1]
project = root / "Output/Gimbal_Control"
tests = project / "Tests"
sys.path.insert(0, str(tests / ".deps"))
sys.path.insert(0, str(root.parent / "15_遥控使用/Output/Remote_GM6020/Tests/.deps"))
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC

spec = importlib.util.spec_from_file_location("selftests", tests / "run_selftests.py")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


before = json.loads((record / "修改前哈希.json").read_text(encoding="utf-8-sig"))
changed = [str(Path(item["Path"]).relative_to(project)).replace("\\", "/") for item in before
           if sha(Path(item["Path"])).upper() != item["Hash"].upper()]
assert set(changed) == {"Core/Src/can.c", "Gimbal_Control.ioc", "App/app_config.h"}, changed
old = (record / "before/Core/Src/can.c").read_text(encoding="utf-8-sig")
new = (project / "Core/Src/can.c").read_text(encoding="utf-8-sig")
assert new == old.replace("  hcan2.Init.AutoRetransmission = DISABLE;",
    "  /* CAN2共享C610/VESC：仲裁让位由硬件重试，错误/STOP仍撤权并取消旧邮箱。 */\n"
    "  hcan2.Init.AutoRetransmission = ENABLE;")
assert (project / "Gimbal_Control.ioc").read_text(encoding="utf-8-sig") == (
    record / "before/Gimbal_Control.ioc").read_text(encoding="utf-8-sig").replace("CAN2.NART=DISABLE", "CAN2.NART=ENABLE")
assert (project / "App/app_config.h").read_text(encoding="utf-8-sig") == (
    record / "before/App/app_config.h").read_text(encoding="utf-8-sig").replace("C610_POSITION_DIAG_V1B", "C610_POSITION_CAN2_V1C")

sources = [record / "test_can_registers.c", project / "Core/Src/can.c",
           project / "Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c"]
includes = [project / name for name in ("Core/Inc", "App", "Drivers/STM32F4xx_HAL_Driver/Inc",
    "Drivers/CMSIS/Device/ST/STM32F4xx/Include", "Drivers/CMSIS/Include")]
tracked = sources + [Path(__file__), project / "Gimbal_Control.ioc"]
for folder in includes:
    tracked += sorted(folder.glob("*.h"))
hashes = {str(path.relative_to(root)).replace("\\", "/"): sha(path) for path in tracked}
output = Path(tempfile.mkdtemp(prefix="gimbal-can2-registers-"))
gcc_dir = Path.home() / ".eide/tools/gcc_arm/bin"
elf = output / "can2_registers.elf"
command = [str(gcc_dir / "arm-none-eabi-gcc.exe"), "-mcpu=cortex-m4", "-mthumb",
    "-mfloat-abi=soft", "-std=c99", "-O0", "-g", "-DSTM32F405xx", "-DUSE_HAL_DRIVER",
    "-Wall", "-Wextra", "-Werror", "-ffunction-sections", "-fdata-sections"]
for folder in includes:
    command += ["-I", str(folder)]
command += ["-nostartfiles", "--specs=nosys.specs", "-Wl,--gc-sections", "-T", str(tests / "test.ld"),
            "-o", str(elf)] + [str(path) for path in sources]
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


def value(name):
    return struct.unpack("<I", bytes(uc.mem_read(symbols["test_" + name], 4)))[0]


result = {name: value(name) for name in ("checks", "failures", "failed_line", "done", "stage")}
result["returned"] = uc.reg_read(UC_ARM_REG_PC) == stop
assert result["done"] and result["returned"] and not result["failures"], result
for name, expected in hashes.items():
    assert sha(root / name) == expected, name
report = dict(success=True, production_changes=changed, register_fixture=result, source_sha256=hashes,
    can1_auto_retransmission=True, can2_auto_retransmission=True, hardware_verified=False,
    limitation="Simulated register inputs test actual HAL branches and cancellation requests, not physical CAN arbitration or stop timing")
(record / "register_validation.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({k:v for k,v in report.items() if k != "source_sha256"}, ensure_ascii=False))
