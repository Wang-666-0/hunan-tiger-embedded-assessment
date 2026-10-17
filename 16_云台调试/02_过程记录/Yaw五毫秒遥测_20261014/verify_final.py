"""验证V2C源哈希、921600/5ms配置及电机控制保持，保存离线证据。"""
from pathlib import Path
import hashlib
import json
import re
import xml.etree.ElementTree as ET

record = Path(__file__).resolve().parent
project = record.parents[1] / "Output/Gimbal_Control"
app = project / "App"
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
report = json.loads((project / "Tests/selftest_results.json").read_text(encoding="utf-8"))
assert report["success"] and len(report["results"]) == 43
checks = sum(g["checks"] for g in report["results"])
assert checks == 402233
for group in report["results"]:
    assert group["finished"] and group["returned"] and not group["failures"], group["name"]
    for name, expected in group["source_sha256"].items():
        assert digest(project / name) == expected, name
telemetry = next(g for g in report["results"] if g["name"] == "yaw_telemetry")
assert telemetry["telemetry_max_bytes"] == 245 and telemetry["telemetry_max_wire_us"] == 2659
before = json.loads((record / "修改前App哈希.json").read_text(encoding="utf-8-sig"))
before = {Path(x["Path"]).name: x["Hash"].lower() for x in before}
changed = {"README.md", "app_config.h", "app_console.c", "yaw_console.c"}
protected = {}
for name, expected in before.items():
    if name not in changed:
        assert digest(app / name) == expected, name
        protected[name] = expected
old = (record / "修改前备份/App/app_config.h").read_text(encoding="utf-8")
new = (app / "app_config.h").read_text(encoding="utf-8")
strip = lambda s: re.sub(r'#define APP_YAW_VERSION "[^"]+"', 'VERSION',
    re.sub(r'#define APP_CONSOLE_PERIOD_MS 5U.*?(?=#define REMOTE_TIMEOUT_MS)', 'TELEMETRY_BLOCK\n', s, flags=re.S))
assert strip(old) == strip(new)
assert '#define APP_CONSOLE_BAUDRATE 921600U' in new and '#define APP_TELEMETRY_PERIOD_MS 5U' in new
usart = (project / "Core/Src/usart.c").read_text(encoding="utf-8")
old_usart = (record / "修改前备份/Core/Src/usart.c").read_text(encoding="utf-8")
expected = old_usart.replace('#include "usart.h"', '#include "usart.h"\n#include "app_config.h"').replace(
    'huart1.Init.BaudRate = 115200;', 'huart1.Init.BaudRate = APP_CONSOLE_BAUDRATE;')
assert usart == expected and 'huart4.Init.BaudRate = 115200;' in usart
ioc = (project / "Gimbal_Control.ioc").read_text(encoding="utf-8")
assert 'USART1.BaudRate=921600' in ioc and 'UART4.BaudRate=115200' in ioc
tree = ET.parse(project / "MDK-ARM/Gimbal_Control.uvprojx")
keil = sorted(n.text.replace("\\", "/") for n in tree.findall("./Targets/Target/Groups/Group/Files/File/FilePath"))
eide = (project / "MDK-ARM/.eide/eide.yml").read_text(encoding="utf-8")
eide_sources = sorted(re.findall(r"- path: ([^\r\n]+\.(?:c|s))\s*$", eide, re.M))
params = json.loads((record / "builder_AC5_O0.params").read_text(encoding="utf-8"))
assert keil == eide_sources == sorted(params["sourceList"]) and len(keil) == 60
assert params["defines"] == ["USE_HAL_DRIVER", "STM32F405xx"]
log = (record / "完整构建.log").read_text(encoding="utf-8-sig")
assert "build successfully" in log and "Warning:" not in log and "Error:" not in log
assert "Code=66364 RO-data=3460 RW-data=444 ZI-data=34420" in log
assert '"GL60II_CASCADE_STEP30_V2C"' in new
result = {
    "version": "GL60II_CASCADE_STEP30_V2C", "success": True,
    "hardware": "V2C not flashed/tested by agent; 5ms telemetry repeats last real ~10ms CAN observation",
    "groups": 43, "checks": checks, "source_lists_match": True, "source_count": 60,
    "telemetry_period_ms": 5, "baudrate": 921600,
    "test_sample_max_bytes": 245, "calculated_wire_us": 2659,
    "host_capture_tests": 15, "protected_app_files": protected,
    "app_sha256": {p.name: digest(p) for p in sorted(app.iterdir()) if p.suffix in (".c", ".h")},
    "serial_source_sha256": digest(project / "Core/Src/usart.c"),
    "ioc_sha256": digest(project / "Gimbal_Control.ioc"),
    "capture_tool_sha256": digest(project / "Tools/motor_capture.py"),
    "capture_script_sha256": digest(project / "Scripts/capture.ps1"),
    "build_sha256": digest(record / "完整构建.log"),
    "test_report_sha256": digest(project / "Tests/selftest_results.json"),
}
(record / "selftest_results.json").write_bytes((project / "Tests/selftest_results.json").read_bytes())
(record / "final_verification.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({k: v for k, v in result.items() if k not in ("protected_app_files", "app_sha256")}, ensure_ascii=False))
