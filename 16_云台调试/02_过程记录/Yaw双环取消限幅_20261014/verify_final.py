"""验证V2B最终源、清单和限定改动范围；不编译或连接硬件。"""
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
assert sum(g["checks"] for g in report["results"]) == 381109
for group in report["results"]:
    assert group["finished"] and group["returned"] and not group["failures"], group["name"]
    for name, expected in group["source_sha256"].items():
        assert digest(project / name) == expected, name
for name in ("yaw_cascade_unlimited", "yaw_cascade_unlimited_console", "yaw_can_unlimited", "yaw_performance"):
    g = next(g for g in report["results"] if g["name"] == name)
    assert not g["compile_defines"], name
before = json.loads((record / "修改前App哈希.json").read_text(encoding="utf-8-sig"))
before = {Path(x["Path"]).name: x["Hash"].lower() for x in before}
changed = {"README.md", "app_config.h", "yaw_cascade.c", "yaw_cascade.h", "yaw_controller.h",
           "yaw_backend.c", "yaw_step.h", "yaw_console.c", "can_bus.c"}
protected = {}
for name, expected in before.items():
    if name not in changed:
        assert digest(app / name) == expected, name
        protected[name] = expected
old_config = (record / "修改前备份/App/app_config.h").read_text(encoding="utf-8")
new_config = (app / "app_config.h").read_text(encoding="utf-8")
normalize = lambda s: re.sub(r'#define APP_YAW_VERSION "[^"]+"', '#define APP_YAW_VERSION "VERSION"', s)
assert normalize(old_config) == normalize(new_config)
def other_can_paths(text):
    first = text.index("HAL_StatusTypeDef CanBus_YawTorque(float torque_nm)")
    end = text.index("HAL_StatusTypeDef CanBus_YawDisable(uint8_t cancel_old)", first)
    return text[:first] + text[end:]
assert other_can_paths((app / "can_bus.c").read_text(encoding="utf-8")) == other_can_paths(
    (record / "修改前备份/App/can_bus.c").read_text(encoding="utf-8"))
tree = ET.parse(project / "MDK-ARM/Gimbal_Control.uvprojx")
keil = sorted(n.text.replace("\\", "/") for n in tree.findall("./Targets/Target/Groups/Group/Files/File/FilePath"))
eide = (project / "MDK-ARM/.eide/eide.yml").read_text(encoding="utf-8")
eide_sources = sorted(re.findall(r"- path: ([^\r\n]+\.(?:c|s))\s*$", eide, re.M))
params = json.loads((record / "builder_AC5_O0.params").read_text(encoding="utf-8"))
assert keil == eide_sources == sorted(params["sourceList"]) and len(keil) == 60
assert params["defines"] == ["USE_HAL_DRIVER", "STM32F405xx"]
log = (record / "完整构建.log").read_text(encoding="utf-8-sig")
assert "build successfully" in log and "Warning:" not in log and "Error:" not in log
assert "Code=66300 RO-data=3452 RW-data=444 ZI-data=34420" in log
assert '"GL60II_CASCADE_STEP30_V2B"' in new_config
result = {
    "version": "GL60II_CASCADE_STEP30_V2B", "success": True,
    "hardware": "V2B not flashed/tested by agent; previous dual-loop screenshot is partial RUNNING",
    "groups": 43, "checks": 381109, "source_lists_match": True, "source_count": 60,
    "configuration_only_version_changed": True, "can_unchanged_outside_yaw_torque": True,
    "protected_app_files": protected,
    "app_sha256": {p.name: digest(p) for p in sorted(app.iterdir()) if p.suffix in (".c", ".h")},
    "build_sha256": digest(record / "完整构建.log"),
    "test_report_sha256": digest(project / "Tests/selftest_results.json"),
}
(record / "selftest_results.json").write_bytes((project / "Tests/selftest_results.json").read_bytes())
(record / "final_verification.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({k: v for k, v in result.items() if k not in ("protected_app_files", "app_sha256")}, ensure_ascii=False))
