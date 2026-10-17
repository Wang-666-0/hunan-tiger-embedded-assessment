"""核对V2G右杆90deg/s；阶跃、PID及所有控制正文保持，不连接硬件。"""
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
assert report["success"] and len(report["results"]) == 44
for group in report["results"]:
    assert group["finished"] and group["returned"] and not group["failures"], group["name"]
    for name, expected in group["source_sha256"].items():
        assert digest(project / name) == expected, name
before = json.loads((record / "修改前App哈希.json").read_text(encoding="utf-8-sig"))
changed = {"README.md", "app_config.h", "yaw_controller.h"}
protected = {}
for item in before:
    name = Path(item["Path"]).name
    if name not in changed:
        assert digest(app / name) == item["Hash"].lower(), name
        protected[name] = item["Hash"].lower()
old = (record / "修改前备份/App/app_config.h").read_text(encoding="utf-8")
new = (app / "app_config.h").read_text(encoding="utf-8")
assert old.replace("GL60II_CASCADE_MANUAL50_V2F", "GL60II_CASCADE_MANUAL50_V2G").replace(
    '#define APP_YAW_RATE_LIMIT_DPS 60.0f', '#define APP_YAW_RATE_LIMIT_DPS 90.0f') == new
values = {
    "APP_YAW_POSITION_KP": 10.0, "APP_YAW_POSITION_KI": 0.15, "APP_YAW_POSITION_KD": 0.01,
    "APP_YAW_SPEED_KP": 0.0037, "APP_YAW_SPEED_KI": 0.003, "APP_YAW_SPEED_KD": 0.000001,
}
for name, expected in values.items():
    assert float(re.search(r'#define ' + name + r' ([0-9.]+)f', new).group(1)) == expected
old_backend = (record / "修改前备份/App/yaw_backend.c").read_text(encoding="utf-8")
new_backend = (app / "yaw_backend.c").read_text(encoding="utf-8")
capture = r'static uint8_t CaptureForStart\(void\).*?(?=\nvoid YawBackend_Init)'
assert re.sub(capture, lambda _: re.search(capture, old_backend, re.S).group(0), new_backend, flags=re.S) == old_backend
limits = (app / "yaw_controller.h").read_text(encoding="utf-8")
old_limits = (record / "修改前备份/App/yaw_controller.h").read_text(encoding="utf-8")
assert old_limits.replace('跟随原CH1满杆60deg/s映射', '跟随CH1配置的满杆速度映射') == limits
assert '#define YAW_CONTROLLER_TARGET_LIMIT_DEG 50.0f' in limits
assert '#define YAW_CONTROLLER_HARD_LIMIT_DEG 60.0f' in limits
assert '#define YAW_CONTROLLER_OFF_ANGLE_LIMIT_DEG 65.0f' in limits
assert '#define YAW_CONTROLLER_MANUAL_RATE_LIMIT_DPS APP_YAW_RATE_LIMIT_DPS' in limits
assert '#define APP_YAW_RATE_LIMIT_DPS 90.0f' in new
assert '(degrees == 30.0f || degrees == -30.0f)' in new_backend
for name in ("yaw_manual50", "yaw_cascade_unlimited", "yaw_cascade_unlimited_console", "yaw_performance"):
    assert not next(g for g in report["results"] if g["name"] == name)["compile_defines"]
assert "#define APP_TELEMETRY_PERIOD_MS 5U" in new
assert "#define APP_CONSOLE_BAUDRATE 921600U" in new
for item in json.loads((record / "修改前串口哈希.json").read_text(encoding="utf-8-sig")):
    assert digest(Path(item["Path"])) == item["Hash"].lower(), item["Path"]
tree = ET.parse(project / "MDK-ARM/Gimbal_Control.uvprojx")
keil = sorted(n.text.replace("\\", "/") for n in tree.findall("./Targets/Target/Groups/Group/Files/File/FilePath"))
eide = (project / "MDK-ARM/.eide/eide.yml").read_text(encoding="utf-8")
sources = sorted(re.findall(r"- path: ([^\r\n]+\.(?:c|s))\s*$", eide, re.M))
params = json.loads((record / "builder_AC5_O0.params").read_text(encoding="utf-8"))
assert keil == sources == sorted(params["sourceList"]) and len(keil) == 60
assert params['defines'] == ['USE_HAL_DRIVER', 'STM32F405xx']
log = (record / "完整构建.log").read_text(encoding="utf-8-sig")
assert "build successfully" in log and "Warning:" not in log and "Error:" not in log
size = re.search(r"Program Size: (Code=\d+ RO-data=\d+ RW-data=\d+ ZI-data=\d+)", log)
assert size
result = {
    "version": "GL60II_CASCADE_MANUAL50_V2G", "success": True,
    "hardware": "No flashing or physical test by agent",
    "response_definition": "first feedback inside target +/-0.2 deg; no dwell; never overwritten",
    "settle_hold_ms": 50, "telemetry_period_ms": 5, "baudrate": 921600,
    "compiled_default_pid": values,
    "manual_target_deg": [-50, 50], "manual_max_dps": 90,
    "feedforward_added": False, "step_control_sources_unchanged": True,
    "manual_mapping": "CH1 velocity, centered hold; CH5 middle/manual, low/off, high/test; CH8 lock/manual rearm unchanged",
    "step_targets_deg": [-30, 30],
    "physical_baseline": "User ID29 before V2E flash: +30 deg, response194ms, overshoot0.070deg, SSE-0.038deg valid, PASS1, settle2194ms",
    "groups": len(report["results"]), "checks": sum(g["checks"] for g in report["results"]),
    "build_size": size.group(1), "source_count": len(keil),
    "protected_app_files": protected,
    "app_sha256": {p.name: digest(p) for p in sorted(app.iterdir()) if p.suffix in (".c", ".h")},
    "build_sha256": digest(record / "完整构建.log"),
    "test_report_sha256": digest(project / "Tests/selftest_results.json"),
}
(record / "selftest_results.json").write_bytes((project / "Tests/selftest_results.json").read_bytes())
(record / "final_verification.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({k: v for k, v in result.items() if k not in ("protected_app_files", "app_sha256")}, ensure_ascii=False))
