"""Verify current software evidence and preserve the shared firmware baseline.

No serial port, ST-Link, motor parameter writes, or physical tests.
"""
from pathlib import Path
import hashlib
import json
import re
import xml.etree.ElementTree as ET

record = Path(__file__).resolve().parent
project = record.parents[1] / "Output/Gimbal_Control"
app = project / "App"
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
tests = json.loads((project / "Tests/selftest_results.json").read_text(encoding="utf-8"))
assert tests["success"] and len(tests["results"]) == 53
source_hashes = {}
for group in tests["results"]:
    assert group["finished"] and group["returned"] and not group["failures"], group["name"]
    for name, expected in group["source_sha256"].items():
        assert sha(project / name) == expected, name
        source_hashes[name] = expected
old_tests = json.loads((record / "修改前备份/Tests/selftest_results.json").read_text(encoding="utf-8"))
old_names = {group["name"] for group in old_tests["results"]}
assert len(old_names) == 44
assert old_names <= {group["name"] for group in tests["results"]}
protected = (
    "yaw_controller", "yaw_cascade", "yaw_step", "gl60_ii",
    "flywheel_jog", "flywheel_speed", "motor_backend", "safety_manager",
    "remote_input", "ibus", "dm_diag", "vesc_diag", "app_fault", "app_diagnostics",
)
protected_hashes = {}
for stem in protected:
    for suffix in (".c", ".h"):
        name = stem + suffix
        assert sha(app / name) == sha(record / "修改前备份/App" / name), name
        protected_hashes[name] = sha(app / name)
config = (app / "app_config.h").read_text(encoding="utf-8")
expected_pid = {
    "APP_YAW_POSITION_KP": 10, "APP_YAW_POSITION_KI": 0.15, "APP_YAW_POSITION_KD": 0.01,
    "APP_YAW_SPEED_KP": 0.0037, "APP_YAW_SPEED_KI": 0.003, "APP_YAW_SPEED_KD": 0.000001,
    "APP_PITCH_POSITION_KP": 12, "APP_PITCH_POSITION_KI": 0.2, "APP_PITCH_POSITION_KD": 0.02,
    "APP_PITCH_SPEED_KP": 0.02, "APP_PITCH_SPEED_KI": 0.03, "APP_PITCH_SPEED_KD": 0.00005,
}
for name, value in expected_pid.items():
    assert float(re.search(r"#define " + name + r" ([0-9.]+)f", config).group(1)) == value
assert '#define APP_YAW_VERSION "GL60II_CASCADE_MANUAL50_V2G"' in config
assert '#define APP_PITCH_VERSION "DM3507_CASCADE_V1"' in config
assert "#define APP_MOTOR_DRIVERS_IMPLEMENTED 0U" in config
assert "#define APP_YAW_RATE_LIMIT_DPS 90.0f" in config
keil = [entry.text.replace("\\", "/") for entry in ET.parse(project / "MDK-ARM/Gimbal_Control.uvprojx").findall(
    "./Targets/Target/Groups/Group/Files/File/FilePath")]
eide = re.findall(r"- path: ([^\r\n]+\.(?:c|s))\s*$",
    (project / "MDK-ARM/.eide/eide.yml").read_text(encoding="utf-8"), re.M)
params = json.loads((record / "builder_AC5_O0.params").read_text(encoding="utf-8"))
assert sorted(keil) == sorted(eide) == sorted(params["sourceList"])
assert len(keil) == 65 and params["options"]["c/cpp-compiler"]["optimization"] == "level-0"
log = (record / "完整构建.log").read_text(encoding="utf-8-sig")
assert "build successfully" in log and "Warning:" not in log and "Error:" not in log
size = re.search(r"Program Size: (Code=\d+ RO-data=\d+ RW-data=\d+ ZI-data=\d+)", log).group(1)
callgraph = (project / "MDK-ARM/build/Gimbal_Control/Gimbal_Control.htm").read_text(encoding="utf-8")
stack_depths = {}
for task in ("CanTask", "ControlTask", "ConsoleTask"):
    match = re.search(r">" + task + r"</STRONG>.*?Max Depth = (\d+)([^<]*)", callgraph, re.S)
    assert match, task
    stack_depths[task] = {"known_max_depth_bytes": int(match.group(1)),
                          "unknown_stack_contribution": "Unknown" in match.group(2)}
assert stack_depths["ConsoleTask"]["known_max_depth_bytes"] < 2000
telem = next(group for group in tests["results"] if group["name"] == "pitch_telemetry")
result = {
    "pitch_version": "DM3507_CASCADE_V1", "yaw_version": "GL60II_CASCADE_MANUAL50_V2G",
    "success": True, "hardware": "No flashing or physical motor test by agent; hardware verification pending",
    "user_confirmed": {"support_available": True, "zero_pose": "horizontal", "allowed_deg": [-50, 50],
                       "no_cable_or_collision_restriction_within_range": True, "disabled_load_drops": True},
    "groups": len(tests["results"]), "checks": sum(group["checks"] for group in tests["results"]),
    "original_groups_retained": len(old_names), "source_count": len(keil), "build_size": size,
    "compiled_default_pid": expected_pid, "pitch_initial_pid_physically_tuned": False,
    "pitch_response_limit_ms": 300, "yaw_response_limit_ms": 250,
    "response_definition": "first new CAN feedback inside target +/-0.2 degrees; settle reported separately",
    "telemetry": {"baudrate": 921600, "period_ms": 5, "columns_per_group": 48,
                  "default_group": "gimbal (original Yaw)", "explicit_pitch_selection": "TELEM PITCH",
                  "pitch_fixture_max_bytes": telem["telemetry_max_bytes"],
                  "pitch_fixture_wire_us_8N1": telem["telemetry_max_wire_us"],
                  "formatting_or_USB_runtime_measured": False},
    "stack_analysis": stack_depths,
    "protected_app_files": protected_hashes,
    "app_sha256": {path.name: sha(path) for path in sorted(app.iterdir()) if path.suffix in (".c", ".h")},
    "test_report_sha256": sha(project / "Tests/selftest_results.json"),
    "build_log_sha256": sha(record / "完整构建.log"),
}
(record / "selftest_results.json").write_bytes((project / "Tests/selftest_results.json").read_bytes())
(record / "final_verification.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({key: value for key, value in result.items() if key not in ("protected_app_files", "app_sha256")}, ensure_ascii=False))
