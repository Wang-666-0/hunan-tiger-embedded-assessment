"""Check source scope, actual ARM C regressions and the complete AC5 build.

No physical hardware operations; previous dated verification reports remain
historical evidence and are not rewritten for this change.
"""
from pathlib import Path
import hashlib
import json
import re
import xml.etree.ElementTree as ET

record = Path(__file__).resolve().parent
workspace = record.parents[1]
project = workspace / "Output/Gimbal_Control"
app = project / "App"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
before = json.loads((record / "修改前哈希.json").read_text(encoding="utf-8-sig"))
changed = {name for name, expected in before.items() if sha(workspace / name) != expected}
expected_app = {"app_config.h", "app_logic.c", "app_logic.h", "app_tasks.c", "app_tasks.h",
                "pitch_controller.c", "pitch_controller.h", "pitch_backend.c", "pitch_backend.h", "pitch_console.c"}
expected_tests = {"test_pitch_controller.c", "test_pitch_backend.c", "test_pitch_console.c", "test_pitch_integration.c"}
assert changed == {"Output/Gimbal_Control/App/" + name for name in expected_app} | {
    "Output/Gimbal_Control/Tests/" + name for name in expected_tests}, changed
old_config = (record / "修改前备份/Output/Gimbal_Control/App/app_config.h").read_text(encoding="utf-8-sig")
config = (app / "app_config.h").read_text(encoding="utf-8-sig")
assert config == old_config.replace('"DM3507_CASCADE_V1"', '"DM3507_CASCADE_V1A"')
limits_source = (app / "pitch_controller.h").read_text(encoding="utf-8")
limits = {}
for name, expected in {"TARGET_MIN": -18, "TARGET_MAX": 33, "HARD_MIN": -19, "HARD_MAX": 34}.items():
    actual = float(re.search(r"#define PITCH_CONTROLLER_" + name + r"_DEG\s+\(?(-?[\d.]+)f", limits_source).group(1))
    assert actual == expected
    limits[name] = actual

tests = json.loads((project / "Tests/selftest_results.json").read_text(encoding="utf-8"))
assert tests["success"] and len(tests["results"]) == 53
for group in tests["results"]:
    assert group["finished"] and group["returned"] and not group["failures"], group["name"]
    for name, expected in group["source_sha256"].items():
        assert sha(project / name) == expected, name
keil = [p.text.replace("\\", "/") for p in ET.parse(project / "MDK-ARM/Gimbal_Control.uvprojx").findall(
    "./Targets/Target/Groups/Group/Files/File/FilePath")]
eide = re.findall(r"- path: ([^\r\n]+\.(?:c|s))\s*$", (project / "MDK-ARM/.eide/eide.yml").read_text(encoding="utf-8"), re.M)
params = json.loads((record / "builder_AC5_O0.params").read_text(encoding="utf-8"))
assert sorted(keil) == sorted(eide) == sorted(params["sourceList"]) and len(keil) == 65
assert params["options"]["c/cpp-compiler"]["optimization"] == "level-0"
build = (record / "完整构建.log").read_text(encoding="utf-8-sig")
assert "build successfully" in build and not re.search(r"(?:Warning|Error):", build)
graph = (project / "MDK-ARM/build/Gimbal_Control/Gimbal_Control.htm").read_text(encoding="utf-8")
stacks = {}
for task, allocated in {"ConsoleTask": 3072, "ControlTask": 3072, "CanTask": 1536}.items():
    m = re.search(r">" + task + r"</STRONG>.*?Max Depth = (\d+)([^<]*)", graph, re.S)
    assert m and int(m.group(1)) < allocated - 512
    stacks[task] = {"known_max_bytes": int(m.group(1)), "allocated_bytes": allocated,
                    "has_unknown_contribution": "Unknown" in m.group(2), "runtime_watermark_measured": False}
protected = {name: expected for name, expected in before.items() if name not in changed}
result = {
    "success": True, "version": "DM3507_CASCADE_V1A", "hardware_verified": False,
    "mechanical_measurement": {"relative_to_user_measured_pose_deg": [-20, 35],
        "old_zero_basis_angle_deg": 0.071, "old_zero_endpoints_deg": [-19.929, 35.071],
        "reflash_reference_instruction": "Use the same measured physical pose for READY zero; do not re-zero at an arbitrary pose."},
    "limits_after_ready_zero_deg": limits, "negative_30_rejected": True,
    "shared_safety_and_pid_conditions_unchanged": True,
    "trace_scope": "first loss of permission after each successful remote unlock; keep through subsequent STOP/OFF/recovery",
    "groups": len(tests["results"]), "checks": sum(g["checks"] for g in tests["results"]),
    "source_count": len(keil), "build_size": re.search(r"Program Size: (Code=\d+ RO-data=\d+ RW-data=\d+ ZI-data=\d+)", build).group(1),
    "stacks": stacks, "changed_files": sorted(changed),
    "protected_unchanged_sha256": protected,
    "current_changed_source_sha256": {name: sha(workspace / name) for name in sorted(changed)},
    "regression_report_sha256": sha(project / "Tests/selftest_results.json"),
    "build_log_sha256": sha(record / "完整构建.log"),
    "unresolved": ["physical cause of brief hold then disable", "measured CAN stability and stop latency", "loaded PID step performance"]
}
(record / "selftest_results.json").write_bytes((project / "Tests/selftest_results.json").read_bytes())
(record / "final_verification.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({k: v for k, v in result.items() if k not in (
    "protected_unchanged_sha256", "current_changed_source_sha256")}, ensure_ascii=False))
