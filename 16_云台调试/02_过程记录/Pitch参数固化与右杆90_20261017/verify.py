"""Validate source/build evidence only; does not access serial/CAN/flash hardware."""
from pathlib import Path
import hashlib
import json
import re

record = Path(__file__).resolve().parent
root = record.parents[1]
firmware = root / "Output/Gimbal_Control"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


before = json.loads((record / "修改前哈希.json").read_text(encoding="utf-8"))
after = json.loads((record / "修改后哈希.json").read_text(encoding="utf-8"))
for name, expected in after.items():
    assert sha(root / name) == expected, name
allowed_app = {
    "app_config.h", "app_console.c", "pitch_backend.c", "pitch_controller.c",
    "pitch_controller.h", "pitch_console.c", "yaw_console.c"
}
allowed_tests = {
    "run_selftests.py", "test_pitch_backend.c", "test_pitch_console.c", "test_pitch_console_dma.c",
    "test_pitch_controller.c", "test_pitch_gravity.c", "test_pitch_remote.c", "test_pitch_telemetry.c",
    "test_yaw_cascade_unlimited_console.c", "test_yaw_console.c", "test_yaw_manual50.c"
}
allowed = {"Output/Gimbal_Control/App/" + n for n in allowed_app}
allowed |= {"Output/Gimbal_Control/Tests/" + n for n in allowed_tests}
changed = [n for n, h in before.items() if sha(root / n) != h]
assert set(changed) <= allowed, changed
report = json.loads((record / "软件自检.json").read_text(encoding="utf-8"))
assert report["success"] and len(report["results"]) == 58
assert sum(g["checks"] for g in report["results"]) == 464316
for group in report["results"]:
    assert not group["failures"] and group["finished"] and group["returned"], group["name"]
    for name, expected in group["source_sha256"].items():
        assert sha(firmware / name) == expected, name
log = (record / "完整构建.log").read_text(encoding="utf-8-sig")
assert "build successfully" in log and "ARM Compiler 5.06" in log
assert not re.search(r"(?:warning|error)\s*[:#]", log, re.I)
params = json.loads((record / "builder_AC5_O0.params").read_text(encoding="utf-8-sig"))
assert len(params["sourceList"]) == 65
binary = firmware / "MDK-ARM/build/Gimbal_Control/Gimbal_Control.axf"
data = binary.read_bytes()
assert b"DM3507_CASCADE_V1E" in data and b"GL60II_CASCADE_MANUAL50_V2H" in data
source_paths = list((firmware / "App").glob("*.c")) + list((firmware / "App").glob("*.h"))
assert binary.stat().st_mtime >= max(p.stat().st_mtime for p in source_paths)
html = binary.with_suffix(".htm").read_text(encoding="utf-8", errors="replace")
stacks = {}
for task, allocation in (("ConsoleTask", 3072), ("ControlTask", 3072), ("CanTask", 1536), ("RemoteTask", 1536)):
    match = re.search(">" + task + r"</STRONG>.*?Max Depth = (\d+)", html, re.S)
    assert match and int(match[1]) < allocation
    stacks[task] = int(match[1])
result = {"yaw": "GL60II_CASCADE_MANUAL50_V2H", "pitch": "DM3507_CASCADE_V1E",
          "groups": 58, "checks": 464316, "failures": 0, "modified_backup_files": changed,
          "static_stack_bytes_plus_unknown": stacks, "hardware_flashed_or_tested": False}
(record / "最终核验.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print("PASS: V2H/V1E; 58 groups / 464316 checks; full AC5 build and source/protected-file hashes verified. No hardware.")
