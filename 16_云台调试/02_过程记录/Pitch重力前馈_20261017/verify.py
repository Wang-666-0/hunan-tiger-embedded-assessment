"""Verify this task's source and software/build evidence; never connects hardware."""
from pathlib import Path
import hashlib
import json
import re

record = Path(__file__).resolve().parent
root = record.parents[1]
firmware = root / "Output/Gimbal_Control"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


before = json.loads((record / "修改前哈希.json").read_text(encoding="utf-8-sig"))
after = json.loads((record / "修改后哈希.json").read_text(encoding="utf-8"))
for name, expected in after.items():
    assert sha(root / name) == expected, "changed since final verification: " + name
allowed = {
    "app_config.h", "app_console.c", "pitch_backend.c", "pitch_backend.h", "pitch_console.c",
    "pitch_controller.c", "pitch_controller.h", "pitch_step.h", "yaw_cascade.c", "yaw_cascade.h",
    "yaw_step.h", "test_pitch_console.c", "test_pitch_telemetry.c", "run_selftests.py", "README.md"
}
changed = [n for n, h in before.items() if sha(root / n) != h]
assert all(Path(n).name in allowed for n in changed), changed
report = json.loads((record / "软件自检.json").read_text(encoding="utf-8"))
assert report["success"] and len(report["results"]) == 57
assert sum(g["checks"] for g in report["results"]) == 457565
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
assert b"DM3507_CASCADE_V1D" in binary.read_bytes()
source_paths = [firmware / "App" / n for n in allowed if n.endswith((".c", ".h")) and not n.startswith("test_")]
assert binary.stat().st_mtime >= max(p.stat().st_mtime for p in source_paths)
html = binary.with_suffix(".htm").read_text(encoding="utf-8", errors="replace")
stack = {}
for task, allocation in (("ConsoleTask", 3072), ("ControlTask", 3072), ("CanTask", 1536), ("RemoteTask", 1536)):
    match = re.search(">" + task + r"</STRONG>.*?Max Depth = (\d+)", html, re.S)
    assert match and int(match[1]) < allocation
    stack[task] = int(match[1])
result = {"version": "DM3507_CASCADE_V1D", "groups": 57, "checks": 457565, "failures": 0,
          "modified_backup_files": changed, "static_stack_bytes_plus_unknown": stack,
          "hardware_flashed_or_tested": False}
(record / "最终核验.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print("PASS: V1D, 57 groups / 457565 checks, full AC5 build, hashes and preserved files verified; no hardware.")
