"""Read-only firmware/source verification; never connects to hardware."""
from pathlib import Path
import hashlib
import json
import re
import xml.etree.ElementTree as ET

record = Path(__file__).resolve().parent
root = record.parents[1]
firmware = root / "Output/Gimbal_Control"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


before = json.loads((record / "修改前哈希.json").read_text(encoding="utf-8"))
after = json.loads((record / "修改后哈希.json").read_text(encoding="utf-8"))
for name, expected in after.items():
    assert sha(root / name) == expected, name
apps = {"app_config.h", "app_tasks.c", "app_console.c", "yaw_console.c", "pitch_console.c", "yaw_backend.c", "can_bus.h"}
tests = {"run_selftests.py", "test_app_console.c", "test_pitch_telemetry.c", "test_pitch_console.c",
         "test_yaw_manual50.c", "test_yaw_cascade_unlimited_console.c"}
allowed = {"Output/Gimbal_Control/App/" + n for n in apps}
allowed |= {"Output/Gimbal_Control/Tests/" + n for n in tests}
allowed |= {"Output/Gimbal_Control/MDK-ARM/.eide/eide.yml", "Output/Gimbal_Control/MDK-ARM/Gimbal_Control.uvprojx"}
changed = [n for n, h in before.items() if sha(root / n) != h]
assert set(changed) <= allowed, changed
report = json.loads((record / "软件自检.json").read_text(encoding="utf-8"))
assert report["success"] and len(report["results"]) == 59
assert sum(g["checks"] for g in report["results"]) == 467812
for group in report["results"]:
    assert not group["failures"] and group["finished"] and group["returned"], group["name"]
    for name, expected in group["source_sha256"].items():
        assert sha(firmware / name) == expected, name
log = (record / "完整构建.log").read_text(encoding="utf-8-sig")
assert "build successfully" in log and "ARM Compiler 5.06" in log
assert not re.search(r"(?:warning|error)\s*[:#]", log, re.I)
params = json.loads((record / "builder_AC5_O0.params").read_text(encoding="utf-8"))
assert len(params["sourceList"]) == 66 and params["sourceList"].count("../App/gimbal_startup.c") == 1
xml = ET.parse(firmware / "MDK-ARM/Gimbal_Control.uvprojx")
paths = [n.text for n in xml.findall(".//FilePath")]
assert len(paths) == 66 and paths.count("../App/gimbal_startup.c") == 1
assert (firmware / "MDK-ARM/.eide/eide.yml").read_text(encoding="utf-8").count("../App/gimbal_startup.c") == 1
binary = firmware / "MDK-ARM/build/Gimbal_Control/Gimbal_Control.axf"
data = binary.read_bytes()
assert b"DM3507_CASCADE_V1F" in data and b"GL60II_CASCADE_MANUAL50_V2I" in data
sources = list((firmware / "App").glob("*.c")) + list((firmware / "App").glob("*.h"))
assert binary.stat().st_mtime >= max(p.stat().st_mtime for p in sources)
html = binary.with_suffix(".htm").read_text(encoding="utf-8", errors="replace")
stacks = {}
for task, allocation in (("ConsoleTask", 3072), ("ControlTask", 3072), ("CanTask", 1536), ("RemoteTask", 1536)):
    match = re.search(">" + task + r"</STRONG>.*?Max Depth = (\d+)", html, re.S)
    assert match and int(match[1]) < allocation
    stacks[task] = int(match[1])
result = {"yaw": "GL60II_CASCADE_MANUAL50_V2I", "pitch": "DM3507_CASCADE_V1F", "groups": 59,
          "checks": 467812, "failures": 0, "modified_backup_files": changed,
          "static_stack_bytes_plus_unknown": stacks, "hardware_flashed_or_tested": False}
(record / "最终核验.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print("PASS: V2I/V1F; 59 groups / 467812 checks; 66-source full build and hashes verified. No hardware.")
