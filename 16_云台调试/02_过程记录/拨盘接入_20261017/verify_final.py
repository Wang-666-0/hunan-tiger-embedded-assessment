"""Verify final source/build/test evidence. No hardware or runtime state changes."""
from pathlib import Path
import hashlib
import json
import re
import xml.etree.ElementTree as ET

record = Path(__file__).resolve().parent
root = record.parents[1]
project = root / "Output/Gimbal_Control"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


protected = json.loads((record / "原有执行器哈希.json").read_text(encoding="utf-8-sig"))
for item in protected:
    assert digest(Path(item["Path"])).upper() == item["Hash"].upper(), item["Path"]
original_config = (record / "before/App/app_config.h").read_text(encoding="utf-8-sig")
current_config = (project / "App/app_config.h").read_text(encoding="utf-8-sig")
for line in original_config.splitlines():
    if line.startswith("#define "):
        assert line in current_config, line
logic_header = (project / "App/app_logic.h").read_bytes()
(record / "before/App/app_logic.h").write_bytes(logic_header.replace(b", APP_STOP_SOURCE_FEEDER", b""))
eide = (project / "MDK-ARM/.eide/eide.yml").read_text(encoding="utf-8-sig")
uvproj = ET.parse(project / "MDK-ARM/Gimbal_Control.uvprojx")
for name in ("controller", "backend", "console"):
    source = "../App/feeder_" + name + ".c"
    assert eide.count("path: " + source) == 1, source
    assert len([f for f in uvproj.iter("FilePath") if f.text == source]) == 1, source
build = json.loads((record / "build_validation.json").read_text(encoding="utf-8"))
for name, expected in build["source_sha256"].items():
    assert digest(root / name) == expected, name
report_path = project / "Tests/selftest_results.json"
report = json.loads(report_path.read_text(encoding="utf-8"))
assert report["success"] and len(report["results"]) == 68
for group in report["results"]:
    assert not group["failures"] and group["finished"] and group["returned"], group["name"]
    for name, expected in group["source_sha256"].items():
        assert digest(project / name) == expected, name
(record / "软件自检.json").write_bytes(report_path.read_bytes())
result = dict(success=True, groups=len(report["results"]),
    checks=sum(g["checks"] for g in report["results"]), failures=0,
    protected_files_unchanged=len(protected), original_config_defines_preserved=True,
    eide_and_uvproj_sources_registered=True, build_sources=build["sources"],
    static_stack_bytes_plus_unknown=build["static_stack_bytes_plus_unknown"],
    hardware_tested=False, flashed=False, hex_generated=False,
    remaining="Physical DIR/GEAR calibration, VOFA GUI import, PID and actual 20Hz validation")
(record / "最终核验.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps(result, ensure_ascii=False))
