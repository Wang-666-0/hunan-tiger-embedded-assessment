"""Validate V1B source/build/selftest evidence without accessing any hardware."""
from pathlib import Path
import hashlib
import json
import re

record = Path(__file__).resolve().parent
root = record.parents[1]
project = root / "Output/Gimbal_Control"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


baseline = json.loads((record / "接入前App哈希.json").read_text(encoding="utf-8-sig"))
allowed = {"app_config.h", "feeder_controller.c", "feeder_controller.h", "feeder_console.c"}
changed = [Path(item["Path"]).name for item in baseline
           if sha(Path(item["Path"])).upper() != item["Hash"].upper()]
assert set(changed) == allowed, changed
previous = (record / "before/App/app_config.h").read_text(encoding="utf-8-sig")
current = (project / "App/app_config.h").read_text(encoding="utf-8-sig")
for line in previous.splitlines():
    if line.startswith("#define ") and not line.startswith("#define APP_FEEDER_VERSION "):
        assert line in current, line
old_controller = (record / "before/App/feeder_controller.c").read_text(encoding="utf-8-sig")
new_controller = (project / "App/feeder_controller.c").read_text(encoding="utf-8-sig")
# Motion control and all default parameters are byte-for-byte equivalent text.
for name, following in (("void Feeder_Init", "uint8_t Feeder_SetConfig"),
                        ("int16_t Feeder_Step", None)):
    def section(text):
        part = text[text.index(name):]
        return part[:part.index(following)] if following else part
    assert section(old_controller) == section(new_controller), name
for formula in ("counts_bound = (rpm_bound + 120.0f) * (float)gap *",
                "((float)FEEDER_ENCODER_COUNTS / 60000.0f) + 32.0f;",
                "expected_counts >= 4064.0f", "absolute((float)delta) > counts_bound"):
    assert formula in old_controller and formula in new_controller, formula
report_file = project / "Tests/selftest_results.json"
report = json.loads(report_file.read_text(encoding="utf-8"))
assert report["success"] and len(report["results"]) == 68
for group in report["results"]:
    assert not group["failures"] and group["finished"] and group["returned"], group["name"]
    for name, expected in group["source_sha256"].items():
        assert sha(project / name) == expected, name
build = json.loads((record / "build_validation.json").read_text(encoding="utf-8"))
assert build["success"] and build["sources"] == 69
for name, expected in build["source_sha256"].items():
    assert sha(root / name) == expected, name
eide = (project / "MDK-ARM/.eide/eide.yml").read_text(encoding="utf-8-sig")
for name in ("controller", "backend", "console"):
    assert eide.count("path: ../App/feeder_" + name + ".c") == 1
(record / "软件自检.json").write_bytes(report_file.read_bytes())
result = dict(success=True, version="C610_POSITION_DIAG_V1B", groups=len(report["results"]),
    checks=sum(g["checks"] for g in report["results"]), failures=0, build_sources=69,
    changed_app_files=sorted(changed), unchanged_app_files=len(baseline) - len(changed),
    original_config_defines_preserved_except_feeder_version=True,
    defaults_motion_and_jump_thresholds_preserved=True, output_step_deg=40, manual_calibration_deg=90,
    static_stack_bytes_plus_unknown=build["static_stack_bytes_plus_unknown"],
    hardware_tested=False, flashed=False, hex_generated=False)
(record / "最终核验.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps(result, ensure_ascii=False))
