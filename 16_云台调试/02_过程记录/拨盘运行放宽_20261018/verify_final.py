"""Verify V1D source protection and the offline build/regression artifacts."""
from pathlib import Path
import hashlib
import json

record = Path(__file__).resolve().parent
root = record.parents[1]
project = root / "Output/Gimbal_Control"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


baseline = json.loads((record / "修改前App哈希.json").read_text(encoding="utf-8-sig"))
changed = [str(Path(item["Path"]).relative_to(project)).replace("\\", "/")
           for item in baseline if sha(Path(item["Path"])).upper() != item["Hash"].upper()]
expected = {"App/app_config.h", "App/feeder_controller.c", "App/feeder_controller.h",
            "App/feeder_backend.c", "App/feeder_backend.h", "App/feeder_console.c"}
assert set(changed) == expected, changed
old_config = (record / "before/App/app_config.h").read_bytes()
new_config = (project / "App/app_config.h").read_bytes()
assert new_config.replace(b"C610_POSITION_TUNE_V1D", b"C610_POSITION_CAN2_V1C") == old_config
build = json.loads((record / "build_validation.json").read_text(encoding="utf-8"))
assert build["success"] and build["sources"] == 69
for name, expected_hash in build["source_sha256"].items():
    assert sha(root / name) == expected_hash, name
report_path = project / "Tests/selftest_results_selected.json"
report = json.loads(report_path.read_text(encoding="utf-8"))
assert report["success"] and len(report["results"]) == 11
for group in report["results"]:
    assert group["finished"] and group["returned"] and not group["failures"], group["name"]
    for name, expected_hash in group["source_sha256"].items():
        assert sha(project / name) == expected_hash, name
# CAN1/CAN2, interrupts, HAL and RTOS sources must stay identical to verified V1C.
v1c = json.loads((record.parent / "CAN2自动重发修复_20261018/build_validation.json").read_text(encoding="utf-8"))
protected_other_sources = 0
for name, expected_hash in v1c["source_sha256"].items():
    if "/App/" not in name.replace("\\", "/"):
        assert sha(root / name) == expected_hash, name
        protected_other_sources += 1
panel = json.loads((root / "Output/VOFA_Feeder_20261017/validation.json").read_text(encoding="utf-8"))
assert panel["channels"] == 39 and panel["tabs"] == 3
assert panel["native_import_verified"] is False and panel["hardware_used"] is False
(record / "软件自检.json").write_bytes(report_path.read_bytes())
result = dict(success=True, version="C610_POSITION_TUNE_V1D", app_changes=changed,
    other_app_files_unchanged=len(baseline) - len(changed),
    non_app_sources_unchanged=protected_other_sources,
    regression_groups=len(report["results"]), regression_checks=sum(g["checks"] for g in report["results"]),
    failures=0, build_sources=69, vofa_channels=39, hardware_used=False, flashed=False, hex_generated=False,
    static_stack_bytes_plus_unknown=build["static_stack_bytes_plus_unknown"])
(record / "最终核验.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps(result, ensure_ascii=False))
