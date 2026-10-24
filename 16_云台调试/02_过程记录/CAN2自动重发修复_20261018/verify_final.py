from pathlib import Path
import hashlib
import json

record = Path(__file__).resolve().parent
root = record.parents[1]
project = root / "Output/Gimbal_Control"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


register = json.loads((record / "register_validation.json").read_text(encoding="utf-8"))
assert register["success"] and register["register_fixture"]["checks"] == 48
build = json.loads((record / "build_validation.json").read_text(encoding="utf-8"))
assert build["success"] and build["sources"] == 69
regression_file = project / "Tests/selftest_results_selected.json"
report = json.loads(regression_file.read_text(encoding="utf-8"))
assert report["success"] and len(report["results"]) == 11
for group in report["results"]:
    assert not group["failures"] and group["finished"] and group["returned"], group["name"]
    for name, expected in group["source_sha256"].items():
        assert sha(project / name) == expected, name
for document in (register, build):
    for name, expected in document["source_sha256"].items():
        assert sha(root / name) == expected, name
baseline = json.loads((record / "修改前哈希.json").read_text(encoding="utf-8-sig"))
changed = [str(Path(item["Path"]).relative_to(project)).replace("\\", "/") for item in baseline
           if sha(Path(item["Path"])).upper() != item["Hash"].upper()]
assert set(changed) == {"Core/Src/can.c", "Gimbal_Control.ioc", "App/app_config.h"}, changed
(record / "软件自检.json").write_bytes(regression_file.read_bytes())
result = dict(success=True, version="C610_POSITION_CAN2_V1C", production_changes=changed,
    register_checks=48, regression_groups=len(report["results"]),
    regression_checks=sum(g["checks"] for g in report["results"]), failures=0,
    build_sources=69, can1_auto_retransmission=True, can2_auto_retransmission=True,
    other_tracked_production_files_unchanged=len(baseline)-len(changed),
    static_stack_bytes_plus_unknown=build["static_stack_bytes_plus_unknown"],
    hardware_verified=False, flashed=False, hex_generated=False)
(record / "最终核验.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps(result, ensure_ascii=False))
