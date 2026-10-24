"""Compare this cleanup against the saved working-tree baseline, not Git HEAD."""
from pathlib import Path
import difflib
import hashlib
import json
import re

audit = Path(__file__).resolve().parent
workspace = audit.parent.parent
project = workspace / "Output/Gimbal_Control"
baseline = json.loads((audit / "baseline_selftests.json").read_text(encoding="utf-8"))
after = json.loads((project / "Tests/selftest_results.json").read_text(encoding="utf-8"))
old_groups = {item["name"]: item for item in baseline["results"]}
new_groups = {item["name"]: item for item in after["results"]}
assert old_groups.keys() == new_groups.keys()
assert baseline["success"] and after["success"]
assert all(item["finished"] == 1 and item["returned"] and item["failures"] == 0
           for item in after["results"])
assert all(old_groups[name]["compile_defines"] == item["compile_defines"]
           for name, item in new_groups.items())

def source_hashes(report):
    hashes = {}
    for group in report["results"]:
        for path, digest in group["source_sha256"].items():
            assert path not in hashes or hashes[path] == digest, path
            hashes[path] = digest
    return hashes

old_hashes, new_hashes = source_hashes(baseline), source_hashes(after)
for path, digest in new_hashes.items():
    assert hashlib.sha256((project / path).read_bytes()).hexdigest() == digest, path

critical = ["App/app_config.h", "App/yaw_controller.c", "App/yaw_controller.h",
            "App/yaw_cascade.c", "App/yaw_cascade.h", "App/yaw_step.c", "App/yaw_step.h",
            "App/pitch_controller.h", "App/pitch_step.c", "App/pitch_step.h",
            "App/can_bus.c", "App/can_bus.h", "App/gimbal_startup.c",
            "App/safety_manager.c", "App/app_logic.c", "App/target_manager.c",
            "App/motor_backend.c", "App/flywheel_jog.c", "App/flywheel_speed.c",
            "App/feeder_backend.c", "App/feeder_controller.c"]
assert all(old_hashes[path] == new_hashes[path] for path in critical)

wire_fields = ("telemetry_max_bytes", "telemetry_max_wire_us")
wire = {name: {key: item[key] for key in wire_fields if key in item}
        for name, item in new_groups.items() if "telemetry_max_bytes" in item}
assert all(all(old_groups[name][key] == value for key, value in values.items())
           for name, values in wire.items())

diff = []
changes = []
for saved in sorted((audit / "before/Output/Gimbal_Control").rglob("*")):
    if not saved.is_file():
        continue
    relative = saved.relative_to(audit / "before")
    current = workspace / relative
    if saved.read_bytes() == current.read_bytes():
        continue
    if current.suffix in (".c", ".h", ".md"):
        old = saved.read_text(encoding="utf-8-sig").splitlines(keepends=True)
        new = current.read_text(encoding="utf-8-sig").splitlines(keepends=True)
        diff.extend(difflib.unified_diff(old, new,
                    fromfile="before/" + relative.as_posix(),
                    tofile="after/" + relative.as_posix()))
        changes.append(relative.as_posix())
new_header = project / "App/console_parse.h"
relative = new_header.relative_to(workspace).as_posix()
diff.extend(difflib.unified_diff([], new_header.read_text(encoding="utf-8").splitlines(True),
            fromfile="/dev/null", tofile="after/" + relative))
changes.append(relative)
(audit / "cleanup_only.diff").write_text("".join(diff), encoding="utf-8")

def rom_size(path):
    text = path.read_text(encoding="utf-8", errors="replace")
    return int(re.search(r"Total ROM Size .*?\s(\d+)\s+\(", text).group(1))

result = {
    "baseline_recorded_at": baseline["recorded_at"],
    "after_recorded_at": after["recorded_at"],
    "groups": len(new_groups),
    "baseline_checks": sum(item["checks"] for item in old_groups.values()),
    "after_checks": sum(item["checks"] for item in new_groups.values()),
    "failures": 0,
    "group_names_and_compile_defines_unchanged": True,
    "critical_sources_unchanged": critical,
    "telemetry_wire_metrics_unchanged": wire,
    "check_count_deltas": {name: item["checks"] - old_groups[name]["checks"]
                           for name, item in new_groups.items()
                           if item["checks"] != old_groups[name]["checks"]},
    "baseline_o0_rom_bytes": rom_size(audit / "baseline_o0.map"),
    "after_o0_rom_bytes": rom_size(audit / "after_o0.map"),
    "cleanup_files": changes,
    "note": "HAL/RTOS mocked. No flash, hardware motion, or measured step-performance improvement.",
}
(audit / "comparison.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({key: value for key, value in result.items()
                  if key not in ("critical_sources_unchanged", "cleanup_files")}, ensure_ascii=False))
