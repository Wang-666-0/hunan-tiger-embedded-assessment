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
            "App/feeder_backend.c", "App/feeder_backend.h", "App/feeder_console.c",
            "App/target_manager.c", "App/target_manager.h", "App/app_tasks.c"}
assert set(changed) == expected, changed
assert (project / "App/app_config.h").read_bytes().replace(
    b"C610_ASSESSMENT_20HZ_V1E", b"C610_POSITION_TUNE_V1D") == (record / "before/App/app_config.h").read_bytes()
new_tasks = (project / "App/app_tasks.c").read_bytes()
insertion = ("#if APP_FEEDER_ENABLED\n"
             "        if (GimbalStartup_OutputReady() && !AppConsole_IsDiagnosticMode()) {\n"
             "            (void)FeederBackend_PrepareRemote();\n"
             "        }\n#endif\n")
old_tasks_hash = next(item["Hash"].lower() for item in baseline if Path(item["Path"]).name == "app_tasks.c")
assert any(hashlib.sha256(new_tasks.replace(insertion.encode("ascii").replace(b"\n", newline), b"")).hexdigest()
           == old_tasks_hash for newline in (b"\n", b"\r\n")), "ControlTask changed beyond the preparation hook"
build = json.loads((record / "build_validation.json").read_text(encoding="utf-8"))
assert build["success"] and build["sources"] == 69
for name, expected_hash in build["source_sha256"].items():
    assert sha(root / name) == expected_hash, name
report_path = project / "Tests/selftest_results.json"
report = json.loads(report_path.read_text(encoding="utf-8"))
assert report["success"] and len(report["results"]) == 69
for group in report["results"]:
    assert group["finished"] and group["returned"] and not group["failures"], group["name"]
    for name, expected_hash in group["source_sha256"].items():
        assert sha(project / name) == expected_hash, name
v1d = json.loads((record.parent / "拨盘运行放宽_20261018/build_validation.json").read_text(encoding="utf-8"))
other_sources = 0
for name, expected_hash in v1d["source_sha256"].items():
    if "/App/" not in name.replace("\\", "/"):
        assert sha(root / name) == expected_hash, name
        other_sources += 1
panel = json.loads((root / "Output/VOFA_Feeder_20261017/validation.json").read_text(encoding="utf-8"))
assert panel["channels"] == 39 and panel["tabs"] == 3
assert panel["channel_map"]["I36"] == "last_motion_ms" and panel["native_import_verified"] is False
(record / "软件完整自检.json").write_bytes(report_path.read_bytes())
result = dict(success=True, version="C610_ASSESSMENT_20HZ_V1E", app_changes=changed,
    protected_app_files=len(baseline)-len(changed), other_sources_unchanged=other_sources,
    groups=len(report["results"]), checks=sum(g["checks"] for g in report["results"]), failures=0,
    build_sources=69, vofa_channels=39, normal_startup_requires_serial=False,
    hardware_verified=False, physical_20hz_verified=False, flashed=False, hex_generated=False,
    static_stack_bytes_plus_unknown=build["static_stack_bytes_plus_unknown"])
(record / "最终核验.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps(result, ensure_ascii=False))
