"""读取原工程与本轮证据，保存最终一致性核验；不编译或连接硬件。"""
from pathlib import Path
import hashlib
import json
import re
import xml.etree.ElementTree as ET

record = Path(__file__).resolve().parent
project = record.parents[1] / "Output/Gimbal_Control"
app = project / "App"
report = json.loads((project / "Tests/selftest_results.json").read_text(encoding="utf-8"))
digest = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
for group in report["results"]:
    assert group["finished"] == 1 and group["returned"] and not group["failures"], group["name"]
    for name, expected in group["source_sha256"].items():
        assert digest(project / name) == expected, name
assert report["success"]
before = json.loads((record / "修改前App哈希.json").read_text(encoding="utf-8-sig"))
before = {Path(item["Path"]).name: item["Hash"].lower() for item in before}
allowed = {"app_config.h", "app_console.c", "yaw_controller.c", "yaw_controller.h",
           "yaw_backend.c", "yaw_backend.h", "yaw_console.c", "yaw_step.h", "README.md"}
unchanged = {}
for name, expected in before.items():
    if name not in allowed:
        actual = digest(app / name)
        assert actual == expected, name
        unchanged[name] = actual
old_config = (record / "修改前备份/App/app_config.h").read_text(encoding="utf-8")
new_config = (app / "app_config.h").read_text(encoding="utf-8")
normalize = lambda txt: re.sub(r'#define APP_YAW_VERSION "[^"]+"', '#define APP_YAW_VERSION "VERSION"', txt)
assert normalize(old_config) == normalize(new_config)
tree = ET.parse(project / "MDK-ARM/Gimbal_Control.uvprojx")
keil = sorted(n.text.replace("\\", "/") for n in tree.findall("./Targets/Target/Groups/Group/Files/File/FilePath"))
eide = (project / "MDK-ARM/.eide/eide.yml").read_text(encoding="utf-8")
eide_sources = sorted(re.findall(r"- path: ([^\r\n]+\.(?:c|s))\s*$", eide, re.M))
params = json.loads((record / "builder_AC5_O0.params").read_text(encoding="utf-8"))
builder = sorted(s.replace("\\", "/") for s in params["sourceList"])
assert keil == eide_sources == builder and len(keil) == 60
assert "../App/yaw_cascade.c" in keil and params["defines"] == ["USE_HAL_DRIVER", "STM32F405xx"]
for source in app.glob("*.c"):
    assert source.with_suffix(".h").exists(), source.name
log = (record / "完整构建.log").read_text(encoding="utf-8-sig")
assert "build successfully" in log and "Warning:" not in log and "Error:" not in log
assert "Code=66780 RO-data=3344 RW-data=444 ZI-data=34420" in log
assert '"GL60II_CASCADE_STEP30_V2A"' in new_config
result = {
    "version": "GL60II_CASCADE_STEP30_V2A", "success": True,
    "physical_test": "V2A pending user EIDE rebuild and ST-Link; latest confirmed hardware V1H",
    "test_groups": len(report["results"]), "checks": sum(r["checks"] for r in report["results"]),
    "source_lists_match": True, "source_count": len(keil), "app_c_count": len(list(app.glob("*.c"))),
    "configuration_only_version_changed": True, "unchanged_app_files": unchanged,
    "app_sha256": {p.name: digest(p) for p in sorted(app.iterdir()) if p.suffix in (".c", ".h")},
    "build_log_sha256": digest(record / "完整构建.log"),
    "test_report_sha256": digest(project / "Tests/selftest_results.json"),
}
(record / "selftest_results.json").write_bytes((project / "Tests/selftest_results.json").read_bytes())
(record / "final_verification.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({k: v for k, v in result.items() if not k.endswith("sha256") and k != "unchanged_app_files"}, ensure_ascii=False))
