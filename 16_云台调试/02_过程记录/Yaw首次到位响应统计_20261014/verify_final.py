"""核对V2D构建、正式C自检哈希和本轮修改范围，不连接硬件。"""
from pathlib import Path
import hashlib
import json
import re
import xml.etree.ElementTree as ET

record = Path(__file__).resolve().parent
project = record.parents[1] / "Output/Gimbal_Control"
app = project / "App"
digest = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
report = json.loads((project / "Tests/selftest_results.json").read_text(encoding="utf-8"))
assert report["success"] and len(report["results"]) == 43
for group in report["results"]:
    assert group["finished"] and group["returned"] and not group["failures"], group["name"]
    for name, expected in group["source_sha256"].items():
        assert digest(project / name) == expected, name
before = json.loads((record / "修改前App哈希.json").read_text(encoding="utf-8-sig"))
changed = {"README.md", "app_config.h", "yaw_step.c", "yaw_step.h"}
protected = {}
for item in before:
    name = Path(item["Path"]).name
    if name not in changed:
        assert digest(app / name) == item["Hash"].lower(), name
        protected[name] = item["Hash"].lower()
old = (record / "修改前备份/App/app_config.h").read_text(encoding="utf-8")
new = (app / "app_config.h").read_text(encoding="utf-8")
assert old.replace("GL60II_CASCADE_STEP30_V2C", "GL60II_CASCADE_STEP30_V2D") == new
assert "#define APP_TELEMETRY_PERIOD_MS 5U" in new
assert "#define APP_CONSOLE_BAUDRATE 921600U" in new
for item in json.loads((record / "修改前串口哈希.json").read_text(encoding="utf-8-sig")):
    assert digest(Path(item["Path"])) == item["Hash"].lower(), item["Path"]
tree = ET.parse(project / "MDK-ARM/Gimbal_Control.uvprojx")
keil = sorted(n.text.replace("\\", "/") for n in tree.findall("./Targets/Target/Groups/Group/Files/File/FilePath"))
eide = (project / "MDK-ARM/.eide/eide.yml").read_text(encoding="utf-8")
sources = sorted(re.findall(r"- path: ([^\r\n]+\.(?:c|s))\s*$", eide, re.M))
params = json.loads((record / "builder_AC5_O0.params").read_text(encoding="utf-8"))
assert keil == sources == sorted(params["sourceList"]) and len(keil) == 60
log = (record / "完整构建.log").read_text(encoding="utf-8-sig")
assert "build successfully" in log and "Warning:" not in log and "Error:" not in log
size = re.search(r"Program Size: (Code=\d+ RO-data=\d+ RW-data=\d+ ZI-data=\d+)", log)
assert size
result = {
    "version": "GL60II_CASCADE_STEP30_V2D", "success": True,
    "hardware": "No flashing or physical test by agent",
    "response_definition": "first feedback inside target +/-0.2 deg; no dwell; never overwritten",
    "settle_hold_ms": 50, "telemetry_period_ms": 5, "baudrate": 921600,
    "groups": len(report["results"]), "checks": sum(g["checks"] for g in report["results"]),
    "build_size": size.group(1), "source_count": len(keil),
    "protected_app_files": protected,
    "app_sha256": {p.name: digest(p) for p in sorted(app.iterdir()) if p.suffix in (".c", ".h")},
    "build_sha256": digest(record / "完整构建.log"),
    "test_report_sha256": digest(project / "Tests/selftest_results.json"),
}
(record / "selftest_results.json").write_bytes((project / "Tests/selftest_results.json").read_bytes())
(record / "final_verification.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({k: v for k, v in result.items() if k not in ("protected_app_files", "app_sha256")}, ensure_ascii=False))
