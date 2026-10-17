"""Synchronize the existing EIDE/Keil manifests, then prepare an O0 local build.

No flashing, serial, CubeMX regeneration, or copying of the firmware project.
"""
from pathlib import Path
import json
import re
import xml.etree.ElementTree as ET

record = Path(__file__).resolve().parent
project = record.parents[1] / "Output/Gimbal_Control"
mdk = project / "MDK-ARM"
new_modules = ("dm3507", "pitch_controller", "pitch_step", "pitch_backend", "pitch_console")
eide_path = mdk / ".eide/eide.yml"
eide = eide_path.read_text(encoding="utf-8")
marker = "        - path: ../App/yaw_console.c\n"
assert marker in eide
new_entries = "".join(f"        - path: ../App/{name}.c\n" for name in new_modules
                      if f"../App/{name}.c" not in eide)
eide_path.write_text(eide.replace(marker, marker + new_entries), encoding="utf-8")

keil_path = mdk / "Gimbal_Control.uvprojx"
keil = keil_path.read_text(encoding="utf-8")
marker_match = re.search(r"(?m)^ *<File>\s*<FileName>yaw_console.c</FileName>.*?</File>\n", keil, re.S)
assert marker_match
entries = "".join(
    "              <File>\n"
    f"                <FileName>{name}.c</FileName>\n"
    "                <FileType>1</FileType>\n"
    f"                <FilePath>../App/{name}.c</FilePath>\n"
    "              </File>\n"
    for name in new_modules if f"../App/{name}.c" not in keil)
keil_path.write_text(keil[:marker_match.end()] + entries + keil[marker_match.end():], encoding="utf-8")
tree = ET.parse(keil_path)
sources = [entry.text.replace("\\", "/") for entry in
           tree.findall("./Targets/Target/Groups/Group/Files/File/FilePath")]
eide_sources = re.findall(r"- path: ([^\r\n]+\.(?:c|s))\s*$", eide_path.read_text(encoding="utf-8"), re.M)
assert sorted(sources) == sorted(eide_sources)
assert len(sources) == 65
for source in sources:
    assert (mdk / source).is_file(), source
baseline_params = record.parent / "Yaw右摇杆90度每秒_20261014/builder_AC5_O0.params"
params = json.loads(baseline_params.read_text(encoding="utf-8"))
params["sourceList"] = sources
params["options"]["c/cpp-compiler"]["optimization"] = "level-0"
params["rom"] = 524288
(record / "builder_AC5_O0.params").write_text(json.dumps(params, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(f"EIDE/Keil: {len(sources)} source entries, including all five Pitch modules")
