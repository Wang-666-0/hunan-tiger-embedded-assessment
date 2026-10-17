"""Offline AC5/O0 compile/link of the existing EIDE project; no HEX, flash or device access."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import hashlib
import json
import re
import subprocess

record = Path(__file__).resolve().parent
root = record.parents[1]
project = root / "Output/Gimbal_Control"
mdk = project / "MDK-ARM"
params = json.loads((mdk / "build/Gimbal_Control/builder.params").read_text(encoding="utf-8-sig"))
tools = Path(params["toolchainLocation"]) / "bin"
output = record / "build_check"
output.mkdir(exist_ok=True)
sources = list(params["sourceList"])
for source in ("feeder_controller", "feeder_backend", "feeder_console"):
    name = "../App/" + source + ".c"
    if name not in sources:
        sources.append(name)
headers = list((project / "App").glob("*.h"))
tracked = [(mdk / name).resolve() for name in sources] + headers
hashes = {str(path.relative_to(root)): hashlib.sha256(path.read_bytes()).hexdigest() for path in tracked}
includes = ["-I" + path for path in params["incDirs"]]
defines = ["-D" + value for value in params["defines"]]


def compile_one(item):
    index, name = item
    source = (mdk / name).resolve()
    obj = output / (f"{index:03d}_" + source.stem + ".o")
    if source.suffix.lower() == ".c":
        command = [str(tools / "armcc.exe"), "-c", "--apcs=interwork"] + includes + defines + [
            "--cpu", "Cortex-M4.fp", "--li", "--c99", "-O0", "--split_sections",
            "--diag_suppress=1", "--diag_suppress=1295", "-g", "-o", str(obj), str(source)]
    else:
        command = [str(tools / "armasm.exe"), "--apcs=interwork"] + includes + [
            "--cpu", "Cortex-M4.fp", "--li", "-g", "-o", str(obj), str(source)]
    result = subprocess.run(command, cwd=mdk, capture_output=True)
    message = (result.stdout + result.stderr).decode("utf-8", errors="replace")
    return name, obj, result.returncode, message


with ThreadPoolExecutor(max_workers=8) as pool:
    results = list(pool.map(compile_one, enumerate(sources)))
log = "\n".join(name + "\n" + message for name, _, _, message in results)
(record / "AC5_compile.log").write_text(log, encoding="utf-8")
failures = [(name, message) for name, _, code, message in results if code]
if failures:
    print(json.dumps(failures, ensure_ascii=False, indent=2))
    raise SystemExit(1)
scatter = output / "Gimbal_Control.sct"
scatter.write_text((mdk / "build/Gimbal_Control/Gimbal_Control.sct").read_text(), encoding="ascii")
binary = output / "Gimbal_Control.axf"
command = [str(tools / "armlink.exe"), "--cpu", "Cortex-M4.fp", "--scatter", str(scatter),
    "--strict", "--summary_stderr", "--info", "summarysizes", "--map", "--xref", "--callgraph",
    "--symbols", "--info", "sizes", "--info", "totals", "--info", "unused", "--info", "veneers",
    "--list", str(output / "Gimbal_Control.map")] + [str(obj) for _, obj, _, _ in results] + ["-o", str(binary)]
linked = subprocess.run(command, cwd=mdk, capture_output=True)
link_log = (linked.stdout + linked.stderr).decode("utf-8", errors="replace")
(record / "AC5_link.log").write_text(link_log, encoding="utf-8")
print(link_log)
assert not linked.returncode, "Link failed"
assert not re.search(r"(?:warning|error)\s*[:#]", log + link_log, re.I), "Compiler diagnostics"
for name, expected in hashes.items():
    assert hashlib.sha256((root / name).read_bytes()).hexdigest() == expected, name
data = binary.read_bytes()
for version in (b"C610_POSITION_40DEG_V1A", b"GL60II_CASCADE_SHAPE_V2K", b"DM3507_CASCADE_V1F"):
    assert version in data, version
html = binary.with_suffix(".htm").read_text(encoding="utf-8", errors="replace")
stacks = {}
for task, limit in (("ConsoleTask", 3072), ("ControlTask", 3072), ("CanTask", 1536), ("RemoteTask", 1536)):
    match = re.search(">" + task + r"</STRONG>.*?Max Depth = (\d+)", html, re.S)
    assert match and int(match[1]) < limit, task
    stacks[task] = int(match[1])
result = dict(success=True, sources=len(sources), compiler="ARM Compiler 5.06 / O0 / EIDE include paths",
    static_stack_bytes_plus_unknown=stacks, source_sha256=hashes,
    hardware_used=False, hex_generated=False, flashed=False)
(record / "build_validation.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({k:v for k,v in result.items() if k != "source_sha256"}, ensure_ascii=False))
