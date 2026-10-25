"""Select logical sections from the ten production C modules for existing tests.

Fixtures supply different HAL/RTOS and actuator mocks, so they still compile only
their required logical sections. No source is copied into this repository: every
test reads the marked sections from the current production file. The normal
EIDE/Keil build compiles all ten files directly, without this helper.
"""
from pathlib import Path
import json
import re


def materialize(app: Path, directory: Path):
    manifest_path = Path(__file__).with_name("app_source_map.json")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    directory.mkdir(parents=True, exist_ok=True)
    selected = {}
    for module, sections in manifest["modules"].items():
        production = app / module
        text = production.read_text(encoding="utf-8")
        for section in sections:
            pattern = (r"/\* APP_SECTION_BEGIN " + re.escape(section) + r" \*/\n"
                       r"([\s\S]*?)/\* APP_SECTION_END " + re.escape(section) + r" \*/")
            matches = re.findall(pattern, text)
            if len(matches) != 1 or section in selected:
                raise RuntimeError(f"Invalid production section: {module}/{section}")
            path = directory / section
            path.write_text(matches[0], encoding="utf-8", newline="\n")
            selected[section] = (path, production)
    return selected


def resolve(sources, app, selected):
    compiled, tracked = [], []
    for source in sources:
        if source.parent == app:
            extracted, production = selected[source.name]
            compiled.append(extracted)
            tracked.append(production)
        else:
            compiled.append(source)
            tracked.append(source)
    return compiled, list(dict.fromkeys(tracked))
