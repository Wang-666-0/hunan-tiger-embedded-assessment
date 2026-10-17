from pathlib import Path
import hashlib, json, re

record = Path(__file__).resolve().parent
workspace = record.parents[1]
project = workspace / "Output/Gimbal_Control"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
before = json.loads((record / "修改前哈希.json").read_text(encoding="utf-8-sig"))
changed = {n for n, h in before.items() if sha(workspace / n) != h}
expected = {"App/app_config.h", "App/app_console.c", "Core/Inc/usart.h", "Core/Src/usart.c",
    "Core/Inc/stm32f4xx_it.h", "Core/Src/stm32f4xx_it.c", "Tests/mocks/usart.h",
    "Tests/run_selftests.py", "Tests/test_pitch_console.c", "Tests/test_pitch_telemetry.c"}
assert changed == {"Output/Gimbal_Control/" + n for n in expected}, changed
tests = json.loads((project / "Tests/selftest_results.json").read_text(encoding="utf-8"))
assert tests["success"] and len(tests["results"]) == 54
for group in tests["results"]:
    assert group["finished"] and group["returned"] and not group["failures"], group["name"]
    for name, expected_hash in group["source_sha256"].items():
        assert sha(project / name) == expected_hash, name
dma = next(g for g in tests["results"] if g["name"] == "pitch_console_dma")
assert dma["compile_defines"] == ["-DAPP_CONSOLE_RX_DMA_ENABLED=1"]
config = (project / "App/app_config.h").read_text(encoding="utf-8-sig")
old = (record / "修改前备份/Output/Gimbal_Control/App/app_config.h").read_text(encoding="utf-8-sig")
assert config == old.replace('"DM3507_CASCADE_V1A"', '"DM3507_CASCADE_V1B"').replace(
    "#define APP_CONSOLE_RX_QUEUE_LENGTH 128U", "#define APP_CONSOLE_RX_QUEUE_LENGTH 128U\n"
    "#ifndef APP_CONSOLE_RX_DMA_ENABLED\n#define APP_CONSOLE_RX_DMA_ENABLED 1U\n#endif\n"
    "#define APP_CONSOLE_RX_DMA_BYTES 128U")
build = (record / "完整构建.log").read_text(encoding="utf-8-sig")
assert "build successfully" in build and not re.search(r"(?:Warning|Error):", build)
graph = (project / "MDK-ARM/build/Gimbal_Control/Gimbal_Control.htm").read_text(encoding="utf-8")
stacks = {task: int(re.search(r">" + task + r"</STRONG>.*?Max Depth = (\d+)", graph, re.S).group(1))
    for task in ("ConsoleTask", "ControlTask", "CanTask")}
assert stacks["ConsoleTask"] < 2500 and stacks["ControlTask"] < 2500 and stacks["CanTask"] < 1000
result = {"success": True, "version": "DM3507_CASCADE_V1B", "hardware_verified": False,
    "groups": len(tests["results"]), "checks": sum(g["checks"] for g in tests["results"]),
    "dma_transport_checks": dma["checks"], "baudrate": 921600,
    "rx_transport": "USART1 DMA2_Stream2 channel4, normal 128-byte ReceiveToIdle, HT disabled",
    "old_transport_fixture_groups": 53, "production_dma_fixture_groups": 1,
    "build_size": re.search(r"Program Size: (Code=\d+ RO-data=\d+ RW-data=\d+ ZI-data=\d+)", build).group(1),
    "known_stack_bytes": stacks, "runtime_watermark_measured": False,
    "changed_preexisting_files": sorted(changed), "new_test": "Tests/test_pitch_console_dma.c",
    "protected_unchanged_sha256": {n: h for n, h in before.items() if n not in changed},
    "current_changed_sha256": {n: sha(workspace / n) for n in changed},
    "new_test_sha256": sha(project / "Tests/test_pitch_console_dma.c"),
    "test_report_sha256": sha(project / "Tests/selftest_results.json"),
    "build_log_sha256": sha(record / "完整构建.log"),
    "unresolved": "Actual UART error and brief-disable cause require new hardware logs; software checks are not physical timing tests."}
(record / "selftest_results.json").write_bytes((project / "Tests/selftest_results.json").read_bytes())
(record / "final_verification.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({k: v for k, v in result.items() if "sha256" not in k}, ensure_ascii=False))
