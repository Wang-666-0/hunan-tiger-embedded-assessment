from pathlib import Path
import hashlib, json, re

record = Path(__file__).resolve().parent
workspace = record.parents[1]
project = workspace / "Output/Gimbal_Control"
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
before = json.loads((record / "修改前哈希.json").read_text(encoding="utf-8"))
changed = {n for n, h in before.items() if sha(workspace / n) != h}
expected = {"App/app_config.h", "App/app_console.c", "App/remote_input.c", "App/remote_input.h",
    "Core/Inc/usart.h", "Core/Src/usart.c", "Core/Inc/stm32f4xx_it.h", "Core/Src/stm32f4xx_it.c",
    "Tests/mocks/usart.h", "Tests/run_selftests.py", "Tests/test_app_console.c",
    "Tests/test_pitch_console.c", "Tests/test_pitch_console_dma.c"}
assert changed == {"Output/Gimbal_Control/" + n for n in expected}, changed
tests = json.loads((project / "Tests/selftest_results.json").read_text(encoding="utf-8"))
assert tests["success"] and len(tests["results"]) == 56
for group in tests["results"]:
    assert group["finished"] and group["returned"] and not group["failures"], group["name"]
    for name, expected_hash in group["source_sha256"].items():
        assert sha(project / name) == expected_hash, name
groups = {g["name"]: g for g in tests["results"]}
assert groups["pitch_dual_uart_dma"]["compile_defines"] == [
    "-DAPP_CONSOLE_RX_DMA_ENABLED=1", "-DAPP_REMOTE_RX_DMA_ENABLED=1"]
config = (project / "App/app_config.h").read_text(encoding="utf-8-sig")
old = (record / "修改前备份/Output/Gimbal_Control/App/app_config.h").read_text(encoding="utf-8-sig")
assert config == old.replace('"DM3507_CASCADE_V1B"', '"DM3507_CASCADE_V1C"').replace(
    "#define REMOTE_RX_QUEUE_LENGTH 256U", "#define REMOTE_RX_QUEUE_LENGTH 256U\n"
    "#ifndef APP_REMOTE_RX_DMA_ENABLED\n#define APP_REMOTE_RX_DMA_ENABLED 1U\n#endif\n"
    "#define APP_REMOTE_RX_DMA_BYTES 64U")
usart = (project / "Core/Src/usart.c").read_text(encoding="utf-8-sig")
assert "hdma_uart4_rx.Instance = DMA1_Stream2" in usart
assert "hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4" in usart
assert "hdma_uart4_rx.Init.Mode = DMA_NORMAL" in usart
assert "__HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT)" in usart
assert "huart4.Init.BaudRate = 115200" in usart
assert "HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0)" in usart
console = (project / "App/app_console.c").read_text(encoding="utf-8-sig")
assert console.count("void HAL_UARTEx_RxEventCallback") == 1
assert "RemoteInput_RxEvent(huart, size)" in console
assert "R|RX|EPOCH|" in console and '"REMOTE RX"' in console
build = (record / "完整构建.log").read_text(encoding="utf-8-sig")
assert "build successfully" in build and not re.search(r"(?:Warning|Error):", build)
graph = (project / "MDK-ARM/build/Gimbal_Control/Gimbal_Control.htm").read_text(encoding="utf-8")
stacks = {task: int(re.search(r">" + task + r"</STRONG>.*?Max Depth = (\d+)", graph, re.S).group(1))
    for task in ("ConsoleTask", "ControlTask", "CanTask", "RemoteTask")}
assert stacks["ConsoleTask"] < 2500 and stacks["ControlTask"] < 2500 and stacks["CanTask"] < 1000
result = {"success": True, "version": "DM3507_CASCADE_V1C", "hardware_verified": False,
    "groups": len(tests["results"]), "checks": sum(g["checks"] for g in tests["results"]),
    "uart4_checks": groups["pitch_remote_dma"]["checks"],
    "dual_uart_checks": groups["pitch_dual_uart_dma"]["checks"],
    "uart4_transport": "DMA1_Stream2 channel4, normal 64-byte ReceiveToIdle, HT disabled, IRQ priority 5",
    "known_stack_bytes": stacks, "runtime_watermark_measured": False,
    "build_size": re.search(r"Program Size: (Code=\d+ RO-data=\d+ RW-data=\d+ ZI-data=\d+)", build).group(1),
    "changed_preexisting_files": sorted(changed),
    "protected_unchanged_sha256": {n: h for n, h in before.items() if n not in changed},
    "current_changed_sha256": {n: sha(workspace / n) for n in changed},
    "new_test_sha256": sha(project / "Tests/test_remote_input_dma.c"),
    "test_report_sha256": sha(project / "Tests/selftest_results.json"),
    "build_log_sha256": sha(record / "完整构建.log"),
    "unresolved": "V1B GUARD confirms UART4 input fault, not a specific ORE/FE/NE cause. Physical stability and first step require user validation."}
(record / "selftest_results.json").write_bytes((project / "Tests/selftest_results.json").read_bytes())
(record / "final_verification.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(json.dumps({k: v for k, v in result.items() if "sha256" not in k}, ensure_ascii=False))
