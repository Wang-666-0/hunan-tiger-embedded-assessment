from pathlib import Path
import re,json
REC=Path(__file__).resolve().parent;FW=REC.parents[1]/'Output/Gimbal_Control';APP=FW/'App';TESTS=FW/'Tests'
helpers=(REC/'refactor_app.py').read_text(encoding='utf-8').split('texts={p.name:')[0]
ns={ '__file__':str(REC/'refactor_app.py') };exec(helpers,ns)
drop=ns['drop_function']; span=ns['function_span'];select=ns['select_branches']
old=(REC/'before/Tests/run_selftests.py').read_text(encoding='utf-8')
old=old.replace('import source_layout\n','')
a=old.index('    # Unit fixtures retain');b=old.index('    groups = {',a)
old=old[:a]+old[b:]
a=old.index('    if args.groups:')
filter_code='''    # Archive only groups for removed profiles; keep current behavior tests.
    retired = {"logic", "legacy_remote", "dm_diag", "vesc_diag", "can_bus", "can_bus_capture",
        "diagnostics", "console", "console_jog", "console_observe", "flywheel_jog", "motor_jog",
        "flywheel_jog_pulse", "motor_jog_pulse", "flywheel_speed", "flywheel_speed_jog", "motor_speed",
        "speed_1000", "dual_jog", "motor_dual", "can_bus_1000", "dual_jog_6000", "motor_dual_6000", "can_bus_6000",
        "yaw_controller_guarded", "yaw_backend_guarded", "yaw_can", "yaw_console", "yaw_console_backend",
        "yaw_console_backend_guarded", "yaw_cascade", "yaw_cascade_console", "yaw_feedforward", "yaw_shape",
        "yaw_shape_console", "yaw_feedforward_console"}
    groups = {name: sources for name, sources in groups.items() if name not in retired}
    files = {p.name: p for p in app.rglob("*.c")}
    removed_sources = {"dm_diag.c", "vesc_diag.c", "app_diagnostics.c"}
    groups = {name: list(dict.fromkeys(files[p.name] if p.parent == app else p
              for p in sources if p.name not in removed_sources)) for name,sources in groups.items()}
'''
old=old[:a]+filter_code+old[a:]
old=old.replace('        compile_sources, production_sources = source_layout.resolve(sources, app, sections)',
                '        compile_sources = production_sources = sources')
old=old.replace('sorted(app.glob("*.h"))','sorted(app.rglob("*.h"))')
old=old.replace('root / "source_layout.py", root / "app_source_map.json"','root / "mocks/bus_contention.h"')
a=old.index('        # 历史26组按原映射回归');b=old.index('        # Preserve the original per-byte',a)
old=old[:a]+'''        if not name.startswith(("pitch_", "feeder_")):
            profile_flags += ["-DAPP_PITCH_ENABLED=0"]
        if name == "speed_6000":
            profile_flags += ["-DAPP_YAW_ENABLED=0"]
        profile_flags += ["-DAPP_FEEDER_ENABLED=" + ("1" if name.startswith("feeder_") else "0")]
        if not name.startswith("feeder_standalone"):
            profile_flags += ["-DAPP_FEEDER_STANDALONE_TEST_ENABLED=0"]
''' +old[b:]
old=old.replace('"-I", str(root / "mocks"), "-I", str(app), "-nostartfiles",',
                '"-I", str(root / "mocks"), "-I", str(app)] + [x for d in sorted(app.iterdir()) if d.is_dir() for x in ("-I", str(d))] + ["-nostartfiles",')
(TESTS/'run_selftests.py').write_text(old,encoding='utf-8',newline='\n')

# Remove the legacy diagnostic test body from the UART fixture, keeping the
# existing HAL mocks used by real telemetry/DMA integration tests.
p=TESTS/'test_app_console.c';s=p.read_text(encoding='utf-8')
for name in ['AppDiagnostics_Cancel','AppDiagnostics_Request','AppDiagnostics_TakeEvent','AppDiagnostics_GetPassive','expect_rejected','event_init','test_run']:
    s=drop(s,name)
s=s.replace('AppDiagnostics_Cancel();','')
s=re.sub(r'^#include "(?:app_diagnostics|dm_diag|vesc_diag)\.h"\n','',s,flags=re.M)
s=re.sub(r'^static (?:DiagKind|DiagEvent)[^\n]*\n','',s,flags=re.M)
s=re.sub(r'^static uint8_t (?:requested_node|event_read)[^\n]*\n','',s,flags=re.M)
s=s.replace(', request_allowed = 1U','').replace('requests, stops, cancels, rearms','stops, rearms')
p.write_text(s,encoding='utf-8',newline='\n')

# Diagnostic transactions are gone. Preserve mailbox-pressure and unrelated
# traffic isolation tests using direct HAL fixture traffic, not a firmware
# query API. The helper is strictly in Tests/mocks, never a firmware source.
(TESTS/'mocks/bus_contention.h').write_text('''#ifndef TEST_BUS_CONTENTION_H
#define TEST_BUS_CONTENTION_H
static uint32_t fixture_other[2];
static inline HAL_StatusTypeDef Fixture_OtherTx(CAN_HandleTypeDef *bus)
{
    CAN_TxHeaderTypeDef header = {0}; uint8_t data[8] = {0}; uint32_t mailbox;
    HAL_StatusTypeDef result;
    header.IDE = CAN_ID_STD; header.StdId = 0x123U; header.RTR = CAN_RTR_DATA; header.DLC = 8U;
    result = HAL_CAN_AddTxMessage(bus, &header, data, &mailbox);
    if (result == HAL_OK) { fixture_other[bus == &hcan1 ? 0U : 1U] = mailbox; }
    return result;
}
static inline void Fixture_CancelOther(void)
{
    if (fixture_other[0]) { (void)HAL_CAN_AbortTxRequest(&hcan1, fixture_other[0]); }
    if (fixture_other[1]) { (void)HAL_CAN_AbortTxRequest(&hcan2, fixture_other[1]); }
    fixture_other[0] = fixture_other[1] = 0U;
}
#endif
''',encoding='utf-8')
for name in ['test_feeder_can.c','test_pitch_can.c','test_yaw_can.c']:
    p=TESTS/name;s=p.read_text(encoding='utf-8')
    s=re.sub(r'^#include "(?:dm_diag|vesc_diag|app_diagnostics)\.h"\n','',s,flags=re.M)
    # Insert after the fixture's two bus instances, before helper functions.
    m=re.search(r'(?m)^CAN_HandleTypeDef hcan1[^\n]*\n',s);assert m,name
    s=s[:m.end()]+'#include "bus_contention.h"\n'+s[m.end():]
    s=re.sub(r'CanBus_ReadDm\([^)]*\)', 'Fixture_OtherTx(&hcan1)',s)
    s=re.sub(r'CanBus_ReadVesc\([^)]*\)', 'Fixture_OtherTx(&hcan2)',s)
    s=s.replace('CanBus_CancelReads()', 'Fixture_CancelOther()').replace('diag_box','other_box')
    for fn in ['CompleteAll','ClearPending']:
        if re.search(r'\b'+fn+r'\s*\(void\)',s):
            a,b=span(s,fn);i=s.index('{',a)
            s=s[:i+1]+'\n    fixture_other[0] = fixture_other[1] = 0U;'+s[i+1:]
    p.write_text(s,encoding='utf-8',newline='\n')

# The fixed-mode identifiers remain read-only metadata for existing tooling.
p=APP/'app_config.h';s=p.read_text(encoding='utf-8')
s=s.replace('#define APP_CONFIG_H\n','#define APP_CONFIG_H\n\n/* 当前正式模式标识；旧模式实现已归档，不再提供模式切换。 */\n#define APP_REMOTE_PROFILE_ASSESSMENT 1U\n#define APP_REMOTE_PROFILE_LEGACY 0U\n#define APP_FLYWHEEL_SPEED_ACTIVE 1U\n#define YAW_CONTROLLER_CASCADE_ENABLED 1U\n#define YAW_CASCADE_OUTPUT_LIMITS_ENABLED 0U\n')
p.write_text(s,encoding='utf-8',newline='\n')
# Keep the negative compatibility probe as a tiny deny-only API; the generic
# transmit implementation and every HAL-send branch of it have been removed.
p=APP/'drivers/can_bus.c';s=p.read_text(encoding='utf-8')+'\n/* 任意帧接口始终拒绝；仅保留兼容性负向检查，不提交 CAN。 */\nHAL_StatusTypeDef CanBus_Send(const CanFrame *frame) { (void)frame; return HAL_ERROR; }\n';p.write_text(s,encoding='utf-8')
p=APP/'drivers/can_bus.h';s=p.read_text(encoding='utf-8').replace('#endif\n','/* 兼容性拒绝入口；无任意报文发送实现。 */\nHAL_StatusTypeDef CanBus_Send(const CanFrame *frame);\n#endif\n');p.write_text(s,encoding='utf-8')

# Update samples for removed Yaw experimental state while checking every wire
# index. Pitch samples/expectations are intentionally unchanged.
p=TESTS/'test_yaw_telemetry.c';s=p.read_text(encoding='utf-8')
s=re.sub(r'\s*yaw_sample\.controller\.(?:ff|shape)\.[A-Za-z_]+\s*=\s*[^;]+;', '',s)
s=s.replace('columns[64] == 160 && columns[65] == 1 && columns[66] == 15','columns[64] == 0 && columns[65] == 0 && columns[66] == columns[1]')
s=s.replace('columns[67] > 351.55f && columns[67] < 351.58f && columns[68] == -3000','columns[67] == 0 && columns[68] == 0')
s=s.replace('columns[69] > 13.765f && columns[69] < 13.767f','columns[69] == columns[1] - columns[0]')
s=s.replace('columns[70] == 0.25f && columns[71] == 0.000002f','columns[70] == 0 && columns[71] == 0')
p.write_text(s,encoding='utf-8',newline='\n')
print('Tests now compile actual files in the five App subdirectories')
