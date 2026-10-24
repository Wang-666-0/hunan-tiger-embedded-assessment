from pathlib import Path
import hashlib
import json

record = Path(__file__).resolve().parent
root = record.parents[1]
project = root / 'Output/Gimbal_Control'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


baseline = json.loads((record / '修改前App哈希.json').read_text(encoding='utf-8-sig'))
changed = [str(Path(item['Path']).relative_to(project)).replace('\\', '/')
           for item in baseline if sha(Path(item['Path'])).upper() != item['Hash'].upper()]
assert set(changed) == {'App/app_config.h', 'App/feeder_controller.c', 'App/feeder_controller.h',
                        'App/feeder_console.c', 'App/pitch_console.c'}, changed
assert (project / 'App/app_config.h').read_bytes().replace(
    b'C610_FEEDBACK_TIME_V1F', b'C610_ASSESSMENT_20HZ_V1E') == (record / 'before/App/app_config.h').read_bytes()
pitch = (project / 'App/pitch_console.c').read_bytes()
pitch = pitch.replace(b'"PITCH", "WHEELS", "FEEDER"', b'"PITCH", "WHEELS"')
pitch = pitch.replace(b'trace->source < sizeof(sources) / sizeof(sources[0])',
                      b'trace->source <= APP_STOP_SOURCE_WHEELS')
assert pitch == (record / 'before/App/pitch_console.c').read_bytes(), 'Pitch change exceeds source label'
controller = (project / 'App/feeder_controller.c').read_bytes()
old_controller = (record / 'before/App/feeder_controller.c').read_bytes()
assert controller[controller.index(b'uint8_t Feeder_Clear'):] == old_controller[old_controller.index(b'uint8_t Feeder_Clear'):]
assert controller[:controller.index(b'uint8_t Feeder_OnFrame')] == old_controller[:old_controller.index(b'uint8_t Feeder_OnFrame')]
build = json.loads((record / 'build_validation.json').read_text(encoding='utf-8'))
assert build['success'] and build['sources'] == 69
for name, expected in build['source_sha256'].items():
    assert sha(root / name) == expected, name
old_build = json.loads((record.parent / '拨盘考核收尾_20261018/build_validation.json').read_text(encoding='utf-8'))
other_sources = 0
for name, expected in old_build['source_sha256'].items():
    if '/App/' not in name.replace('\\', '/'):
        assert sha(root / name) == expected, name
        other_sources += 1
report_path = project / 'Tests/selftest_results_selected.json'
report = json.loads(report_path.read_text(encoding='utf-8'))
required = {'feeder_controller', 'feeder_backend', 'feeder_stream', 'feeder_can', 'feeder_wheels',
            'assessment_remote', 'assessment_wheels', 'pitch_console', 'pitch_gimbal_startup',
            'pitch_integration', 'pitch_console_dma'}
assert report['success'] and {g['name'] for g in report['results']} == required
for group in report['results']:
    assert group['finished'] and group['returned'] and not group['failures'], group['name']
    for name, expected in group['source_sha256'].items():
        assert sha(project / name) == expected, name
panel = json.loads((root / 'Output/VOFA_Feeder_20261017/validation.json').read_text(encoding='utf-8'))
assert panel['channels'] == 39 and panel['tabs'] == 3
assert panel['channel_map']['I36'] == 'last_motion_ms'
(record / '软件回归.json').write_bytes(report_path.read_bytes())
result = dict(success=True, version='C610_FEEDBACK_TIME_V1F', app_changes=changed,
    protected_app_files=len(baseline)-len(changed), other_sources_unchanged=other_sources,
    groups=len(report['results']), checks=sum(g['checks'] for g in report['results']), failures=0,
    build_sources=69, vofa_channels=39, normal_startup_requires_serial=False,
    hardware_verified=False, physical_20hz_verified=False, flashed=False, hex_generated=False,
    static_stack_bytes_plus_unknown=build['static_stack_bytes_plus_unknown'])
(record / '最终核验.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(json.dumps(result, ensure_ascii=False))
