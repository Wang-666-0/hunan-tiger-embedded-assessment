"""Generate an independent VOFA+ feeder panel from an exported native v100 schema.

This only writes JSON artifacts. It never opens a serial port, imports a live
configuration, enables a motor, confirms calibration, or sends a command.
"""
from copy import deepcopy
import json
from pathlib import Path
import struct


PROJECT = Path(__file__).resolve().parents[1]
REFERENCE = PROJECT.parent / "VOFA_Pitch_20261017"
OUTPUT = PROJECT.parent / "VOFA_Feeder_20261017"
CHANNELS = [
    ("axis_pos_deg", "输出轴实际 deg"),
    ("axis_target_deg", "输出轴目标 deg"),
    ("raw_rpm", "转子实际 RPM"),
    ("target_rotor_rpm", "转子目标 RPM"),
    ("command_raw", "C610给定 raw"),
    ("fb_current_raw", "C610反馈转矩 raw"),
    ("pos_kp", "位置 KP"),
    ("speed_kp", "速度 KP"),
    ("speed_ki", "速度 KI"),
    ("max_rpm", "转子限速 RPM"),
    ("current_limit", "给定限幅 raw"),
    ("step_deg", "每发输出轴 deg"),
    ("direction", "已生效 DIR"),
    ("gear_ratio", "已生效 GEAR"),
    ("calibrated", "已确认 CAL"),
    ("have_zero", "零点有效"),
    ("online", "反馈在线"),
    ("armed", "拨盘 ARM"),
    ("busy", "执行中"),
    ("faults", "故障位"),
    ("wheels_ready", "真实双轮稳速"),
    ("accepted", "已接受动作"),
    ("completed", "已完成动作"),
    ("rejected", "已拒绝动作"),
    ("aborted", "已中止动作"),
    ("feedback_frames", "反馈帧数"),
    ("age_ms", "反馈年龄 ms"),
    ("raw_angle", "原始编码 0..8191"),
    ("stop_pending", "停机待提交"),
    ("single_accepted", "已接受单发"),
    ("continuous_accepted", "已接受连发"),
    ("completed_interval_ms", "完成间隔 ms"),
    ("tolerance_deg", "输出轴容差 deg"),
    ("settle_rpm", "到位转子速度 RPM"),
    ("settle_ms", "到位确认 ms"),
    ("motion_timeout_ms", "动作超时 ms"),
    ("last_motion_ms", "最近单步耗时 ms"),
    ("raw_rotor_pos", "原始累计转子 deg"),
    ("raw_rotor_target", "原始目标转子 deg"),
]

# These are widget input defaults, never an applied configuration. Every input
# also displays its own telemetry channel, so the board remains the authority.
DEFAULTS = dict(PKP=8.0, VKP=2.0, VKI=5.0, RPM=3600.0, LIMIT=3000.0,
                TOL=1.5, SETTLE=10.0, TIMEOUT=3000.0, DIR=-1.0,
                GEAR=35.68, CAL=1.0, STEP=40.0)


def read(name):
    return json.loads((REFERENCE / name).read_text(encoding="utf-8-sig"))


def write(name, value):
    (OUTPUT / name).write_text(
        json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def walk(value):
    yield value
    if isinstance(value, dict):
        for child in value.values():
            yield from walk(child)
    elif isinstance(value, list):
        for child in value:
            yield from walk(child)


def main():
    commands = read("vofa+.cmds.json")
    tabs = read("vofa+.tabviews.json")
    config = read("vofa+.config.json")
    original_tab = tabs["ctx"][0]["tabs"][0]
    templates = original_tab["widgets"]
    slider_template = next(w for w in templates if w["path"] == "slider")
    button_template = next(w for w in templates if w["path"] == "bound_bt")
    wave_template = next(w for w in templates if w["path"] == "WaveChart")
    gauge_template = next(w for w in templates if w["path"] == "gauge")
    commands["ctx"].update(name="拨盘 40度 受控调试", loop_on=False,
                            loop_count=1, down=True, subCmds=[])

    def command(label, text, intro="仅人工单次发送；以板端返回及反馈值为准"):
        index = len(commands["ctx"]["subCmds"])
        commands["ctx"]["subCmds"].append(dict(
            name=label, intro_on=True, intro=intro, hex_on=False,
            loop_on=False, loop_ms=100, loop_count=1,
            cmd_hex=(text + "\n").encode("ascii").hex(" ").upper(),
            is_group=False, down=False, subCmds=[]))
        return index

    def place(widget, x, y, width, height):
        widget["ctx"]["."]["ctx"] = {
            ".": dict(x=x, y=y, width=width, height=height),
            "resize_border": {"ctx": {".": dict(
                full_screen=False, full_height=False, full_width=False)}}}
        return widget

    def name_widget(widget, label):
        widget["ctx"]["name_menu"]["ctx"]["attr"].update(
            name_=label, name_link_ch=False, name_link_cmd=False)

    def slider(label, parameter, channel, low, high, step, decimal, x, y):
        widget = place(deepcopy(slider_template), x, y, 285, 105)
        ctx = widget["ctx"]
        value = DEFAULTS[parameter]
        ctx["."].update({"from": low, "to": high, "step_size": step})
        ctx["slider_inside"]["target_value"] = value
        caution = ("先在OFF状态手动转动核对方向及GEAR；确认前禁止ARM。"
                   "输入初值不会自动生效，实际值看绑定反馈。")
        ctx["cmd_menu"]["ctx"] = [command(label, f"FEEDER {parameter} %f", caution)]
        ctx["ch_menu"]["ctx"] = channel
        ctx["value_menu"]["ctx"]["attr"]["decimal"] = decimal
        for argument in ctx["argument_menu"]["ctx"]["."]["model_ctx"]:
            argument.update(float_value=value,
                            hex_value=struct.pack("<f", value).hex(" "))
        name_widget(widget, label)
        return widget

    def button(label, text, x, y, width=190, intro=None):
        widget = place(deepcopy(button_template), x, y, width, 64)
        index = command(label, text) if intro is None else command(label, text, intro)
        widget["ctx"]["cmd_menu"]["ctx"] = [index]
        # Press sends once. Releasing cannot duplicate ARM/CAL/SINGLE commands.
        arguments = widget["ctx"]["argument_menu"]["ctx"]["."]["model_ctx"]
        arguments[0]["enabled"], arguments[1]["enabled"] = True, False
        name_widget(widget, label)
        return widget

    def chart(lines, x, y, width, height, bottom, top):
        widget = place(deepcopy(wave_template), x, y, width, height)
        ctx = widget["ctx"]
        ctx["."]["font_size"] = 15
        ctx["axis_x_wave"]["ctx"]["."].update(left_index=48999, right_index=49999)
        ctx["axis_x_wave"]["ctx"]["container"]["ctx"] = []
        for key in ("axis_y_wave", "axis_y_bar"):
            ctx[key]["ctx"]["."].update(bottom_value=bottom, top_value=top)
        ctx["rbw"]["ctx"]["."].update(lines=lines, fake_continuous=False, line_width=2)
        return widget

    def gauge(channel, x, y, low=0, high=1, decimal=0):
        widget = place(deepcopy(gauge_template), x, y, 145, 180)
        label = f"{CHANNELS[channel][1]} I{channel}"
        widget["ctx"]["."].update({"from": low, "to": high, "decimal": decimal})
        widget["ctx"]["ch_menu"]["ctx"] = channel
        widget["ctx"]["value_menu"]["ctx"]["attr"]["decimal"] = decimal
        name_widget(widget, label)
        return widget

    def tab(label, widgets):
        return dict(name=label, widgets=widgets, file_url="", bg=deepcopy(original_tab["bg"]))

    observe = [chart([37], 10, 10, 810, 290, -400, 400),
               chart([2, 3], 10, 320, 810, 250, -6200, 6200)]
    for i, spec in enumerate([
        ("DIR 实测后选±1 / 0未确认", "DIR", 12, -1, 1, 1, 0),
        ("GEAR 实测转子/输出轴", "GEAR", 13, 0.1, 100, 0.01, 3),
        ("CAL 仅实测确认后置1", "CAL", 14, 0, 1, 1, 0),
        ("STEP 输出轴每发40度", "STEP", 11, 0.1, 360, 0.1, 1),
    ]):
        observe.append(slider(*spec, 850 + i % 2 * 300, 10 + i // 2 * 115))
    for i, (label, text) in enumerate([
        ("拨盘波形 VIEW", "FEEDER VIEW"), ("状态 STATUS", "FEEDER STATUS"),
        ("指令与接线 INFO", "FEEDER INFO"), ("实际参数 PID", "FEEDER PID"),
        ("拨盘 OFF", "FEEDER OFF"), ("清故障 CLEAR", "FEEDER CLEAR"),
        ("静止置零 ZERO", "FEEDER ZERO"),
        ("人工恢复 READY", "FEEDER READY"),
    ]):
        observe.append(button(label, text, 850 + i % 3 * 200, 265 + i // 3 * 75))
    for i, channel in enumerate([16, 26, 25, 27, 19, 28, 12, 13, 14, 15]):
        high = {26: 100, 25: 10000, 27: 8192, 19: 512, 12: 1, 13: 100}.get(channel, 1)
        observe.append(gauge(channel, 10 + i % 5 * 153, 595 + i // 5 * 190,
                             -1 if channel == 12 else 0, high, 3 if channel == 13 else 0))
    observe.append(gauge(11, 850, 595, 0, 180, 1))
    observe.append(gauge(37, 1003, 595, -4000, 4000, 2))
    observe.append(gauge(38, 1156, 595, -4000, 4000, 2))

    closed_loop = [chart([0, 1], 10, 10, 810, 300, -405, 5),
                   chart([2, 3], 10, 330, 810, 250, -6200, 6200),
                   chart([4, 5], 10, 600, 810, 250, -3000, 3000)]
    for i, spec in enumerate([
        ("位置 KP I6", "PKP", 6, 0.001, 100, 0.05, 3),
        ("速度 KP I7", "VKP", 7, 0.001, 100, 0.05, 3),
        ("速度 KI I8", "VKI", 8, 0, 1000, 0.1, 3),
        ("转子限速 RPM I9", "RPM", 9, 1, 6000, 10, 0),
        ("C610给定限幅 raw I10", "LIMIT", 10, 1, 3000, 50, 0),
        ("输出轴容差 deg I32", "TOL", 32, 0.05, 5, 0.05, 2),
        ("到位确认 ms I34", "SETTLE", 34, 5, 500, 1, 0),
        ("动作超时 ms I35", "TIMEOUT", 35, 200, 5000, 50, 0),
    ]):
        closed_loop.append(slider(*spec, 850 + i % 2 * 300, 10 + i // 2 * 115))
    for i, (label, text, intro) in enumerate([
        ("拨盘 VIEW", "FEEDER VIEW", "仅切换串口波形到拨盘组，不改变Yaw/Pitch控制"),
        ("实际参数 PID", "FEEDER PID", "查询板端实际生效参数"),
        ("拨盘 OFF", "FEEDER OFF", "立即撤销拨盘动作，取消旧目标"),
        ("实测确认后 ARM", "FEEDER ARM", "先核对实际接线、输出轴40度、DIR、GEAR、CAL与静止零点；确认前禁止ARM"),
        ("互锁单发 SINGLE", "FEEDER SINGLE", "仅人工单次请求；须正常模式、拨盘已ARM及真实双轮稳速；拒绝请求不补发"),
        ("状态 STATUS", "FEEDER STATUS", "查询板端状态与动作计数"),
        ("实际射频 RESULT", "FEEDER RESULT", "读取编码器实际完成边界的频率，不能代替实弹计数"),
        ("人工恢复 READY", "FEEDER READY", "安全OFF且确认机构起点后人工恢复；正常启动自动准备，无需点击"),
    ]):
        closed_loop.append(button(label, text, 850 + i % 3 * 200, 485 + i // 3 * 75, intro=intro))
    for i, channel in enumerate([17, 18, 20, 19]):
        closed_loop.append(gauge(channel, 850 + i * 150, 735, 0, 512 if channel == 19 else 1))

    counts = [chart([21, 22, 23, 24], 10, 10, 810, 300, 0, 100),
              chart([31], 850, 10, 590, 300, 0, 150),
              chart([0, 1], 10, 330, 810, 250, -5, 405),
              chart([26], 850, 330, 590, 250, 0, 100)]
    for i, channel in enumerate([21, 22, 23, 24, 29, 30, 31, 36, 33, 34, 35, 28]):
        high = {31: 150, 33: 100, 34: 100, 35: 5000, 28: 1, 36: 3000}.get(channel, 1000)
        counts.append(gauge(channel, 10 + i % 8 * 153, 605 + i // 8 * 190, 0, high))
    counts.append(button("拨盘 VIEW", "FEEDER VIEW", 1240, 615))
    counts.append(button("实际计数 STATUS", "FEEDER STATUS", 1240, 690))
    counts.append(button("拨盘 OFF", "FEEDER OFF", 1240, 765))

    tabs["ctx"] = [deepcopy(tabs["ctx"][0])]
    tabs["ctx"][0].update(tabs=[
        tab("1 通信与手动方向标定", observe),
        tab("2 闭环与互锁单发", closed_loop),
        tab("3 实际动作与20Hz计数", counts),
    ], currentIndex=0, lastIndex=0, width=1920, height=1080, file_url="")
    command("暂停波形", "TELEM OFF", "只暂停串口波形，不改变电机控制")
    command("恢复波形", "TELEM ON", "恢复当前遥测组，须手动FEEDER VIEW选择拨盘组")

    view = config["ctx"]["wave_view"]["ctx"]
    serial = view["left_panel"]["ctx"]["pal"]["ctx"]["serial"]
    serial.update(port="", baud="921600", dtr=False, rts=False)
    view["left_panel"]["ctx"]["pal"]["ctx"]["protocol_combo"] = "FireWater"
    tx = view["tx_rx_zone"]["ctx"]
    tx["."].update(loop_on=False, loop_count=1, cmd_hex="", send_file=False,
                   send_file_path="", send_hex_on=False)
    tx["send_record_model"]["ctx"] = []
    view["color_bar"]["ctx"]["."].update(delta_t=5, left_index=48999, right_index=49999)
    settings = view["settingsPanel"]["ctx"]["."]["settings_ctx"]
    while len(settings) < len(CHANNELS):
        settings.append(deepcopy(settings[0]))
    del settings[len(CHANNELS):]
    for i, (_, label) in enumerate(CHANNELS):
        settings[i].update(value=0, name=f"I{i} {label}", scale=1, yoffset=0, xoffset=0)
    # Disable every loop flag present in the inherited native configuration.
    for document in (commands, tabs, config):
        for item in walk(document):
            if isinstance(item, dict) and "loop_on" in item:
                item.update(loop_on=False, loop_count=1)

    OUTPUT.mkdir(parents=True, exist_ok=True)
    for filename, document in (("vofa+.cmds.json", commands),
                               ("vofa+.tabviews.json", tabs),
                               ("vofa+.config.json", config)):
        write(filename, document)
    count = len(commands["ctx"]["subCmds"])
    bound_channels, bound_commands = set(), set()
    for page in tabs["ctx"][0]["tabs"]:
        for widget in page["widgets"]:
            ctx = widget["ctx"]
            if "cmd_menu" in ctx:
                indices = ctx["cmd_menu"]["ctx"]
                assert len(indices) == 1 and 0 <= indices[0] < count
                bound_commands.update(indices)
            if "ch_menu" in ctx:
                channel = ctx["ch_menu"]["ctx"]
                assert 0 <= channel < len(CHANNELS)
                bound_channels.add(channel)
            if widget["path"] == "WaveChart":
                indices = ctx["rbw"]["ctx"]["."]["lines"]
                assert all(0 <= channel < len(CHANNELS) for channel in indices)
                bound_channels.update(indices)
            if widget["path"] == "bound_bt":
                args = ctx["argument_menu"]["ctx"]["."]["model_ctx"]
                assert args[0]["enabled"] and not args[1]["enabled"]
    for cmd in commands["ctx"]["subCmds"]:
        wire = bytes.fromhex(cmd["cmd_hex"])
        assert wire.endswith(b"\n") and wire.count(b"\n") == 1
        assert not cmd["loop_on"] and cmd["loop_count"] == 1
        if b"%" in wire:
            assert wire.count(b"%f") == 1
        assert wire.startswith((b"FEEDER ", b"TELEM "))
    assert bound_channels == set(range(len(CHANNELS)))
    for document in (commands, tabs, config):
        assert all(not item["loop_on"] for item in walk(document)
                   if isinstance(item, dict) and "loop_on" in item)
    assert not tx["."]["cmd_hex"] and not tx["send_record_model"]["ctx"]
    assert not serial["port"]
    for name in ("vofa+.cmds.json", "vofa+.tabviews.json", "vofa+.config.json"):
        assert json.loads((OUTPUT / name).read_text(encoding="utf-8"))
    write("validation.json", dict(
        schema="VOFA+ native export v100", commands=count, tabs=3,
        channels=len(CHANNELS), baud=921600, delta_t_ms=5,
        protocol="FireWater", source=str(REFERENCE),
        checks=["JSON roundtrip", "native widget templates", "command indices",
                "all 39 telemetry indices bound", "single LF command termination",
                "all loops off", "buttons send on press only", "empty serial port",
                "empty command buffer/history", "no serial/network/motor access"],
        channel_map={f"I{i}": name for i, (name, _) in enumerate(CHANNELS)},
        action_policy="Compiled measured calibration; firmware prepares at safe OFF; manual recovery only at checked start pose; real wheels gate new shots",
        native_import_verified=False, hardware_used=False,
        warnings=["Structural validation only; VOFA GUI import not tested",
                  "Widget input defaults are not applied board parameters",
                  "I36 is last step latency; completion frequency is in FEEDER RESULT; physical 20Hz still needs observation"]))
    print(f"Generated and structurally validated {count} commands, 3 tabs, 39 channels: {OUTPUT}")


if __name__ == "__main__":
    main()
