"""Build a separate VOFA+ panel from the repository's exported native widget schema.

Does not open a serial port, edit the user's live VOFA configuration, or issue commands.
"""
from copy import deepcopy
import json
from pathlib import Path
import struct


PROJECT = Path(__file__).resolve().parents[1]
REFERENCE = PROJECT.parent / "VOFA_Pitch_20261017"
OUTPUT = PROJECT.parent / "VOFA_Yaw_PID_20261018"


def read(name):
    return json.loads((REFERENCE / name).read_text(encoding="utf-8-sig"))


def write(name, value):
    (OUTPUT / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main():
    commands, tabs, config = read("vofa+.cmds.json"), read("vofa+.tabviews.json"), read("vofa+.config.json")
    templates = tabs["ctx"][0]["tabs"][0]["widgets"]
    slider_template = next(w for w in templates if w["path"] == "slider")
    button_template = next(w for w in templates if w["path"] == "bound_bt")
    wave_template = next(w for w in templates if w["path"] == "WaveChart")
    gauge_template = next(w for w in templates if w["path"] == "gauge")
    commands["ctx"].update(name="Yaw V2N 达标双环PID", loop_on=False, loop_count=1, down=True)
    commands["ctx"]["subCmds"] = []

    def command(name, text):
        index = len(commands["ctx"]["subCmds"])
        commands["ctx"]["subCmds"].append(dict(name=name, intro_on=True, intro="单次发送；系数仅RAM",
            hex_on=False, loop_on=False, loop_ms=100, loop_count=1,
            cmd_hex=(text + "\n").encode("ascii").hex(" ").upper(), is_group=False, down=False, subCmds=[]))
        return index

    def place(widget, x, y, width, height):
        widget["ctx"]["."]["ctx"] = {".": dict(x=x, y=y, width=width, height=height),
            "resize_border": {"ctx": {".": dict(full_screen=False, full_height=False, full_width=False)}}}
        return widget

    def name_widget(widget, label):
        widget["ctx"]["name_menu"]["ctx"]["attr"].update(name_=label, name_link_ch=False, name_link_cmd=False)

    def slider(label, text, channel, low, high, step, value, decimal, x, y):
        widget = place(deepcopy(slider_template), x, y, 275, 105)
        ctx = widget["ctx"]
        ctx["."].update({"from": low, "to": high, "step_size": step})
        ctx["slider_inside"]["target_value"] = value
        ctx["cmd_menu"]["ctx"] = [command(label, text)]
        ctx["ch_menu"]["ctx"] = channel
        ctx["value_menu"]["ctx"]["attr"]["decimal"] = decimal
        for argument in ctx["argument_menu"]["ctx"]["."]["model_ctx"]:
            argument.update(float_value=value, hex_value=struct.pack("<f", value).hex(" "))
        name_widget(widget, label)
        return widget

    def button(label, text, x, y, width=170):
        widget = place(deepcopy(button_template), x, y, width, 65)
        widget["ctx"]["cmd_menu"]["ctx"] = [command(label, text)]
        # Send only on press: HOME/STEP must not be duplicated on release.
        args = widget["ctx"]["argument_menu"]["ctx"]["."]["model_ctx"]
        args[0]["enabled"], args[1]["enabled"] = True, False
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

    def gauge(label, channel, x, y, low, high, decimal):
        widget = place(deepcopy(gauge_template), x, y, 145, 225)
        widget["ctx"]["."].update({"from": low, "to": high, "decimal": decimal})
        widget["ctx"]["ch_menu"]["ctx"] = channel
        widget["ctx"]["value_menu"]["ctx"]["attr"]["decimal"] = decimal
        name_widget(widget, label)
        return widget

    original_tab = tabs["ctx"][0]["tabs"][0]

    def tab(label, widgets):
        return dict(name=label, widgets=widgets, file_url="", bg=deepcopy(original_tab["bg"]))

    pid_widgets = [chart([0, 1], 10, 10, 760, 420, -2, 33)]
    pid_parameters = [
        ("位置 KP", "YAW PKP %f", 7, 0, 25, 0.5, 14, 2),
        ("位置 KI", "YAW PKI %f", 8, 0, 1, 0.05, 0.15, 3),
        ("位置 KD", "YAW PKD %f", 9, 0, 1, 0.01, 0.15, 3),
        ("速度 KP", "YAW VKP %f", 10, 0, 0.015, 0.0001, 0.0075, 5),
        ("速度 KI", "YAW VKI %f", 11, 0, 0.02, 0.0005, 0.005, 4),
        ("速度 KD", "YAW VKD %f", 12, 0, 0.0005, 0.00001, 0.00018, 6),
    ]
    for i, spec in enumerate(pid_parameters):
        pid_widgets.append(slider(*spec, 790 + (i // 3) * 290, 10 + i % 3 * 115))
    for i, (label, text) in enumerate([
        ("Yaw PID", "YAW PID"), ("Yaw 状态", "YAW STATUS"), ("阶跃 +30", "YAW STEP 30"),
        ("回零 HOME", "YAW HOME"), ("完整结果", "YAW RESULT"), ("阶跃就绪检查", "YAW STEP CHECK"), ("安全准备 READY", "YAW READY"), ("Yaw 波形", "YAW VIEW"), ("停止 STOP", "STOP"),
    ]):
        pid_widgets.append(button(label, text, 790 + i % 3 * 185, 365 + i // 3 * 75))
    diagnostics = [chart([0, 1], 10, 10, 660, 310, -2, 33),
        chart([2, 3], 690, 10, 660, 310, -400, 400),
        chart([5, 6, 56, 57], 10, 340, 660, 310, -1, 1),
        chart([58], 690, 340, 660, 310, -2, 2)]
    tabs["ctx"][0].update(tabs=[tab("Yaw 达标PID", pid_widgets),
        tab("角度-速度-力矩诊断", diagnostics)], currentIndex=0, lastIndex=0, width=1920, height=1080, file_url="")
    # Additional read-only commands are available in the command list, not action sliders.
    command("阶跃就绪检查", "YAW STEP CHECK")
    command("阶跃 -30", "YAW STEP -30")
    command("暂停波形", "TELEM OFF")
    command("恢复波形", "TELEM ON")

    view = config["ctx"]["wave_view"]["ctx"]
    serial = view["left_panel"]["ctx"]["pal"]["ctx"]["serial"]
    serial.update(port="", baud="921600")
    view["left_panel"]["ctx"]["pal"]["ctx"]["protocol_combo"] = "FireWater"
    view["tx_rx_zone"]["ctx"]["."].update(loop_on=False, loop_count=1, cmd_hex="")
    view["tx_rx_zone"]["ctx"]["send_record_model"]["ctx"] = []
    view["color_bar"]["ctx"]["."].update(delta_t=5, left_index=48999, right_index=49999)
    settings = view["settingsPanel"]["ctx"]["."]["settings_ctx"]
    for i in range(72):
        if i >= len(settings):
            settings.append(deepcopy(settings[0]))
        settings[i].update(value=0, name=f"I{i}", scale=1, yoffset=0, xoffset=0)
    # Keep native channel names so the guide and firmware use identical indices.
    OUTPUT.mkdir(parents=True, exist_ok=True)
    write("vofa+.cmds.json", commands)
    write("vofa+.tabviews.json", tabs)
    write("vofa+.config.json", config)
    count = len(commands["ctx"]["subCmds"])
    for window in tabs["ctx"]:
        for page in window["tabs"]:
            for widget in page["widgets"]:
                ctx = widget["ctx"]
                if "cmd_menu" in ctx:
                    assert 0 <= ctx["cmd_menu"]["ctx"][0] < count
                if "ch_menu" in ctx:
                    assert 0 <= ctx["ch_menu"]["ctx"] < 72
                if widget["path"] == "WaveChart":
                    assert all(0 <= i < 72 for i in ctx["rbw"]["ctx"]["."]["lines"])
    for cmd in commands["ctx"]["subCmds"]:
        wire = bytes.fromhex(cmd["cmd_hex"])
        assert wire.endswith(b"\n") and not cmd["loop_on"]
        assert not any(old in wire for old in (b"FBIAS", b"FANG", b"FKV", b" FC", b"FVS", b"FKA", b"FMAX", b"SHAPE", b"YAW FF"))
        if b"%" in wire:
            assert wire.count(b"%f") == 1
    write("validation.json", dict(schema="VOFA+ native export v100", commands=count, tabs=2,
        channels=72, baud=921600, delta_t_ms=5, checks="JSON, command paths, data indices, LF, loop off, retired commands absent",
        source=str(REFERENCE), native_import_verified=False, hardware_used=False))
    print(f"Generated and structurally validated {count} commands, 2 tabs: {OUTPUT}")


if __name__ == "__main__":
    main()
