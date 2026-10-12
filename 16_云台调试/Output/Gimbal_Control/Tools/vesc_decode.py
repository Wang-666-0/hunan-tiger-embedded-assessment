"""Decode captured VESC read replies; this module never opens a device or writes settings.

Wire layouts are taken from the supplied official bldc source, commit
94b305ec124b67b0161f2d3d85c4d5b35e48edc5 (6.06). Full configurations are decoded
only after their signature and exact length match the bundled schema.
"""
from __future__ import annotations

import json
import math
from pathlib import Path
import struct


class DecodeError(ValueError):
    """A reply cannot safely be interpreted using the known layout."""


def finite_float(raw: bytes, endian: str = ">") -> float:
    value = struct.unpack(endian + "f", raw)[0]
    if not math.isfinite(value):
        raise DecodeError("non-finite float")
    return value


def _config(payload: bytes, command: int) -> dict:
    if len(payload) < 5:
        raise DecodeError("configuration signature missing")
    signature = int.from_bytes(payload[1:5], "big")
    schemas = json.loads(Path(__file__).with_name("vesc_schema_6_06.json").read_text(encoding="utf-8"))
    schema = schemas["mcconf" if command == 14 else "appconf"]
    result = {"signature": signature, "schema_source": schemas["source"], "decoded": False}
    if signature != schema["signature"]:
        result["reason"] = "unknown_configuration_signature; raw reply preserved"
        return result
    if len(payload) != 1 + schema["length"]:
        result["reason"] = "known_signature_but_unexpected_length; raw reply preserved"
        result["expected_length"] = 1 + schema["length"]
        return result
    fields = {}
    for field in schema["fields"]:
        offset = 1 + field["offset"]
        raw = payload[offset:offset + field["size"]]
        if field["type"] == "float32_auto":
            value = finite_float(raw)
        else:
            value = int.from_bytes(raw, "big", signed=field["type"] in ("int8", "int32", "float16"))
            if field["type"] == "float16":
                value /= field["scale"]
        fields[field["name"]] = value
    result.update(decoded=True, fields=fields)
    return result


def decode_vesc(payload: bytes, expected_command: int) -> dict:
    """Return fields alongside layout provenance. Raw bytes remain in the report."""
    if not payload or payload[0] != expected_command:
        raise DecodeError("reply command does not match request")
    if expected_command == 0:
        if len(payload) < 4:
            raise DecodeError("firmware version too short")
        try:
            terminator = payload.index(0, 3)
        except ValueError as exc:
            raise DecodeError("hardware name lacks terminator") from exc
        uuid_end = terminator + 13
        if uuid_end > len(payload):
            raise DecodeError("firmware UUID missing")
        return {"major": payload[1], "minor": payload[2],
                "hardware_name": payload[3:terminator].decode("ascii", errors="replace"),
                "uuid_hex": payload[terminator + 1:uuid_end].hex().upper(),
                "extra_hex": payload[uuid_end:].hex().upper()}
    if expected_command in (14, 17):
        return _config(payload, expected_command)
    if expected_command == 91:
        if len(payload) != 50:
            raise DecodeError("GET_MCCONF_TEMP requires the known 50-byte layout")
        names = ("l_current_min_scale", "l_current_max_scale", "l_min_erpm", "l_max_erpm",
                 "l_min_duty", "l_max_duty", "l_watt_min", "l_watt_max",
                 "l_in_current_min", "l_in_current_max")
        fields = {name: finite_float(payload[1 + 4*i:5 + 4*i]) for i, name in enumerate(names)}
        poles = payload[41]
        fields.update(si_motor_poles=poles,
                      configured_pole_pairs=poles / 2 if poles and poles % 2 == 0 else None,
                      si_gear_ratio=finite_float(payload[42:46]),
                      si_wheel_diameter=finite_float(payload[46:50]))
        return {"layout": "official_bldc_6.06_GET_MCCONF_TEMP", "fields": fields,
                "note": "Values are ESC configuration, not physical identification. Confirm poles and gear ratio on the mechanism."}
    if expected_command == 4:
        # The base GET_VALUES prefix includes electrical RPM, not wheel mechanical RPM.
        if len(payload) < 54:
            raise DecodeError("GET_VALUES base fields incomplete")
        offset = 1
        fields = {}
        for name, size, scale in (
            ("fet_temperature_c", 2, 10), ("motor_temperature_c", 2, 10),
            ("motor_current_a", 4, 100), ("input_current_a", 4, 100),
            ("id_current_a", 4, 100), ("iq_current_a", 4, 100),
            ("duty_cycle", 2, 1000), ("electrical_rpm", 4, 1),
            ("input_voltage_v", 2, 10), ("amp_hours", 4, 10000),
            ("amp_hours_charged", 4, 10000), ("watt_hours", 4, 10000),
            ("watt_hours_charged", 4, 10000), ("tachometer", 4, 1),
            ("tachometer_absolute", 4, 1),
        ):
            value = int.from_bytes(payload[offset:offset + size], "big", signed=True)
            fields[name] = value if scale == 1 else value / scale
            offset += size
        fields["fault_code"] = payload[offset]
        return {"layout": "official_bldc_6.06_GET_VALUES_base_prefix", "fields": fields,
                "extra_hex": payload[offset + 1:].hex().upper()}
    raise DecodeError("command is outside the read whitelist")
