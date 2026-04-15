#!/usr/bin/env python3
"""
Usage: python3 generate.py [--output-dir DIR]

Reads protocol.yaml and generates:
  - packet.hpp (C++ ROS driver header)
  - navigation_auto.h (STM32 C header)
  - protocol.py (Python module for tools)
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

import yaml
from jinja2 import Environment, FileSystemLoader, StrictUndefined


SCRIPT_DIR = Path(__file__).resolve().parent
PROTOCOL_FILE = SCRIPT_DIR / "protocol.yaml"
TEMPLATES_DIR = SCRIPT_DIR / "templates"


def to_camel(name: str) -> str:
    return "".join(part.capitalize() for part in name.split("_"))


def to_upper(name: str) -> str:
    return name.upper()


def normalize_protocol(raw: Any) -> dict[str, Any]:
    if not isinstance(raw, dict):
        raise ValueError("protocol.yaml 顶层必须是 mapping")
    type_map = raw.get("type_map")
    packets_raw = raw.get("packets")
    if not isinstance(type_map, dict):
        raise ValueError("protocol.yaml 缺少 type_map 或格式错误")
    if not isinstance(packets_raw, dict):
        raise ValueError("protocol.yaml 缺少 packets 或格式错误")

    packets = []
    for packet_name, packet_cfg in packets_raw.items():
        if not isinstance(packet_cfg, dict):
            raise ValueError(f"packet '{packet_name}' 配置必须是 mapping")
        fields_raw = packet_cfg.get("fields", [])
        if not isinstance(fields_raw, list):
            raise ValueError(f"packet '{packet_name}' 的 fields 必须是 list")

        direction = packet_cfg.get("direction")
        if direction not in {"stm32_to_ros", "ros_to_stm32"}:
            raise ValueError(f"packet '{packet_name}' 的 direction 非法: {direction}")

        header_value = packet_cfg.get("header")
        if header_value is None:
            raise ValueError(f"packet '{packet_name}' 缺少 header")
        frequency_hz = packet_cfg.get("frequency_hz")
        if not isinstance(frequency_hz, int) or frequency_hz <= 0:
            raise ValueError(f"packet '{packet_name}' 的 frequency_hz 必须是正整数")

        fields = []
        payload_size = 0
        struct_chars = []
        for field in fields_raw:
            if not isinstance(field, dict):
                raise ValueError(f"packet '{packet_name}' 中存在非法 field 配置")
            field_type = field.get("type")
            if field_type not in type_map:
                raise ValueError(
                    f"packet '{packet_name}' 字段 '{field.get('name')}' 的类型 '{field_type}' 未在 type_map 定义"
                )
            type_info = type_map[field_type]
            try:
                c_type = type_info["c"]
                cpp_type = type_info["cpp"]
                python_struct = type_info["python_struct"]
                size = int(type_info["size"])
            except Exception as exc:  # noqa: BLE001
                raise ValueError(f"type_map.{field_type} 配置不完整") from exc

            payload_size += size
            struct_chars.append(python_struct)
            fields.append(
                {
                    "name": field["name"],
                    "type": field_type,
                    "c_type": c_type,
                    "cpp_type": cpp_type,
                    "python_struct": python_struct,
                    "size": size,
                    "unit": field.get("unit"),
                    "desc": field.get("desc", ""),
                }
            )

        total_size = 1 + payload_size + 2
        payload_format = "<B" + "".join(struct_chars)
        struct_format = payload_format + "H"

        packet_upper = to_upper(packet_name)
        if direction == "ros_to_stm32":
            cpp_header_const = f"HEADER_{packet_upper}_TX"
            c_header_const = "FH_RX"
        else:
            cpp_header_const = f"HEADER_{packet_upper}"
            c_header_const = f"FH_TX_{packet_upper}"

        packets.append(
            {
                "key": packet_name,
                "name_camel": to_camel(packet_name),
                "name_upper": packet_upper,
                "header": int(header_value),
                "header_hex": f"0x{int(header_value):02X}",
                "direction": direction,
                "frequency_hz": frequency_hz,
                "send_interval": 1000 // frequency_hz,
                "fields": fields,
                "payload_size": payload_size,
                "total_size": total_size,
                "payload_format": payload_format,
                "struct_format": struct_format,
                "cpp_header_const": cpp_header_const,
                "c_header_const": c_header_const,
                "cpp_struct_name": (
                    f"Receive{to_camel(packet_name)}Packet" if direction == "stm32_to_ros" else f"Send{to_camel(packet_name)}Packet"
                ),
                "c_struct_name": (
                    f"Send{to_camel(packet_name)}Packet" if direction == "stm32_to_ros" else "ReceivePacket"
                ),
                "py_class_name": f"{to_camel(packet_name)}Packet",
            }
        )

    stm32_to_ros = [p for p in packets if p["direction"] == "stm32_to_ros"]
    ros_to_stm32 = [p for p in packets if p["direction"] == "ros_to_stm32"]
    if len(ros_to_stm32) != 1:
        raise ValueError("当前模板要求恰好一个 ros_to_stm32 包")

    return {
        "version": raw.get("version", ""),
        "type_map": type_map,
        "packets": packets,
        "stm32_to_ros": stm32_to_ros,
        "ros_to_stm32": ros_to_stm32,
        "nav_packet": ros_to_stm32[0],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate protocol code from protocol.yaml")
    _ = parser.add_argument("--output-dir", type=Path, default=SCRIPT_DIR / "generated")
    args = parser.parse_args()

    try:
        with PROTOCOL_FILE.open("r", encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        context = normalize_protocol(raw)
    except (yaml.YAMLError, OSError, ValueError) as exc:
        print(f"Error: failed to parse protocol.yaml: {exc}", file=sys.stderr)
        return 1

    env = Environment(
        loader=FileSystemLoader(str(TEMPLATES_DIR)),
        undefined=StrictUndefined,
        trim_blocks=False,
        lstrip_blocks=True,
        keep_trailing_newline=True,
    )

    outputs = [
        ("packet.hpp.j2", "packet.hpp"),
        ("navigation_auto.h.j2", "navigation_auto.h"),
        ("protocol_py.j2", "protocol.py"),
    ]

    out_dir = args.output_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    generated_paths = []
    for template_name, output_name in outputs:
        template = env.get_template(template_name)
        rendered = template.render(**context)
        out_path = out_dir / output_name
        out_path.write_text(rendered, encoding="utf-8")
        generated_paths.append(out_path)

    print(f"Generated {len(generated_paths)} files in {out_dir}:")
    for p in generated_paths:
        print(f"  - {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
