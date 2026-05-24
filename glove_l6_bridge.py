#!/usr/bin/env python3
"""Bridge 9MOTCAP G7s UDP glove frames to RealHand L6/L20/O6 hands over CAN.

The RealHand SDK exposes L6/O6 targets as six normalized angles in the order:
thumb flexion, thumb abduction, index, middle, ring, pinky. Values are in
0..100, where 100 is close to open and 0 is close to closed.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import signal
import socket
import sys
import time
from collections import OrderedDict
from collections.abc import Mapping
from contextlib import ExitStack
from dataclasses import dataclass, field
from typing import Any


L6_JOINTS = ("thumb_flex", "thumb_abd", "index", "middle", "ring", "pinky")
O6_JOINTS = L6_JOINTS

L20_JOINTS = (
    "thumb_abd",
    "thumb_yaw",
    "thumb_root1",
    "thumb_tip",
    "index_abd",
    "index_root1",
    "index_tip",
    "middle_abd",
    "middle_root1",
    "middle_tip",
    "ring_abd",
    "ring_root1",
    "ring_tip",
    "pinky_abd",
    "pinky_root1",
    "pinky_tip",
)

MODEL_JOINTS = {
    "l6": L6_JOINTS,
    "o6": O6_JOINTS,
    "l20": L20_JOINTS,
}

DEFAULT_THUMB_MODE = "pitch-side"

DEFAULT_RANGES = {
    "thumb_flex": (0.0, 60.0),
    "thumb_abd": (-45.0, 45.0),
    "index": (0.0, 90.0),
    "middle": (0.0, 90.0),
    "ring": (0.0, 90.0),
    "pinky": (0.0, 90.0),
}

DEFAULT_L6_SIDE_PITCH_RANGES = {
    **DEFAULT_RANGES,
    "thumb_flex": (-45.0, 45.0),
    "thumb_abd": (0.0, 90.0),
}

DEFAULT_L20_RANGES = {
    "thumb_abd": (-45.0, 45.0),
    "thumb_yaw": (-90.0, 90.0),
    "thumb_root1": (0.0, 90.0),
    "thumb_tip": (0.0, 90.0),
    "index_abd": (-30.0, 30.0),
    "index_root1": (0.0, 90.0),
    "index_tip": (0.0, 90.0),
    "middle_abd": (-30.0, 30.0),
    "middle_root1": (0.0, 90.0),
    "middle_tip": (0.0, 90.0),
    "ring_abd": (-30.0, 30.0),
    "ring_root1": (0.0, 90.0),
    "ring_tip": (0.0, 90.0),
    "pinky_abd": (-30.0, 30.0),
    "pinky_root1": (0.0, 90.0),
    "pinky_tip": (0.0, 90.0),
}

DEFAULT_L20_SIDE_PITCH_RANGES = {
    **DEFAULT_L20_RANGES,
    "thumb_abd": (0.0, 90.0),
    "thumb_root1": (-45.0, 45.0),
}

MODEL_DEFAULT_RANGES = {
    ("l6", "pitch-side"): DEFAULT_RANGES,
    ("l6", "side-pitch"): DEFAULT_L6_SIDE_PITCH_RANGES,
    ("o6", "pitch-side"): DEFAULT_RANGES,
    ("o6", "side-pitch"): DEFAULT_L6_SIDE_PITCH_RANGES,
    ("l20", "pitch-side"): DEFAULT_L20_RANGES,
    ("l20", "side-pitch"): DEFAULT_L20_SIDE_PITCH_RANGES,
}

DEFAULT_INVERTED = frozenset(("thumb_flex", "index", "middle", "ring", "pinky"))
DEFAULT_L6_SIDE_PITCH_INVERTED = frozenset(("index", "middle", "ring", "pinky"))
DEFAULT_L20_INVERTED = frozenset(
    (
        "thumb_root1",
        "thumb_tip",
        "index_root1",
        "index_tip",
        "middle_root1",
        "middle_tip",
        "ring_root1",
        "ring_tip",
        "pinky_root1",
        "pinky_tip",
    )
)
DEFAULT_L20_SIDE_PITCH_INVERTED = frozenset(
    (
        "thumb_tip",
        "index_root1",
        "index_tip",
        "middle_root1",
        "middle_tip",
        "ring_root1",
        "ring_tip",
        "pinky_root1",
        "pinky_tip",
    )
)

MODEL_DEFAULT_INVERTED = {
    ("l6", "pitch-side"): DEFAULT_INVERTED,
    ("l6", "side-pitch"): DEFAULT_L6_SIDE_PITCH_INVERTED,
    ("o6", "pitch-side"): DEFAULT_INVERTED,
    ("o6", "side-pitch"): DEFAULT_L6_SIDE_PITCH_INVERTED,
    ("l20", "pitch-side"): DEFAULT_L20_INVERTED,
    ("l20", "side-pitch"): DEFAULT_L20_SIDE_PITCH_INVERTED,
}

MODEL_OPEN_POSES = {
    "l6": [100.0] * len(L6_JOINTS),
    "o6": [100.0] * len(O6_JOINTS),
    "l20": [
        50.0,
        50.0,
        100.0,
        100.0,
        50.0,
        100.0,
        100.0,
        50.0,
        100.0,
        100.0,
        50.0,
        100.0,
        100.0,
        50.0,
        100.0,
        100.0,
    ],
}

HAND_KEY_ALIASES = {
    "left": ("leftHand", "left_hand", "LeftHand", "left", "Left"),
    "right": ("rightHand", "right_hand", "RightHand", "right", "Right"),
}

JOINT_ALIASES = {
    "thumb_flex": (
        "thumb_flex",
        "thumbFlex",
        "thumb_bend",
        "thumbBend",
        "thumb_curl",
        "thumbCurl",
        "thumb_mcp",
        "thumbMCP",
        "thumb_mcp_flex",
        "thumbMcpFlex",
        "thumb_cmc_pitch",
        "thumbCmcPitch",
        "thumb_pitch",
        "thumbPitch",
        "thumb1",
        "finger0",
    ),
    "thumb_abd": (
        "thumb_abd",
        "thumbAbd",
        "thumb_abduction",
        "thumbAbduction",
        "thumb_adduction",
        "thumbAdduction",
        "thumb_splay",
        "thumbSplay",
        "thumb_yaw",
        "thumbYaw",
        "thumb_cmc_yaw",
        "thumbCmcYaw",
        "thumb_side",
        "thumbSide",
        "thumb2",
    ),
    "index": (
        "index",
        "index_flex",
        "indexFlex",
        "index_bend",
        "indexBend",
        "index_curl",
        "indexCurl",
        "index_mcp",
        "indexMCP",
        "index_mcp_pitch",
        "indexMcpPitch",
        "index_proximal",
        "indexProximal",
        "forefinger",
        "forefingerFlex",
        "finger1",
    ),
    "middle": (
        "middle",
        "middle_flex",
        "middleFlex",
        "middle_bend",
        "middleBend",
        "middle_curl",
        "middleCurl",
        "middle_mcp",
        "middleMCP",
        "middle_mcp_pitch",
        "middleMcpPitch",
        "middle_proximal",
        "middleProximal",
        "finger2",
    ),
    "ring": (
        "ring",
        "ring_flex",
        "ringFlex",
        "ring_bend",
        "ringBend",
        "ring_curl",
        "ringCurl",
        "ring_mcp",
        "ringMCP",
        "ring_mcp_pitch",
        "ringMcpPitch",
        "ring_proximal",
        "ringProximal",
        "finger3",
    ),
    "pinky": (
        "pinky",
        "pinkie",
        "little",
        "little_finger",
        "littleFinger",
        "pinky_flex",
        "pinkyFlex",
        "pinkie_flex",
        "pinkieFlex",
        "little_flex",
        "littleFlex",
        "pinky_bend",
        "pinkyBend",
        "pinky_curl",
        "pinkyCurl",
        "pinky_mcp",
        "pinkyMCP",
        "little_mcp",
        "littleMCP",
        "pinky_mcp_pitch",
        "pinkyMcpPitch",
        "finger4",
    ),
}

LIST_ALIASES = (
    "angles",
    "angle",
    "joints",
    "jointAngles",
    "fingerAngles",
    "handAngles",
    "fingers",
)

G7S_ARRAY_ALIASES = {
    "pitch": ("pitch", "Pitch", "fingerPitch", "finger_pitch"),
    "side": ("side", "Side", "fingerSide", "finger_side"),
}


class MissingJointsError(ValueError):
    """Raised when a glove frame cannot be mapped to all required joints."""

    def __init__(self, missing: list[str], available: list[str]) -> None:
        super().__init__(
            "missing joints: "
            + ", ".join(missing)
            + "; available numeric fields: "
            + ", ".join(available[:60])
        )
        self.missing = missing
        self.available = available


@dataclass
class Stats:
    received: int = 0
    parsed: int = 0
    sent: int = 0
    parse_errors: int = 0
    map_errors: int = 0
    sent_by_hand: dict[str, int] = field(default_factory=dict)
    map_errors_by_hand: dict[str, int] = field(default_factory=dict)
    last_angles: dict[str, list[float]] = field(default_factory=dict)
    last_thumb_log_time: float = 0.0


@dataclass
class HandRuntime:
    name: str
    glove_hand: str
    hand_model: str
    thumb_mode: str
    thumb_abd_source: str
    l20_thumb_tip_source: str
    ranges: dict[str, tuple[float, float]]
    inverted: set[str]
    controller: "RealHandController"
    smoothed: list[float] | None = None
    last_sent: list[float] | None = None
    last_send_time: float = 0.0
    last_map_error: float = 0.0
    last_thumb_log_time: float = 0.0


def parse_json_option(value: str | None, *, name: str) -> Any:
    if not value:
        return None
    try:
        return json.loads(value)
    except json.JSONDecodeError as exc:
        raise argparse.ArgumentTypeError(f"{name} is not valid JSON: {exc}") from exc


def load_json_file(path: str | None) -> Any:
    if not path:
        return None
    with open(path, "r", encoding="utf-8") as file:
        return json.load(file)


def normalize_key(key: str) -> str:
    return "".join(ch.lower() for ch in key if ch.isalnum())


def is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def flatten_numbers(value: Any, prefix: str = "") -> OrderedDict[str, float]:
    flat: OrderedDict[str, float] = OrderedDict()
    if isinstance(value, Mapping):
        for key, item in value.items():
            path = f"{prefix}.{key}" if prefix else str(key)
            flat.update(flatten_numbers(item, path))
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            path = f"{prefix}.{index}" if prefix else str(index)
            flat.update(flatten_numbers(item, path))
    elif is_number(value) and prefix:
        flat[prefix] = float(value)
    return flat


def lookup_path(value: Any, path: str) -> float | None:
    current = value
    for part in path.replace("/", ".").split("."):
        if part == "":
            continue
        if isinstance(current, Mapping):
            if part not in current:
                return None
            current = current[part]
        elif isinstance(current, (list, tuple)):
            try:
                current = current[int(part)]
            except (ValueError, IndexError):
                return None
        else:
            return None
    if is_number(current):
        return float(current)
    return None


def find_numeric_list(value: Any, length: int = len(L6_JOINTS)) -> list[float] | None:
    if isinstance(value, (list, tuple)) and len(value) >= length:
        items = list(value[:length])
        if all(is_number(item) for item in items):
            return [float(item) for item in items]
    if isinstance(value, Mapping):
        for key in LIST_ALIASES:
            if key in value:
                found = find_numeric_list(value[key], length=length)
                if found is not None:
                    return found
        for item in value.values():
            found = find_numeric_list(item, length=length)
            if found is not None:
                return found
    return None


def coerce_number_sequence(value: Any, minimum_length: int) -> list[float] | None:
    if isinstance(value, (list, tuple)) and len(value) >= minimum_length:
        items = list(value[:minimum_length])
        if all(is_number(item) for item in items):
            return [float(item) for item in items]

    if isinstance(value, Mapping):
        items: list[Any] = []
        for index in range(minimum_length):
            if index in value:
                items.append(value[index])
            elif str(index) in value:
                items.append(value[str(index)])
            else:
                return None
        if all(is_number(item) for item in items):
            return [float(item) for item in items]

    return None


def find_number_array(value: Any, aliases: tuple[str, ...], minimum_length: int) -> list[float] | None:
    if not isinstance(value, Mapping):
        return None

    normalized_aliases = {normalize_key(alias) for alias in aliases}
    for key, item in value.items():
        if normalize_key(str(key)) in normalized_aliases:
            found = coerce_number_sequence(item, minimum_length)
            if found is not None:
                return found

    for item in value.values():
        if isinstance(item, Mapping):
            found = find_number_array(item, aliases, minimum_length)
            if found is not None:
                return found
        elif isinstance(item, (list, tuple)):
            for child in item:
                found = find_number_array(child, aliases, minimum_length)
                if found is not None:
                    return found

    return None


def select_hand(frame: Any, glove_hand: str) -> Any:
    found = find_hand_payload(frame, glove_hand)
    return frame if found is None else found


def select_hand_for_mode(frame: Any, glove_hand: str, both_hands: bool) -> Any | None:
    found = find_hand_payload(frame, glove_hand)
    if found is not None:
        return found
    if both_hands:
        return None
    return frame


def find_hand_payload(value: Any, glove_hand: str) -> Any | None:
    if not isinstance(value, Mapping):
        return None

    for key in HAND_KEY_ALIASES[glove_hand]:
        if key in value:
            return value[key]

    normalized_wanted = {normalize_key(key) for key in HAND_KEY_ALIASES[glove_hand]}
    for key, item in value.items():
        if normalize_key(str(key)) in normalized_wanted:
            return item

    for item in value.values():
        if isinstance(item, Mapping):
            found = find_hand_payload(item, glove_hand)
            if found is not None:
                return found
        elif isinstance(item, (list, tuple)):
            for child in item:
                found = find_hand_payload(child, glove_hand)
                if found is not None:
                    return found

    return None


def extract_by_mapping(
    hand_data: Any,
    mapping: Mapping[str, str],
    joint_order: tuple[str, ...],
) -> dict[str, float]:
    flat = flatten_numbers(hand_data)
    normalized = {normalize_key(path): value for path, value in flat.items()}
    extracted: dict[str, float] = {}

    for joint in joint_order:
        path = mapping.get(joint)
        if not path:
            continue
        value = lookup_path(hand_data, path)
        if value is None:
            value = flat.get(path)
        if value is None:
            value = normalized.get(normalize_key(path))
        if value is not None:
            extracted[joint] = value

    return extracted


def extract_by_aliases(hand_data: Any) -> dict[str, float]:
    flat = flatten_numbers(hand_data)
    indexed: list[tuple[str, str, str, float]] = []
    for path, value in flat.items():
        leaf = path.rsplit(".", 1)[-1]
        indexed.append((path, normalize_key(path), normalize_key(leaf), value))

    extracted: dict[str, float] = {}
    for joint, aliases in JOINT_ALIASES.items():
        alias_norms = [normalize_key(alias) for alias in aliases]

        for alias in alias_norms:
            match = next((row for row in indexed if row[2] == alias), None)
            if match is not None:
                extracted[joint] = match[3]
                break
        if joint in extracted:
            continue

        for alias in alias_norms:
            match = next((row for row in indexed if row[1] == alias), None)
            if match is not None:
                extracted[joint] = match[3]
                break
        if joint in extracted:
            continue

        for alias in alias_norms:
            match = next((row for row in indexed if row[1].endswith(alias)), None)
            if match is not None:
                extracted[joint] = match[3]
                break

    return extracted


def choose_thumb_abd(
    pitch: list[float],
    side: list[float],
    roll: list[float],
    thumb_mode: str,
    source: str,
) -> float:
    if source == "neutral":
        return 0.0
    if source == "pitch":
        return pitch[0]
    if source == "side":
        return side[0]
    if source == "roll":
        return roll[0]
    return pitch[0] if thumb_mode == "side-pitch" else side[0]


def extract_g7s_arrays(
    hand_data: Any,
    thumb_mode: str = DEFAULT_THUMB_MODE,
    thumb_abd_source: str = "neutral",
) -> dict[str, float]:
    pitch = find_number_array(hand_data, G7S_ARRAY_ALIASES["pitch"], 5)
    side = find_number_array(hand_data, G7S_ARRAY_ALIASES["side"], 5)
    if pitch is None or side is None:
        return {}
    roll = find_number_array(hand_data, ("roll", "Roll", "fingerRoll", "finger_roll"), 5) or [0.0] * 5

    if thumb_mode == "side-pitch":
        thumb_flex = side[0]
    else:
        thumb_flex = pitch[0]
    thumb_abd = choose_thumb_abd(pitch, side, roll, thumb_mode, thumb_abd_source)

    return {
        "thumb_flex": thumb_flex,
        "thumb_abd": thumb_abd,
        "index": pitch[1],
        "middle": pitch[2],
        "ring": pitch[3],
        "pinky": pitch[4],
    }


def choose_l20_tip(
    two_pitch: list[float] | None,
    end_pitch: list[float] | None,
    pitch: list[float],
    index: int,
    source: str = "end",
) -> float:
    if source == "end" and end_pitch is not None:
        return end_pitch[index]
    if source == "two" and two_pitch is not None:
        return two_pitch[index]
    if source == "pitch":
        return pitch[index]
    if source == "max" and two_pitch is not None and end_pitch is not None:
        return max(two_pitch[index], end_pitch[index])
    if end_pitch is not None:
        return end_pitch[index]
    if two_pitch is not None:
        return two_pitch[index]
    return pitch[index]


def extract_l20_g7s_arrays(
    hand_data: Any,
    thumb_mode: str = DEFAULT_THUMB_MODE,
    thumb_abd_source: str = "neutral",
    thumb_tip_source: str = "end",
) -> dict[str, float]:
    pitch = find_number_array(hand_data, G7S_ARRAY_ALIASES["pitch"], 5)
    if pitch is None:
        return {}

    side = find_number_array(hand_data, G7S_ARRAY_ALIASES["side"], 5) or [0.0] * 5
    roll = find_number_array(hand_data, ("roll", "Roll", "fingerRoll", "finger_roll"), 5) or [0.0] * 5
    two_pitch = find_number_array(hand_data, ("two_pitch", "twoPitch", "pip", "middlePitch"), 5)
    end_pitch = find_number_array(hand_data, ("end_pitch", "endPitch", "dip", "tipPitch"), 5)

    if thumb_mode == "side-pitch":
        thumb_root1 = side[0]
    else:
        thumb_root1 = pitch[0]
    thumb_abd = choose_thumb_abd(pitch, side, roll, thumb_mode, thumb_abd_source)

    return {
        "thumb_abd": thumb_abd,
        "thumb_yaw": roll[0],
        "thumb_root1": thumb_root1,
        "thumb_tip": choose_l20_tip(two_pitch, end_pitch, pitch, 0, thumb_tip_source),
        "index_abd": side[1],
        "index_root1": pitch[1],
        "index_tip": choose_l20_tip(two_pitch, end_pitch, pitch, 1, "max"),
        "middle_abd": side[2],
        "middle_root1": pitch[2],
        "middle_tip": choose_l20_tip(two_pitch, end_pitch, pitch, 2, "max"),
        "ring_abd": side[3],
        "ring_root1": pitch[3],
        "ring_tip": choose_l20_tip(two_pitch, end_pitch, pitch, 3, "max"),
        "pinky_abd": side[4],
        "pinky_root1": pitch[4],
        "pinky_tip": choose_l20_tip(two_pitch, end_pitch, pitch, 4, "max"),
    }


def extract_l6_joint_values(
    hand_data: Any,
    mapping: Mapping[str, str] | None,
    thumb_mode: str = DEFAULT_THUMB_MODE,
    thumb_abd_source: str = "neutral",
) -> dict[str, float]:
    extracted = extract_by_mapping(hand_data, mapping, L6_JOINTS) if mapping else {}

    if len(extracted) < len(L6_JOINTS):
        g7s_values = extract_g7s_arrays(
            hand_data,
            thumb_mode=thumb_mode,
            thumb_abd_source=thumb_abd_source,
        )
        extracted.update({joint: value for joint, value in g7s_values.items() if joint not in extracted})

    if len(extracted) < len(L6_JOINTS):
        alias_values = extract_by_aliases(hand_data)
        extracted.update({joint: value for joint, value in alias_values.items() if joint not in extracted})

    if len(extracted) < len(L6_JOINTS):
        values = find_numeric_list(hand_data)
        if values is not None:
            extracted.update(
                {joint: values[index] for index, joint in enumerate(L6_JOINTS) if joint not in extracted}
            )

    missing = [joint for joint in L6_JOINTS if joint not in extracted]
    if missing:
        raise MissingJointsError(missing, list(flatten_numbers(hand_data).keys()))

    return {joint: extracted[joint] for joint in L6_JOINTS}


def extract_l20_joint_values(
    hand_data: Any,
    mapping: Mapping[str, str] | None,
    thumb_mode: str = DEFAULT_THUMB_MODE,
    thumb_abd_source: str = "neutral",
    thumb_tip_source: str = "end",
) -> dict[str, float]:
    extracted = extract_by_mapping(hand_data, mapping, L20_JOINTS) if mapping else {}

    if len(extracted) < len(L20_JOINTS):
        g7s_values = extract_l20_g7s_arrays(
            hand_data,
            thumb_mode=thumb_mode,
            thumb_abd_source=thumb_abd_source,
            thumb_tip_source=thumb_tip_source,
        )
        extracted.update({joint: value for joint, value in g7s_values.items() if joint not in extracted})

    if len(extracted) < len(L20_JOINTS):
        values = find_numeric_list(hand_data, length=len(L20_JOINTS))
        if values is not None:
            extracted.update(
                {joint: values[index] for index, joint in enumerate(L20_JOINTS) if joint not in extracted}
            )

    missing = [joint for joint in L20_JOINTS if joint not in extracted]
    if missing:
        raise MissingJointsError(missing, list(flatten_numbers(hand_data).keys()))

    return {joint: extracted[joint] for joint in L20_JOINTS}


def extract_joint_values(
    hand_data: Any,
    mapping: Mapping[str, str] | None,
    hand_model: str = "l6",
    thumb_mode: str = DEFAULT_THUMB_MODE,
    thumb_abd_source: str = "neutral",
    l20_thumb_tip_source: str = "end",
) -> dict[str, float]:
    if hand_model == "l20":
        return extract_l20_joint_values(
            hand_data,
            mapping,
            thumb_mode=thumb_mode,
            thumb_abd_source=thumb_abd_source,
            thumb_tip_source=l20_thumb_tip_source,
        )
    return extract_l6_joint_values(
        hand_data,
        mapping,
        thumb_mode=thumb_mode,
        thumb_abd_source=thumb_abd_source,
    )


def glove_to_l6_angles(
    values: Mapping[str, float],
    ranges: Mapping[str, tuple[float, float]],
    inverted: set[str],
) -> list[float]:
    return glove_to_normalized_angles(values, ranges, inverted, L6_JOINTS)


def glove_to_normalized_angles(
    values: Mapping[str, float],
    ranges: Mapping[str, tuple[float, float]],
    inverted: set[str],
    joint_order: tuple[str, ...],
) -> list[float]:
    angles: list[float] = []
    for joint in joint_order:
        raw = values[joint]
        low, high = ranges[joint]
        if high == low:
            raise ValueError(f"range for {joint} has identical min/max")
        percent = (raw - low) * 100.0 / (high - low)
        percent = clamp(percent, 0.0, 100.0)
        if joint in inverted:
            percent = 100.0 - percent
        angles.append(percent)
    return angles


def smooth_angles(previous: list[float] | None, current: list[float], alpha: float) -> list[float]:
    if previous is None or alpha >= 1.0:
        return current
    return [old * (1.0 - alpha) + new * alpha for old, new in zip(previous, current)]


def should_send(previous: list[float] | None, current: list[float], deadband: float) -> bool:
    if previous is None:
        return True
    return max(abs(old - new) for old, new in zip(previous, current)) >= deadband


def parse_datagram(data: bytes) -> Any:
    text = data.decode("utf-8-sig", errors="replace").strip()
    if not text:
        raise ValueError("empty datagram")
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        start = text.find("{")
        end = text.rfind("}")
        if start >= 0 and end > start:
            return json.loads(text[start : end + 1])
        raise


class GloveUdpClient:
    def __init__(
        self,
        glove_ip: str,
        glove_port: int,
        local_host: str,
        local_port: int,
        timeout: float,
        connect_format: str,
    ) -> None:
        self.glove_addr = (glove_ip, glove_port)
        self.local_addr = (local_host, local_port)
        self.timeout = timeout
        self.connect_format = connect_format
        self.sock: socket.socket | None = None

    def __enter__(self) -> "GloveUdpClient":
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(self.local_addr)
        sock.settimeout(self.timeout)
        self.sock = sock
        self.connect()
        logging.info("UDP listening on %s, connected to %s", sock.getsockname(), self.glove_addr)
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        try:
            self.disconnect()
        finally:
            if self.sock is not None:
                self.sock.close()

    def recv(self) -> bytes:
        if self.sock is None:
            raise RuntimeError("UDP client is not open")
        data, _addr = self.sock.recvfrom(65535)
        return data

    def connect(self) -> None:
        self._send_action("CONNECT")

    def disconnect(self) -> None:
        self._send_action("DisConnect")

    def _send_action(self, action: str) -> None:
        if self.sock is None or self.connect_format == "none":
            return
        payloads: list[bytes] = []
        if self.connect_format in ("json", "both"):
            payloads.append(json.dumps({"action": action}, separators=(",", ":")).encode("utf-8"))
        if self.connect_format in ("text", "both"):
            payloads.append(action.encode("utf-8"))
        for payload in payloads:
            self.sock.sendto(payload, self.glove_addr)


class RealHandController:
    def __init__(
        self,
        name: str,
        hand_model: str,
        side: str,
        can_interface: str,
        can_type: str,
        dry_run: bool,
        torque: float,
        open_on_start: bool,
        open_on_exit: bool,
    ) -> None:
        self.name = name
        self.hand_model = hand_model
        self.side = side
        self.can_interface = can_interface
        self.can_type = can_type
        self.dry_run = dry_run
        self.torque = torque
        self.open_on_start = open_on_start
        self.open_on_exit = open_on_exit
        self.hand: Any = None

    def __enter__(self) -> "RealHandController":
        if self.dry_run:
            logging.info(
                "%s dry run enabled; RealHand %s will not be opened, torque would be %.1f",
                self.name,
                self.hand_model.upper(),
                self.torque,
            )
            return self
        if self.hand_model == "l20":
            from realhand.hand.l20 import L20

            hand_class = L20
        elif self.hand_model == "o6":
            from realhand.hand.o6 import O6

            hand_class = O6
        else:
            from realhand.hand.l6 import L6

            hand_class = L6

        self.hand = hand_class(side=self.side, interface_name=self.can_interface, interface_type=self.can_type)
        self.hand.stop_polling()
        self.set_torque()
        if self.open_on_start:
            self.send(MODEL_OPEN_POSES[self.hand_model])
            time.sleep(0.2)
        logging.info(
            "opened %s RealHand %s side=%s on %s/%s",
            self.name,
            self.hand_model.upper(),
            self.side,
            self.can_type,
            self.can_interface,
        )
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        if self.hand is None:
            return
        try:
            if self.open_on_exit:
                self.send(MODEL_OPEN_POSES[self.hand_model])
                time.sleep(0.2)
        finally:
            self.hand.close()

    def send(self, angles: list[float]) -> None:
        rounded = [round(clamp(value, 0.0, 100.0), 2) for value in angles]
        if self.dry_run:
            logging.info("dry-run %s %s angles=%s", self.name, self.hand_model.upper(), rounded)
            return
        self.hand.angle.set_angles(rounded)

    def set_torque(self) -> None:
        torques = [round(clamp(self.torque, 0.0, 100.0), 2)] * len(MODEL_JOINTS[self.hand_model])
        self.hand.torque.set_torques(torques)
        logging.info("%s %s torque=%s", self.name, self.hand_model.upper(), torques)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control a RealHand L6/L20/O6 from G7s UDP glove data.")
    parser.add_argument("--glove-ip", required=True, help="G7s host IP, for example 192.168.0.5.")
    parser.add_argument("--glove-port", type=int, default=9011, help="G7s UDP command/data port.")
    parser.add_argument("--local-host", default="0.0.0.0", help="Local UDP bind address.")
    parser.add_argument("--local-port", type=int, default=0, help="Local UDP bind port. 0 lets the OS choose.")
    parser.add_argument("--connect-format", choices=("json", "text", "both", "none"), default="both")

    parser.add_argument("--glove-hand", choices=("left", "right"), default="right")
    parser.add_argument("--l6-side", "--hand-side", choices=("left", "right"), default="right")
    parser.add_argument("--hand-model", choices=("l6", "l20", "o6"), default="l6")
    parser.add_argument(
        "--g7s-thumb-mode",
        choices=("side-pitch", "pitch-side"),
        default=DEFAULT_THUMB_MODE,
        help="G7s thumb axis mapping: pitch-side maps pitch[0] to thumb flex/root and side[0] to thumb abduction.",
    )
    parser.add_argument("--can-interface", default="can0")
    parser.add_argument("--both-hands", action="store_true", help="Control both leftHand and rightHand from one UDP stream.")
    parser.add_argument("--left-can-interface", default="can0", help="CAN interface for the left hand when --both-hands is used.")
    parser.add_argument("--right-can-interface", default="can1", help="CAN interface for the right hand when --both-hands is used.")
    parser.add_argument("--left-l6-side", choices=("left", "right"), default="left")
    parser.add_argument("--right-l6-side", choices=("left", "right"), default="right")
    parser.add_argument("--left-hand-model", choices=("l6", "l20", "o6"), help="Override --hand-model for leftHand.")
    parser.add_argument("--right-hand-model", choices=("l6", "l20", "o6"), help="Override --hand-model for rightHand.")
    parser.add_argument("--can-type", default="socketcan")
    parser.add_argument("--torque", type=float, default=50.0, help="Joint torque limit for every joint, 0..100. Default is half torque.")

    parser.add_argument("--mapping", type=lambda value: parse_json_option(value, name="--mapping"))
    parser.add_argument("--mapping-file", help="JSON file mapping model joints to glove field paths.")
    parser.add_argument("--ranges", type=lambda value: parse_json_option(value, name="--ranges"))
    parser.add_argument("--ranges-file", help="JSON file with per-joint [min, max] input ranges.")
    parser.add_argument(
        "--invert",
        default=None,
        help="Comma-separated joints whose input direction should be inverted.",
    )
    parser.add_argument(
        "--invert-thumb",
        action="store_true",
        help="Flip only the primary thumb flex/root joint after the default inversion set is chosen.",
    )
    parser.add_argument(
        "--thumb-abd-source",
        choices=("neutral", "auto", "side", "pitch", "roll"),
        default="neutral",
        help="Input source for L6/O6 thumb_abd and L20 thumb_abd. Default neutral avoids sticking to a constant side[0].",
    )
    parser.add_argument(
        "--left-thumb-abd-source",
        choices=("neutral", "auto", "side", "pitch", "roll"),
        help="Override --thumb-abd-source for leftHand when --both-hands is used.",
    )
    parser.add_argument(
        "--right-thumb-abd-source",
        choices=("neutral", "auto", "side", "pitch", "roll"),
        help="Override --thumb-abd-source for rightHand when --both-hands is used.",
    )
    parser.add_argument(
        "--l20-thumb-tip-source",
        choices=("end", "two", "max", "pitch"),
        default="end",
        help="Input source for L20 thumb_tip. Default end uses end_pitch[0].",
    )

    parser.add_argument("--rate-hz", type=float, default=50.0, help="Maximum hand command rate.")
    parser.add_argument("--smoothing-alpha", type=float, default=0.35, help="EMA alpha in 0..1; 1 disables smoothing.")
    parser.add_argument("--deadband", type=float, default=0.8, help="Minimum normalized angle delta before sending.")
    parser.add_argument("--timeout", type=float, default=1.0, help="UDP receive timeout in seconds.")
    parser.add_argument("--stats-interval", type=float, default=2.0)

    parser.add_argument("--dry-run", action="store_true", help="Receive and map data without opening or moving the hand.")
    parser.add_argument("--show-fields", action="store_true", help="Print numeric paths from the first frame, then exit.")
    parser.add_argument("--show-thumb", action="store_true", help="Log raw and mapped thumb values while running.")
    parser.add_argument("--open-on-start", action="store_true", help="Send the model open pose once after opening the hand.")
    parser.add_argument("--open-on-exit", action="store_true", help="Send the model open pose before closing the hand.")
    parser.add_argument("--debug", action="store_true")
    return parser


def parse_mapping(args: argparse.Namespace) -> dict[str, str] | None:
    mapping = args.mapping or load_json_file(args.mapping_file)
    if mapping is None:
        return None
    if not isinstance(mapping, Mapping):
        raise ValueError("mapping must be a JSON object")
    known = {joint for joints in MODEL_JOINTS.values() for joint in joints}
    unknown = set(mapping) - known
    if unknown:
        raise ValueError(f"unknown mapping joints: {', '.join(sorted(unknown))}")
    return {str(key): str(value) for key, value in mapping.items()}


def parse_ranges(args: argparse.Namespace, hand_model: str, thumb_mode: str) -> dict[str, tuple[float, float]]:
    loaded = args.ranges or load_json_file(args.ranges_file) or {}
    if not isinstance(loaded, Mapping):
        raise ValueError("ranges must be a JSON object")
    joint_order = MODEL_JOINTS[hand_model]
    ranges = dict(MODEL_DEFAULT_RANGES[(hand_model, thumb_mode)])
    for joint, pair in loaded.items():
        if joint not in joint_order:
            continue
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise ValueError(f"range for {joint} must be [min, max]")
        ranges[joint] = (float(pair[0]), float(pair[1]))
    return ranges


def parse_inverted(value: str | None, hand_model: str, thumb_mode: str) -> set[str]:
    if value is None:
        return set(MODEL_DEFAULT_INVERTED[(hand_model, thumb_mode)])
    if not value:
        return set()
    joints = {item.strip() for item in value.split(",") if item.strip()}
    unknown = joints - set(MODEL_JOINTS[hand_model])
    if unknown:
        raise ValueError(f"unknown inverted joints: {', '.join(sorted(unknown))}")
    return joints


def build_hand_configs(args: argparse.Namespace) -> list[dict[str, str]]:
    if args.both_hands:
        return [
            {
                "name": "left",
                "glove_hand": "left",
                "hand_model": args.left_hand_model or args.hand_model,
                "thumb_mode": args.g7s_thumb_mode,
                "thumb_abd_source": args.left_thumb_abd_source or args.thumb_abd_source,
                "l20_thumb_tip_source": args.l20_thumb_tip_source,
                "l6_side": args.left_l6_side,
                "can_interface": args.left_can_interface,
            },
            {
                "name": "right",
                "glove_hand": "right",
                "hand_model": args.right_hand_model or args.hand_model,
                "thumb_mode": args.g7s_thumb_mode,
                "thumb_abd_source": args.right_thumb_abd_source or args.thumb_abd_source,
                "l20_thumb_tip_source": args.l20_thumb_tip_source,
                "l6_side": args.right_l6_side,
                "can_interface": args.right_can_interface,
            },
        ]

    return [
        {
            "name": args.glove_hand,
            "glove_hand": args.glove_hand,
            "hand_model": args.hand_model,
            "thumb_mode": args.g7s_thumb_mode,
            "thumb_abd_source": args.thumb_abd_source,
            "l20_thumb_tip_source": args.l20_thumb_tip_source,
            "l6_side": args.l6_side,
            "can_interface": args.can_interface,
        }
    ]


def log_stats(stats: Stats) -> None:
    angles = {
        hand: [round(value, 1) for value in values]
        for hand, values in sorted(stats.last_angles.items())
    }
    logging.info(
        "stats received=%d parsed=%d sent=%d sent_by_hand=%s parse_errors=%d map_errors=%d map_errors_by_hand=%s last=%s",
        stats.received,
        stats.parsed,
        stats.sent,
        dict(sorted(stats.sent_by_hand.items())),
        stats.parse_errors,
        stats.map_errors,
        dict(sorted(stats.map_errors_by_hand.items())),
        angles,
    )


def rounded_array(values: list[float] | None) -> list[float] | None:
    if values is None:
        return None
    return [round(value, 2) for value in values[:5]]


def thumb_raw_values(hand_data: Any) -> dict[str, float | list[float] | None]:
    pitch = find_number_array(hand_data, G7S_ARRAY_ALIASES["pitch"], 5)
    side = find_number_array(hand_data, G7S_ARRAY_ALIASES["side"], 5)
    roll = find_number_array(hand_data, ("roll", "Roll", "fingerRoll", "finger_roll"), 5)
    two_pitch = find_number_array(hand_data, ("two_pitch", "twoPitch", "pip", "middlePitch"), 5)
    end_pitch = find_number_array(hand_data, ("end_pitch", "endPitch", "dip", "tipPitch"), 5)
    return {
        "pitch0": None if pitch is None else round(pitch[0], 2),
        "side0": None if side is None else round(side[0], 2),
        "roll0": None if roll is None else round(roll[0], 2),
        "two_pitch0": None if two_pitch is None else round(two_pitch[0], 2),
        "end_pitch0": None if end_pitch is None else round(end_pitch[0], 2),
        "pitch": rounded_array(pitch),
        "side": rounded_array(side),
        "roll": rounded_array(roll),
        "two_pitch": rounded_array(two_pitch),
        "end_pitch": rounded_array(end_pitch),
    }


def thumb_angle_summary(hand_model: str, angles: list[float]) -> dict[str, float]:
    if hand_model == "l20":
        names = ("thumb_abd", "thumb_yaw", "thumb_root1", "thumb_tip")
        return {name: round(angles[index], 2) for index, name in enumerate(names)}
    return {
        "thumb_flex": round(angles[0], 2),
        "thumb_abd": round(angles[1], 2),
    }


def run(args: argparse.Namespace) -> int:
    mapping = parse_mapping(args)

    if args.rate_hz <= 0:
        raise ValueError("--rate-hz must be positive")
    if not 0.0 < args.smoothing_alpha <= 1.0:
        raise ValueError("--smoothing-alpha must be in (0, 1]")
    if args.deadband < 0:
        raise ValueError("--deadband must be non-negative")
    if not 0.0 <= args.torque <= 100.0:
        raise ValueError("--torque must be in [0, 100]")

    stop = False

    def request_stop(_signum: int, _frame: Any) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    stats = Stats()
    last_stats_time = time.monotonic()
    min_interval = 1.0 / args.rate_hz
    hand_configs = build_hand_configs(args)
    ranges_by_model = {
        (config["hand_model"], config["thumb_mode"]): parse_ranges(
            args,
            config["hand_model"],
            config["thumb_mode"],
        )
        for config in hand_configs
    }
    inverted_by_model = {
        (config["hand_model"], config["thumb_mode"]): parse_inverted(
            args.invert,
            config["hand_model"],
            config["thumb_mode"],
        )
        for config in hand_configs
    }
    if args.invert_thumb:
        for key, inverted in inverted_by_model.items():
            hand_model, thumb_mode = key
            joint = "thumb_root1" if hand_model == "l20" and thumb_mode == "side-pitch" else "thumb_flex"
            if hand_model == "l20" and thumb_mode == "pitch-side":
                joint = "thumb_root1"
            if joint in inverted:
                inverted.remove(joint)
            else:
                inverted.add(joint)

    with ExitStack() as stack:
        udp = stack.enter_context(
            GloveUdpClient(
                glove_ip=args.glove_ip,
                glove_port=args.glove_port,
                local_host=args.local_host,
                local_port=args.local_port,
                timeout=args.timeout,
                connect_format=args.connect_format,
            )
        )
        hands = [
            HandRuntime(
                name=config["name"],
                glove_hand=config["glove_hand"],
                hand_model=config["hand_model"],
                thumb_mode=config["thumb_mode"],
                thumb_abd_source=config["thumb_abd_source"],
                l20_thumb_tip_source=config["l20_thumb_tip_source"],
                ranges=ranges_by_model[(config["hand_model"], config["thumb_mode"])],
                inverted=inverted_by_model[(config["hand_model"], config["thumb_mode"])],
                controller=stack.enter_context(
                    RealHandController(
                        name=config["name"],
                        hand_model=config["hand_model"],
                        side=config["l6_side"],
                        can_interface=config["can_interface"],
                        can_type=args.can_type,
                        dry_run=args.dry_run,
                        torque=args.torque,
                        open_on_start=args.open_on_start,
                        open_on_exit=args.open_on_exit,
                    )
                ),
            )
            for config in hand_configs
        ]

        while not stop:
            try:
                data = udp.recv()
            except socket.timeout:
                udp.connect()
                now = time.monotonic()
                if now - last_stats_time >= args.stats_interval:
                    log_stats(stats)
                    last_stats_time = now
                continue

            stats.received += 1
            try:
                frame = parse_datagram(data)
                stats.parsed += 1
            except Exception as exc:
                stats.parse_errors += 1
                if args.debug:
                    logging.warning("failed to parse UDP datagram: %s", exc)
                continue

            if args.show_fields:
                output = {
                    hand.glove_hand: list(
                        flatten_numbers(select_hand_for_mode(frame, hand.glove_hand, args.both_hands) or {}).keys()
                    )
                    for hand in hands
                }
                print(json.dumps(output, indent=2))
                return 0

            now = time.monotonic()
            for hand in hands:
                hand_data = select_hand_for_mode(frame, hand.glove_hand, args.both_hands)
                if hand_data is None:
                    continue
                try:
                    joint_values = extract_joint_values(
                        hand_data,
                        mapping,
                        hand.hand_model,
                        thumb_mode=hand.thumb_mode,
                        thumb_abd_source=hand.thumb_abd_source,
                        l20_thumb_tip_source=hand.l20_thumb_tip_source,
                    )
                    target = glove_to_normalized_angles(
                        joint_values,
                        hand.ranges,
                        hand.inverted,
                        MODEL_JOINTS[hand.hand_model],
                    )
                    if args.show_thumb and now - hand.last_thumb_log_time >= 0.5:
                        logging.info(
                            "%s thumb raw=%s mapped=%s mode=%s abd_source=%s tip_source=%s inverted=%s",
                            hand.name,
                            thumb_raw_values(hand_data),
                            thumb_angle_summary(hand.hand_model, target),
                            hand.thumb_mode,
                            hand.thumb_abd_source,
                            hand.l20_thumb_tip_source,
                            sorted(hand.inverted),
                        )
                        hand.last_thumb_log_time = now
                except MissingJointsError as exc:
                    stats.map_errors += 1
                    stats.map_errors_by_hand[hand.name] = stats.map_errors_by_hand.get(hand.name, 0) + 1
                    if now - hand.last_map_error > 2.0:
                        logging.warning("%s: %s", hand.name, exc)
                        hand.last_map_error = now
                    continue

                hand.smoothed = smooth_angles(hand.smoothed, target, args.smoothing_alpha)
                if now - hand.last_send_time < min_interval:
                    continue
                if not should_send(hand.last_sent, hand.smoothed, args.deadband):
                    continue

                hand.controller.send(hand.smoothed)
                hand.last_sent = list(hand.smoothed)
                hand.last_send_time = now
                stats.sent += 1
                stats.sent_by_hand[hand.name] = stats.sent_by_hand.get(hand.name, 0) + 1
                stats.last_angles[hand.name] = list(hand.smoothed)

            if now - last_stats_time >= args.stats_interval:
                log_stats(stats)
                last_stats_time = now

    log_stats(stats)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.debug else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    try:
        return run(args)
    except Exception as exc:
        logging.error("%s", exc)
        if args.debug:
            raise
        return 1


if __name__ == "__main__":
    sys.exit(main())
