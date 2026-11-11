"""Recorded joint configurations for the three workcell buttons.

Positions describe the center of each cap's exposed surface in world meters.
The six angles put the button_tip site at that point, facing into the cap.
The positive-elbow inverse-kinematics branch is shared by all three targets.
"""
import json
from pathlib import Path

BUTTON_NAMES = ("red", "yellow", "green")
TARGETS_PATH = Path(__file__).with_name("button_targets.json")


def button_target(name):
    if name not in BUTTON_NAMES:
        raise ValueError(f"button must be one of {BUTTON_NAMES}")
    with TARGETS_PATH.open() as stream:
        return json.load(stream)[name]
