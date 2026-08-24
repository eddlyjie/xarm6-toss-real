from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "object_commissioning_bundle",
    ROOT / "scripts/29_prepare_object_commissioning.py",
)
commissioning = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(commissioning)


def build(object_key="O1", label="day1"):
    return commissioning.build_bundle(
        object_key=object_key,
        label=label,
        held_position=360,
        release_position=540,
        preclose_position=430,
        close_position=360,
    )


@pytest.mark.parametrize("object_key", ["O1", "O2", "O3"])
def test_one_measurement_set_builds_three_staged_profiles(object_key):
    result = build(object_key)
    bundle = result["bundle"]

    assert bundle["offline_generated"] is True
    assert bundle["robot_connection_attempted"] is False
    assert list(bundle["profiles"]) == ["empty_g1", "throw_only", "object"]
    assert len(result["files"]) == 7

    payloads = {item["path"]: item["payload"] for item in result["files"]}
    profiles = [payloads[path] for path in bundle["profiles"].values()]
    assert [profile["hardware_modes_allowed"] for profile in profiles] == [
        ["empty_arm", "empty_g1"],
        ["empty_arm", "empty_g1", "throw_only"],
        ["empty_arm", "empty_g1", "throw_only", "object"],
    ]
    assert all(profile["g1"]["held_position"] == 360 for profile in profiles)
    assert all(
        [event["position"] for event in profile["g1"]["events"]]
        == [540, 430, 360]
        for profile in profiles
    )


def test_bundle_contains_copyable_execution_order():
    bundle = build()["bundle"]
    commands = bundle["execution_order"]

    assert [row["step"] for row in commands] == [
        "plan_only",
        "empty_arm_0.25x",
        "empty_arm_0.5x",
        "empty_arm_1x",
        "empty_g1",
        "soft_mat_throw_only",
        "guarded_object_recatch",
    ]
    assert commands[0]["connects_robot"] is False
    assert all(row["connects_robot"] for row in commands[1:])
    assert "--execute-object" in commands[-1]["command"]


def test_write_bundle_creates_valid_files_and_refuses_overwrite(tmp_path, monkeypatch):
    result = build()
    monkeypatch.setattr(commissioning, "prepare_deployment", lambda root, profile_path, mode: (
        {"object_id": result["bundle"]["object_id"]},
        [],
    ))

    paths = commissioning.write_bundle(result, root=tmp_path)
    assert len(paths) == 7
    assert all(path.is_file() for path in paths)
    bundle_path = next(path for path in paths if path.name == "commissioning_bundle.json")
    saved = json.loads(bundle_path.read_text(encoding="utf-8"))
    assert saved == result["bundle"]
    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        commissioning.write_bundle(result, root=tmp_path)


@pytest.mark.parametrize("label", ["../escape", "spaces are bad", ""])
def test_label_is_a_simple_path_component(label):
    with pytest.raises(ValueError, match="label"):
        build(label=label)
