from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def load_script(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


commissioning = load_script(
    "commissioning_for_pose_ladder_test",
    ROOT / "scripts/29_prepare_object_commissioning.py",
)
ladder = load_script(
    "pose_ladder_bundle",
    ROOT / "scripts/32_prepare_pose_ladder.py",
)


def commissioning_file(tmp_path: Path, object_key: str = "O1") -> Path:
    tmp_path.mkdir(parents=True, exist_ok=True)
    result = commissioning.build_bundle(
        object_key=object_key,
        label="day1",
        held_position=360,
        release_position=540,
        preclose_position=430,
        close_position=360,
    )
    path = tmp_path / "commissioning_bundle.json"
    path.write_text(json.dumps(result["bundle"]), encoding="utf-8")
    return path


@pytest.mark.parametrize("object_key", ["O1", "O2", "O3"])
def test_low_commissioning_generates_next_and_high_profiles(tmp_path, object_key):
    result = ladder.build_pose_ladder(commissioning_file(tmp_path, object_key))
    bundle = result["bundle"]

    assert list(bundle["poses"]) == ["next", "high"]
    assert len(result["files"]) == 13
    assert bundle["g1"]["held_position"] == 360
    payloads = {item["path"]: item["payload"] for item in result["files"]}
    for pose in bundle["poses"].values():
        assert pose["desired_angle_deg"] > 0.0
        for stage, profile_path in pose["profiles"].items():
            profile = payloads[profile_path]
            assert profile["g1"]["held_position"] == 360
            assert [event["position"] for event in profile["g1"]["events"]] == [
                540,
                430,
                360,
            ]
            assert stage in profile_path


def test_each_pose_contains_the_full_staged_command_ladder(tmp_path):
    bundle = ladder.build_pose_ladder(commissioning_file(tmp_path))["bundle"]
    expected = [
        "plan_only",
        "empty_arm_0.25x",
        "empty_arm_0.5x",
        "empty_arm_1x",
        "empty_g1",
        "soft_mat_throw_only",
        "guarded_object_recatch",
    ]
    for pose in bundle["poses"].values():
        commands = pose["execution_order"]
        assert [row["step"] for row in commands] == expected
        assert commands[0]["connects_robot"] is False
        assert all(row["connects_robot"] for row in commands[1:])


def test_write_creates_valid_payload_set_and_refuses_overwrite(tmp_path, monkeypatch):
    result = ladder.build_pose_ladder(commissioning_file(tmp_path / "input"))
    monkeypatch.setattr(
        ladder,
        "prepare_deployment",
        lambda root, profile_path, mode: (
            {"object_id": result["bundle"]["object_id"]},
            [],
        ),
    )
    output_root = tmp_path / "output"
    paths = ladder.write_pose_ladder(result, root=output_root)
    assert len(paths) == 13
    assert all(path.is_file() for path in paths)
    with pytest.raises(FileExistsError, match="refusing to overwrite"):
        ladder.write_pose_ladder(result, root=output_root)


def test_o0_or_wrong_object_bundle_is_not_used_for_cuboid_ladder(tmp_path):
    path = tmp_path / "commissioning_bundle.json"
    path.write_text(
        json.dumps(
            {
                "schema": "xarm6_object_commissioning_bundle_v1",
                "object_key": "O0",
                "object_id": "yellow_cube_38mm_8g",
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="O1, O2, or O3"):
        ladder.build_pose_ladder(path)
