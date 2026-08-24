from __future__ import annotations

import importlib.metadata
import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "real_robot_environment", ROOT / "scripts/28_check_real_robot_environment.py"
)
preflight = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(preflight)


GOOD_VERSIONS = {
    "xarm-python-sdk": "1.17.0",
    "numpy": "2.1.0",
    "scipy": "1.14.1",
    "opencv-python-headless": "5.0.0.93",
}


def resolver(versions):
    def resolve(name):
        if name not in versions:
            raise importlib.metadata.PackageNotFoundError(name)
        return versions[name]

    return resolve


def test_preflight_is_offline_and_reports_real_commissioning_state():
    report = preflight.build_report(package_resolver=resolver(GOOD_VERSIONS))

    assert report["offline_only"] is True
    assert report["robot_connection_attempted"] is False
    assert report["video_angle_runtime"]["ok"] is True
    assert report["environment_ready"] is True
    assert report["ready_for_o0_staged_execution"] is True
    assert report["handoff"]["files_and_joint_envelopes_ready"] is True
    assert report["handoff"]["g1_calibration_complete"] == {
        "O0": True,
        "O1": False,
        "O2": False,
        "O3": False,
    }
    assert report["handoff"]["staged_execution_ready"] == {
        "O0": True,
        "O1": False,
        "O2": False,
        "O3": False,
    }
    assert report["handoff"]["objects_awaiting_g1_calibration"] == [
        "O1",
        "O2",
        "O3",
    ]


def test_missing_sdk_blocks_hardware_readiness_without_importing_it():
    versions = {
        "numpy": "2.1.0",
        "scipy": "1.14.1",
        "opencv-python-headless": "5.0.0.93",
    }
    report = preflight.build_report(package_resolver=resolver(versions))

    sdk = next(row for row in report["packages"] if row["name"] == "xarm-python-sdk")
    assert sdk["installed"] is None
    assert sdk["ok"] is False
    assert report["environment_ready"] is False
    assert report["ready_for_o0_staged_execution"] is False


def test_missing_aruco_runtime_blocks_environment(monkeypatch):
    monkeypatch.setattr(
        preflight,
        "check_video_angle_runtime",
        lambda: {
            "ok": False,
            "version": "4.6.0",
            "required_apis": ["aruco.ArucoDetector", "aruco.generateImageMarker"],
            "detail": "missing OpenCV APIs",
        },
    )
    report = preflight.build_report(package_resolver=resolver(GOOD_VERSIONS))
    assert report["environment_ready"] is False
    assert "[FAIL] OpenCV ArUco runtime" in preflight.render_summary(report)


def test_wrong_g1_speed_is_reported_before_hardware_use(tmp_path):
    source = json.loads(preflight.DEFAULT_HARDWARE.read_text(encoding="utf-8"))
    source["gripper"]["speed"] = 3000
    hardware = tmp_path / "hardware.json"
    hardware.write_text(json.dumps(source), encoding="utf-8")

    report = preflight.build_report(
        hardware_path=hardware,
        package_resolver=resolver(GOOD_VERSIONS),
    )

    assert report["hardware"]["configuration_valid"] is False
    assert report["hardware"]["problems"] == [
        "G1 speed must be 5000 for the current profiles"
    ]
    assert report["environment_ready"] is False


def test_summary_distinguishes_ready_o0_from_uncalibrated_cuboids():
    report = preflight.build_report(package_resolver=resolver(GOOD_VERSIONS))
    summary = preflight.render_summary(report)

    assert "[PASS] O0 G1: ready" in summary
    assert "[WAIT] O1 G1: onsite calibration required" in summary
    assert "No robot connection or command was attempted." in summary


def test_discovery_requires_a_complete_three_stage_bundle(tmp_path):
    base = tmp_path / "real_handoff/cuboid30/low/day1"
    base.mkdir(parents=True)
    profiles = {
        "empty_g1": "configs/real/O1/day1/empty_g1.json",
        "throw_only": "configs/real/O1/day1/throw_only.json",
        "object": "configs/real/O1/day1/object.json",
    }
    bundle = {
        "schema": "xarm6_object_commissioning_bundle_v1",
        "object_key": "O1",
        "label": "day1",
        "profiles": profiles,
    }
    (base / "commissioning_bundle.json").write_text(json.dumps(bundle))

    missing = preflight.discover_commissioning_bundles(tmp_path)
    assert missing["O1"]["ready"] is False

    for relative in profiles.values():
        path = tmp_path / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("{}")
    complete = preflight.discover_commissioning_bundles(tmp_path)
    assert complete["O1"]["ready"] is True
    assert complete["O1"]["latest"]["label"] == "day1"
    assert complete["O2"]["ready"] is False


def test_report_marks_generated_bundle_ready(monkeypatch):
    discovered = {
        key: {"ready": False, "latest": None, "bundles": []}
        for key in ("O1", "O2", "O3")
    }
    discovered["O1"] = {
        "ready": True,
        "latest": {"label": "day1", "bundle": "bundle.json", "profiles": {}},
        "bundles": [],
    }
    monkeypatch.setattr(preflight, "discover_commissioning_bundles", lambda root: discovered)

    report = preflight.build_report(package_resolver=resolver(GOOD_VERSIONS))
    assert report["handoff"]["staged_execution_ready"]["O1"] is True
    assert report["handoff"]["objects_awaiting_g1_calibration"] == ["O2", "O3"]
    summary = preflight.render_summary(report)
    assert "[PASS] O1 G1: ready via commissioning bundle day1" in summary
