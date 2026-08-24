from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "onsite_progress", ROOT / "scripts/33_show_onsite_progress.py"
)
progress = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(progress)


def write_commissioning(root: Path, object_key: str, name: str, label: str = "day1") -> Path:
    profiles = {}
    for stage in ("empty_g1", "throw_only", "object"):
        relative = f"configs/real/{object_key}/{label}/{stage}.json"
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("{}\n")
        profiles[stage] = relative
    bundle = {
        "schema": "xarm6_object_commissioning_bundle_v1",
        "object_key": object_key,
        "label": label,
        "profiles": profiles,
        "execution_order": [
            {
                "step": "plan_only",
                "command": f"python runner.py --profile {profiles['empty_g1']}",
            }
        ],
    }
    path = root / f"real_handoff/{name}/low/{label}/commissioning_bundle.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(bundle))
    return path


def write_trial(
    root: Path,
    object_key: str,
    profile: str,
    trial_id: str,
    *,
    success: bool = True,
) -> None:
    path = root / "real_results" / object_key / f"{trial_id}.trial.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            {
                "schema": progress.TRIAL_SCHEMA,
                "object_key": object_key,
                "profile": profile,
                "trial_id": trial_id,
                "complete_demo_success": success,
            }
        )
    )


def test_empty_session_prioritizes_o0_baseline(tmp_path):
    status = progress.build_status(tmp_path)

    assert status["four_object_first_demo_coverage"] == 0
    assert status["objects"][0]["state"] == "ready_for_first_staged_demo"
    assert status["objects"][1]["state"] == "awaiting_g1_calibration"
    assert "restore the O0 low" in status["recommended_next"]["objective"]
    assert "cube38/low_5deg.json" in status["recommended_next"]["command"]


def test_after_o0_success_next_step_is_o1_calibration(tmp_path):
    write_trial(tmp_path, "O0", "cube38_low", "o0_low_01")
    status = progress.build_status(tmp_path)

    assert status["four_object_first_demo_coverage"] == 1
    assert "measure O1 G1" in status["recommended_next"]["objective"]
    assert "--object O1" in status["recommended_next"]["command"]


def test_completed_commissioning_bundle_yields_exact_first_command(tmp_path):
    bundle = write_commissioning(tmp_path, "O1", "cuboid30")
    write_trial(tmp_path, "O0", "cube38_low", "o0_low_01")
    status = progress.build_status(tmp_path)

    o1 = status["objects"][1]
    assert o1["g1_calibration_ready"] is True
    assert o1["commissioning_bundle"] == str(bundle)
    assert status["recommended_next"]["objective"] == "run the O1 low commissioning ladder"
    assert status["recommended_next"]["command"].startswith("python runner.py")


def test_all_first_demos_switch_to_pose_coverage(tmp_path):
    for key, name in (("O1", "cuboid30"), ("O2", "cuboid33"), ("O3", "cuboid38")):
        write_commissioning(tmp_path, key, name)
    for key in ("O0", "O1", "O2", "O3"):
        write_trial(tmp_path, key, f"{key.lower()}_low", f"{key.lower()}_low_01")

    status = progress.build_status(tmp_path)

    assert status["four_object_first_demo_coverage"] == 4
    assert "next distinguishable O0 pose" in status["recommended_next"]["objective"]
    assert "cube38/medium_6p5deg.json" in status["recommended_next"]["command"]


def test_after_pose_coverage_tool_requests_repeats(tmp_path):
    for key, name in (("O1", "cuboid30"), ("O2", "cuboid33"), ("O3", "cuboid38")):
        write_commissioning(tmp_path, key, name)
    profiles = {
        "O0": ("o0_low", "o0_medium", "o0_high"),
        "O1": ("o1_low", "o1_next"),
        "O2": ("o2_low", "o2_next"),
        "O3": ("o3_low", "o3_next"),
    }
    for key, names in profiles.items():
        for index, name in enumerate(names):
            write_trial(tmp_path, key, name, f"{key.lower()}_{index}")

    status = progress.build_status(tmp_path)

    assert "(1/5 recorded trials)" in status["recommended_next"]["objective"]
    assert status["recommended_next"]["command"].startswith("repeat the same")
