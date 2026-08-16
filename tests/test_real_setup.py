import json

import numpy as np
from scipy.spatial.transform import Rotation

from xarm6_toss_sim import G1ApertureModel, URDFKinematics, load_real_setup


def test_received_preplace_pose_matches_local_fk():
    setup = load_real_setup()
    recorded = json.loads(
        (setup.source_root / "real_cube_demo/configs/handoff_place.json").read_text(encoding="utf-8")
    )
    joint = np.asarray(recorded["preplace_joint_rad"], dtype=float)
    expected = np.asarray(recorded["preplace_tcp"], dtype=float)
    transform = URDFKinematics(setup.urdf_path).forward(joint)
    position_error_m = np.linalg.norm(transform[:3, 3] - expected[:3] / 1000.0)
    expected_rotation = Rotation.from_euler("xyz", expected[3:]).as_matrix()
    orientation_error_rad = np.linalg.norm(
        Rotation.from_matrix(transform[:3, :3] @ expected_rotation.T).as_rotvec()
    )
    assert position_error_m < 0.002
    assert orientation_error_rad < 0.01


def test_g1_aperture_uses_received_geometry_and_cube_anchor():
    setup = load_real_setup()
    model = G1ApertureModel(
        setup.urdf_path,
        anchor_position=setup.held_gripper_position,
        anchor_width_m=setup.cube_side_m,
    )
    assert np.isclose(model.projected_width_m(370.0), 0.038)
    assert 0.08 < model.projected_width_m(850.0) < 0.09
    assert model.projected_width_m(520.0) > model.projected_width_m(370.0)


def test_real_limits_and_calibration_are_loaded_from_handoff():
    setup = load_real_setup()
    assert setup.control_period_s == 0.02
    assert setup.max_joint_speed_rad_s == 0.45
    assert setup.max_joint_acceleration_rad_s2 == 1.5
    assert setup.detach_delay_range_s == (0.025, 0.044)
    assert setup.third_view.serial == "317222073552"
    assert setup.wrist.serial == "233622079809"
    assert np.isclose(setup.third_view.intrinsic[0, 0], 597.4084346880913)


def test_recorded_external_pose_is_not_the_rejected_inward_geometry():
    setup = load_real_setup()
    recorded = json.loads(
        (setup.source_root / "real_cube_demo/configs/handoff_place.json").read_text(encoding="utf-8")
    )
    metrics = URDFKinematics(setup.urdf_path).radial_metrics(recorded["preplace_joint_rad"])
    assert metrics["tcp_horizontal_radius_m"] > 0.55
