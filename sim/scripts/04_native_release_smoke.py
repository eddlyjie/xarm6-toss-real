#!/usr/bin/env python3
"""Native xArm6+G1 grasp and partial-release smoke with a light cube.

The cube pose and velocity are written exactly once at episode initialization.
Every subsequent state is produced by the arm controller, G1 contacts and
PhysX rigid-body dynamics.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from isaaclab.app import AppLauncher


SIM_ROOT = Path(__file__).resolve().parents[1]
XARM_ROOT = SIM_ROOT.parent
DEFAULT_USD = (
    SIM_ROOT
    / "assets"
    / "xarm6_g1"
    / "xarm6_g1.usd"
    / "xarm6_g1"
    / "xarm6_g1.usda"
)
DEFAULT_CONFIG = SIM_ROOT / "configs" / "upward_throw_smoke.json"

parser = argparse.ArgumentParser()
parser.add_argument("--usd", type=Path, default=DEFAULT_USD)
parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
parser.add_argument("--output", type=Path, required=True)
parser.add_argument("--video-path", type=Path, default=None)
parser.add_argument("--cube-size-m", type=float, default=0.038)
parser.add_argument("--cube-mass-kg", type=float, default=0.035)
parser.add_argument(
    "--cube-offset-hand-m",
    type=float,
    nargs=3,
    default=(0.0, 0.0, 0.0),
    metavar=("X", "Y", "Z"),
)
parser.add_argument("--held-drive-rad", type=float, default=0.37)
parser.add_argument("--partial-open-drive-rad", type=float, default=0.52)
parser.add_argument("--settle-s", type=float, default=0.40)
parser.add_argument("--post-release-s", type=float, default=0.50)
parser.add_argument("--release-time-s", type=float, default=0.60)
parser.add_argument(
    "--catch-servo-start-time-s",
    type=float,
    default=None,
    help="Enable closed-loop Cartesian catch servo at this episode time.",
)
parser.add_argument(
    "--catch-close-time-s",
    type=float,
    default=None,
    help="Command the G1 back to held position at this episode time.",
)
parser.add_argument(
    "--vision-control-end-time-s",
    type=float,
    default=None,
    help="Last episode time at which camera state may update the arm target.",
)
parser.add_argument("--catch-servo-gain", type=float, default=0.75)
parser.add_argument(
    "--catch-max-joint-step-rad",
    type=float,
    default=0.035,
)
parser.add_argument(
    "--catch-prediction-horizon-s",
    type=float,
    default=0.0,
    help="Ballistic lookahead used for the arm intercept target.",
)
parser.add_argument(
    "--detach-delay-prior-s",
    type=float,
    default=0.05,
    help="Encoder/FK prior for physical detach after the open command.",
)
parser.add_argument(
    "--intercept-residual-model",
    type=Path,
    default=None,
    help="Frozen deployable delta-intercept checkpoint.",
)
parser.add_argument(
    "--catch-evidence-window-s",
    type=float,
    default=0.50,
)
parser.add_argument(
    "--observation-mode",
    choices=("physics", "global_camera"),
    default="physics",
    help="Cube position source used by the post-release catch servo.",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from isaaclab.sensors import Camera, CameraCfg, ContactSensor, ContactSensorCfg
from isaaclab.utils.math import quat_apply, quat_apply_inverse


sys.path.insert(0, str(XARM_ROOT / "src"))
from xarm6_toss.ballistic_tracker import BallisticTracker  # noqa: E402
from xarm6_toss.control_reference import (  # noqa: E402
    QuinticJointSegment,
    generate_joint_reference,
)
from xarm6_toss.intercept_residual import (  # noqa: E402
    InterceptResidualPolicy,
    residual_features,
)


GRIPPER_PASSIVE_JOINTS = (
    "left_finger_joint",
    "left_inner_knuckle_joint",
    "right_outer_knuckle_joint",
    "right_finger_joint",
    "right_inner_knuckle_joint",
)

GRIPPER_PRIM = (
    "/World/XArm6/Geometry/world/link_base/link1/link2/link3/link4/"
    "link5/link6/link_eef/xarm_gripper_base_link"
)
LEFT_FINGER_PRIM = (
    GRIPPER_PRIM + "/left_outer_knuckle/left_finger"
)
RIGHT_FINGER_PRIM = (
    GRIPPER_PRIM + "/right_outer_knuckle/right_finger"
)

GLOBAL_CAMERA_K = np.asarray(
    [
        [597.4, 0.0, 316.8],
        [0.0, 595.8, 242.7],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float32,
)
GLOBAL_CAMERA_R_BASE = np.asarray(
    [
        [-0.01506891, -0.58940248, -0.80769898],
        [-0.99977402, -0.00323198, 0.02101085],
        [-0.01499432, 0.80783307, -0.58922059],
    ],
    dtype=np.float32,
)
GLOBAL_CAMERA_POSITION_BASE_M = np.asarray(
    [1.00698621, 0.00035981, 0.64736570], dtype=np.float32
)
GLOBAL_CAMERA_QUAT_XYZW = (0.6272, -0.6327, -0.3275, 0.3132)


def robot_cfg(usd_path: Path, held_drive_rad: float) -> ArticulationCfg:
    config = json.loads(args_cli.config.read_text(encoding="utf-8"))
    start_q = config["reference_segments"][0]["start_joint_rad"]
    return ArticulationCfg(
        prim_path="/World/XArm6",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(usd_path),
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=2.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=4,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.001,
                rest_offset=0.0,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                **{
                    f"joint{index + 1}": float(value)
                    for index, value in enumerate(start_q)
                },
                "drive_joint": held_drive_rad,
                **{
                    name: held_drive_rad
                    for name in GRIPPER_PASSIVE_JOINTS
                },
            },
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=["joint[1-6]"],
                effort_limit_sim={
                    "joint[1-2]": 50.0,
                    "joint[3-5]": 32.0,
                    "joint6": 20.0,
                },
                velocity_limit_sim=3.14,
                stiffness=400.0,
                damping=40.0,
            ),
            "gripper_drive": ImplicitActuatorCfg(
                joint_names_expr=["drive_joint"],
                effort_limit_sim=50.0,
                velocity_limit_sim=2.0,
                stiffness=120.0,
                damping=5.0,
            ),
            "gripper_passive": ImplicitActuatorCfg(
                joint_names_expr=list(GRIPPER_PASSIVE_JOINTS),
                effort_limit_sim=1.0,
                velocity_limit_sim=2.0,
                stiffness=0.0,
                damping=0.0,
            ),
        },
    )


def cube_cfg(size_m: float, mass_kg: float) -> RigidObjectCfg:
    return RigidObjectCfg(
        prim_path="/World/Cube",
        spawn=sim_utils.CuboidCfg(
            size=(size_m, size_m, size_m),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                max_linear_velocity=3.0,
                max_angular_velocity=30.0,
                max_depenetration_velocity=1.0,
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=4,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=mass_kg),
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=0.001,
                rest_offset=0.0,
            ),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=1.2,
                dynamic_friction=0.9,
                restitution=0.02,
                friction_combine_mode="max",
                restitution_combine_mode="min",
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.95, 0.72, 0.05),
                roughness=0.6,
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 2.0)),
    )


def load_reference(config_path: Path):
    config = json.loads(config_path.read_text(encoding="utf-8"))
    segments = tuple(
        QuinticJointSegment(**segment)
        for segment in config["reference_segments"]
    )
    samples = generate_joint_reference(
        segments,
        float(config["control_period_s"]),
    )
    return config, samples


def step_assets(
    sim: sim_utils.SimulationContext,
    robot: Articulation,
    cube: RigidObject,
    contact_sensors: tuple[ContactSensor, ContactSensor],
    camera: Camera | None,
) -> None:
    robot.write_data_to_sim()
    cube.write_data_to_sim()
    sim.step(render=camera is not None)
    dt = sim.get_physics_dt()
    robot.update(dt)
    cube.update(dt)
    for sensor in contact_sensors:
        sensor.update(dt, force_recompute=True)
    if camera is not None:
        camera.update(dt, force_recompute=True)


def camera_rgb_frame(camera: Camera) -> np.ndarray:
    rgb_data = camera.data.output["rgb"]
    rgb = rgb_data.torch[0] if hasattr(rgb_data, "torch") else rgb_data[0]
    rgb = rgb[..., :3].to(dtype=torch.float32)
    if float(torch.max(rgb).item()) <= 1.5:
        rgb = rgb * 255.0
    return (
        torch.clamp(rgb, 0.0, 255.0)
        .to(dtype=torch.uint8)
        .detach()
        .cpu()
        .numpy()
    )


def cube_contact_force_n(sensor: ContactSensor) -> float:
    net_forces = sensor.data.net_forces_w
    if net_forces is None:
        return 0.0
    return float(
        torch.linalg.vector_norm(net_forces.torch).item()
    )


def estimate_cube_position_from_global_camera(
    camera: Camera,
    cube_size_m: float,
    expected_position_base_m: torch.Tensor,
) -> tuple[torch.Tensor | None, dict[str, float]]:
    rgb_data = camera.data.output["rgb"]
    depth_data = camera.data.output["distance_to_image_plane"]
    rgb = rgb_data.torch[0] if hasattr(rgb_data, "torch") else rgb_data[0]
    depth = (
        depth_data.torch[0, :, :, 0]
        if hasattr(depth_data, "torch")
        else depth_data[0, :, :, 0]
    )
    rgb = rgb[..., :3].to(dtype=torch.float32)
    scale = 255.0 if float(torch.max(rgb).item()) > 1.5 else 1.0
    red, green, blue = rgb.unbind(dim=-1)
    rotation = torch.as_tensor(GLOBAL_CAMERA_R_BASE, device=camera.device)
    translation = torch.as_tensor(
        GLOBAL_CAMERA_POSITION_BASE_M, device=camera.device
    )
    expected_camera = rotation.T @ (
        expected_position_base_m - translation
    )
    expected_u = (
        GLOBAL_CAMERA_K[0, 0] * expected_camera[0] / expected_camera[2]
        + GLOBAL_CAMERA_K[0, 2]
    )
    expected_v = (
        GLOBAL_CAMERA_K[1, 1] * expected_camera[1] / expected_camera[2]
        + GLOBAL_CAMERA_K[1, 2]
    )
    rows = torch.arange(rgb.shape[0], device=camera.device)[:, None]
    columns = torch.arange(rgb.shape[1], device=camera.device)[None, :]
    roi = (
        torch.abs(rows - expected_v) <= 40.0
    ) & (torch.abs(columns - expected_u) <= 40.0)
    yellow_score = 2.0 * red - green - blue
    mask = roi & (
        (red > 0.45 * scale)
        & (yellow_score > 0.30 * scale)
        & ((green - blue) > 0.015 * scale)
    )
    pixels = torch.nonzero(mask, as_tuple=False)
    if pixels.shape[0] < 3:
        failure_metadata = {
            "detected": 0.0,
            "rgb_max": float(torch.max(rgb).item()),
            "red_max": float(torch.max(red).item()),
            "green_max": float(torch.max(green).item()),
            "blue_min": float(torch.min(blue).item()),
            "pixel_count": float(pixels.shape[0]),
            "max_yellow_score": float(torch.max(yellow_score).item()),
            "red_green_count": float(
                torch.count_nonzero((red - green) > 0.02 * scale).item()
            ),
            "expected_u_px": float(expected_u.item()),
            "expected_v_px": float(expected_v.item()),
        }
        print(
            json.dumps({"camera_detection_failure": True, **failure_metadata}),
            flush=True,
        )
        return None, failure_metadata
    v = torch.mean(pixels[:, 0].to(torch.float32))
    u = torch.mean(pixels[:, 1].to(torch.float32))
    z_surface = torch.median(depth[mask])
    z_center = z_surface + 0.5 * cube_size_m
    camera_point = torch.stack(
        (
            (u - GLOBAL_CAMERA_K[0, 2]) / GLOBAL_CAMERA_K[0, 0] * z_center,
            (v - GLOBAL_CAMERA_K[1, 2]) / GLOBAL_CAMERA_K[1, 1] * z_center,
            z_center,
        )
    )
    position_base = rotation @ camera_point + translation
    return position_base, {
        "detected": 1.0,
        "u_px": float(u.item()),
        "v_px": float(v.item()),
        "depth_surface_m": float(z_surface.item()),
        "pixel_count": float(pixels.shape[0]),
        "expected_u_px": float(expected_u.item()),
        "expected_v_px": float(expected_v.item()),
    }


def hand_state(
    robot: Articulation,
    gripper_body_id: int,
    finger_body_ids: list[int],
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    body_pose = robot.data.body_pose_w.torch[0]
    gripper_pose = body_pose[gripper_body_id]
    finger_midpoint = body_pose[finger_body_ids, :3].mean(dim=0)
    return gripper_pose[:3], gripper_pose[3:7], finger_midpoint


def place_cube_once(
    robot: Articulation,
    cube: RigidObject,
    gripper_body_id: int,
    finger_body_ids: list[int],
) -> list[float]:
    hand_position, hand_quaternion, finger_midpoint = hand_state(
        robot,
        gripper_body_id,
        finger_body_ids,
    )
    local_offset = torch.tensor(
        [args_cli.cube_offset_hand_m],
        dtype=torch.float32,
        device=robot.device,
    )
    world_offset = quat_apply(
        hand_quaternion.unsqueeze(0),
        local_offset,
    )[0]
    pose = cube.data.default_root_pose.torch.clone()
    pose[0, :3] = finger_midpoint + world_offset
    pose[0, 3:7] = hand_quaternion
    velocity = torch.zeros((1, 6), device=robot.device)
    cube.write_root_pose_to_sim_index(root_pose=pose)
    cube.write_root_velocity_to_sim_index(root_velocity=velocity)
    cube.reset()
    return [float(value) for value in pose[0].tolist()]


def record_state(
    time_s: float,
    phase: str,
    robot: Articulation,
    cube: RigidObject,
    arm_ids: list[int],
    drive_id: int,
    gripper_body_id: int,
    finger_body_ids: list[int],
    contact_sensors: tuple[ContactSensor, ContactSensor],
) -> dict[str, object]:
    hand_position, hand_quaternion, finger_midpoint = hand_state(
        robot,
        gripper_body_id,
        finger_body_ids,
    )
    cube_pose = cube.data.root_link_pose_w.torch[0]
    cube_linear = cube.data.root_com_lin_vel_w.torch[0]
    cube_angular = cube.data.root_com_ang_vel_w.torch[0]
    relative_hand = quat_apply_inverse(
        hand_quaternion.unsqueeze(0),
        (cube_pose[:3] - hand_position).unsqueeze(0),
    )[0]
    return {
        "time_s": time_s,
        "phase": phase,
        "cube_position_w_m": [
            float(value) for value in cube_pose[:3].tolist()
        ],
        "cube_quaternion_wxyz": [
            float(value) for value in cube_pose[3:7].tolist()
        ],
        "cube_linear_velocity_w_m_s": [
            float(value) for value in cube_linear.tolist()
        ],
        "cube_angular_velocity_w_rad_s": [
            float(value) for value in cube_angular.tolist()
        ],
        "hand_position_w_m": [
            float(value) for value in hand_position.tolist()
        ],
        "finger_midpoint_w_m": [
            float(value) for value in finger_midpoint.tolist()
        ],
        "cube_position_hand_m": [
            float(value) for value in relative_hand.tolist()
        ],
        "arm_joint_position_rad": [
            float(value)
            for value in robot.data.joint_pos.torch[0, arm_ids].tolist()
        ],
        "arm_joint_velocity_rad_s": [
            float(value)
            for value in robot.data.joint_vel.torch[0, arm_ids].tolist()
        ],
        "gripper_drive_rad": float(
            robot.data.joint_pos.torch[0, drive_id].item()
        ),
        "left_finger_cube_contact_force_n": cube_contact_force_n(
            contact_sensors[0]
        ),
        "right_finger_cube_contact_force_n": cube_contact_force_n(
            contact_sensors[1]
        ),
    }


def summarize(
    records: list[dict[str, object]],
    placed_pose: list[float],
) -> dict[str, object]:
    settle_records = [
        record for record in records
        if record["phase"] == "settle"
    ]
    throw_records = [
        record for record in records
        if record["phase"] == "throw"
    ]
    flight_records = [
        record for record in records
        if record["phase"] == "flight"
    ]
    postrelease_records = [
        record for record in records
        if record["phase"] in ("flight", "catch")
    ]
    baseline = np.asarray(
        settle_records[-1]["cube_position_hand_m"],
        dtype=float,
    )
    prethrow_errors = [
        float(
            np.linalg.norm(
                np.asarray(record["cube_position_hand_m"], dtype=float)
                - baseline
            )
        )
        for record in throw_records
    ]
    detach_time_s = None
    for record in postrelease_records:
        error = float(
            np.linalg.norm(
                np.asarray(record["cube_position_hand_m"], dtype=float)
                - baseline
            )
        )
        if error >= 0.012:
            detach_time_s = float(record["time_s"])
            break

    release_height = float(
        throw_records[-1]["cube_position_w_m"][2]
    )
    maximum_height = max(
        float(record["cube_position_w_m"][2])
        for record in postrelease_records
    )
    final_record = records[-1]
    catch_records = [
        record for record in records
        if record["phase"] == "catch"
    ]
    catch_stable = None
    catch_max_relative_error = None
    catch_max_relative_motion = None
    bilateral_contact_fraction = None
    if catch_records:
        evidence_start = float(catch_records[-1]["time_s"]) - (
            args_cli.catch_evidence_window_s
        )
        evidence_records = [
            record for record in catch_records
            if float(record["time_s"]) >= evidence_start
        ]
        catch_errors = [
            float(
                np.linalg.norm(
                    np.asarray(record["cube_position_hand_m"], dtype=float)
                    - baseline
                )
            )
            for record in evidence_records
        ]
        catch_max_relative_error = max(catch_errors)
        relative_positions = np.asarray(
            [record["cube_position_hand_m"] for record in evidence_records],
            dtype=float,
        )
        relative_center = np.mean(relative_positions, axis=0)
        catch_max_relative_motion = float(
            np.max(
                np.linalg.norm(relative_positions - relative_center, axis=1)
            )
        )
        bilateral_contact_fraction = float(
            np.mean(
                [
                    record["left_finger_cube_contact_force_n"] > 0.05
                    and record["right_finger_cube_contact_force_n"] > 0.05
                    for record in evidence_records
                ]
            )
        )
        catch_stable = (
            catch_max_relative_motion <= 0.003
            and bilateral_contact_fraction >= 0.90
            and min(
                float(record["cube_position_w_m"][2])
                for record in evidence_records
            ) > args_cli.cube_size_m * 0.75
        )
    return {
        "schema": "xarm6_native_release_smoke_v1",
        "cube_state_writes_after_initialization": 0,
        "placed_cube_pose_w": placed_pose,
        "prethrow_max_relative_error_m": max(prethrow_errors),
        "prethrow_stable": max(prethrow_errors) <= 0.008,
        "detach_detected": detach_time_s is not None,
        "detach_time_s": detach_time_s,
        "release_command_time_s": args_cli.release_time_s,
        "detach_delay_s": (
            None
            if detach_time_s is None
            else detach_time_s - args_cli.release_time_s
        ),
        "release_height_m": release_height,
        "maximum_height_m": maximum_height,
        "free_vertical_displacement_m": maximum_height - release_height,
        "catch_servo_enabled": args_cli.catch_servo_start_time_s is not None,
        "catch_servo_start_time_s": args_cli.catch_servo_start_time_s,
        "catch_close_time_s": args_cli.catch_close_time_s,
        "catch_prediction_horizon_s": args_cli.catch_prediction_horizon_s,
        "catch_max_relative_error_m": catch_max_relative_error,
        "catch_max_relative_motion_m": catch_max_relative_motion,
        "catch_stable": catch_stable,
        "catch_evidence_window_s": args_cli.catch_evidence_window_s,
        "bilateral_contact_fraction": bilateral_contact_fraction,
        "final_cube_position_w_m": final_record["cube_position_w_m"],
        "final_cube_linear_velocity_w_m_s": (
            final_record["cube_linear_velocity_w_m_s"]
        ),
        "cube_size_m": args_cli.cube_size_m,
        "cube_mass_kg": args_cli.cube_mass_kg,
        "held_drive_rad": args_cli.held_drive_rad,
        "partial_open_drive_rad": args_cli.partial_open_drive_rad,
        "cube_offset_hand_m": list(args_cli.cube_offset_hand_m),
    }


def main() -> int:
    if not args_cli.usd.is_file():
        raise FileNotFoundError(args_cli.usd)
    config, reference = load_reference(args_cli.config)
    control_period = float(config["control_period_s"])
    duration = float(reference[-1].time_s) + args_cli.post_release_s
    if args_cli.catch_servo_start_time_s is not None:
        if args_cli.catch_close_time_s is None:
            raise ValueError("catch servo requires --catch-close-time-s")
        if args_cli.catch_close_time_s < args_cli.catch_servo_start_time_s:
            raise ValueError("catch close time must not precede servo start")
        duration = max(
            duration,
            args_cli.catch_close_time_s + args_cli.catch_evidence_window_s + 0.10,
        )

    args_cli.output.mkdir(parents=True, exist_ok=True)
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(
            dt=0.005,
            device=args_cli.device,
            render_interval=4,
        )
    )
    ground_cfg = sim_utils.CuboidCfg(
        size=(2.0, 2.0, 0.02),
        collision_props=sim_utils.CollisionPropertiesCfg(
            contact_offset=0.001,
            rest_offset=0.0,
        ),
        visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=(0.18, 0.20, 0.22),
            roughness=0.9,
        ),
    )
    ground_cfg.func(
        "/World/GroundPlane", ground_cfg, translation=(0.0, 0.0, -0.01)
    )
    sim_utils.DomeLightCfg(
        intensity=2500.0,
        color=(0.8, 0.8, 0.8),
    ).func(
        "/World/Light",
        sim_utils.DomeLightCfg(
            intensity=2500.0,
            color=(0.8, 0.8, 0.8),
        ),
    )
    robot = Articulation(robot_cfg(args_cli.usd, args_cli.held_drive_rad))
    cube = RigidObject(cube_cfg(args_cli.cube_size_m, args_cli.cube_mass_kg))
    global_camera = None
    if args_cli.observation_mode == "global_camera":
        global_camera = Camera(
            CameraCfg(
                prim_path="/World/GlobalCamera",
                update_period=1.0 / 60.0,
                height=480,
                width=640,
                data_types=["rgb", "distance_to_image_plane"],
                offset=CameraCfg.OffsetCfg(
                    pos=tuple(float(v) for v in GLOBAL_CAMERA_POSITION_BASE_M),
                    rot=GLOBAL_CAMERA_QUAT_XYZW,
                    convention="ros",
                ),
                spawn=sim_utils.PinholeCameraCfg.from_intrinsic_matrix(
                    intrinsic_matrix=GLOBAL_CAMERA_K.reshape(-1).tolist(),
                    width=640,
                    height=480,
                    clipping_range=(0.10, 3.0),
                ),
            )
        )
    sim_utils.activate_contact_sensors(LEFT_FINGER_PRIM)
    sim_utils.activate_contact_sensors(RIGHT_FINGER_PRIM)
    contact_sensors = (
        ContactSensor(
            ContactSensorCfg(
                prim_path=LEFT_FINGER_PRIM,
                update_period=0.0,
                debug_vis=False,
            )
        ),
        ContactSensor(
            ContactSensorCfg(
                prim_path=RIGHT_FINGER_PRIM,
                update_period=0.0,
                debug_vis=False,
            )
        ),
    )
    sim.reset()

    arm_ids, _ = robot.find_joints("joint[1-6]")
    drive_ids, _ = robot.find_joints("drive_joint")
    gripper_body_ids, _ = robot.find_bodies("xarm_gripper_base_link")
    finger_body_ids, _ = robot.find_bodies("left_finger|right_finger")
    if (
        len(arm_ids) != 6
        or len(drive_ids) != 1
        or len(gripper_body_ids) != 1
        or len(finger_body_ids) != 2
    ):
        raise RuntimeError("unexpected xArm6/G1 joint or body layout")
    drive_id = drive_ids[0]
    gripper_body_id = gripper_body_ids[0]

    q = robot.data.default_joint_pos.torch.clone()
    dq = robot.data.default_joint_vel.torch.clone()
    robot.write_joint_position_to_sim_index(position=q)
    robot.write_joint_velocity_to_sim_index(velocity=dq)
    robot.set_joint_position_target_index(target=q)
    robot.reset()
    step_assets(sim, robot, cube, contact_sensors, global_camera)
    placed_pose = place_cube_once(
        robot,
        cube,
        gripper_body_id,
        finger_body_ids,
    )

    held_target = torch.tensor(
        [[args_cli.held_drive_rad]],
        device=sim.device,
    )
    robot.set_joint_position_target_index(
        target=held_target,
        joint_ids=drive_ids,
    )
    video_frames: list[np.ndarray] = []
    last_video_frame_index = -1
    records: list[dict[str, object]] = []
    time_s = -args_cli.settle_s
    while time_s < 0.0:
        step_assets(sim, robot, cube, contact_sensors, global_camera)
        time_s += sim.get_physics_dt()
        video_frame_index = int((time_s + args_cli.settle_s) * 60.0)
        if (
            args_cli.video_path is not None
            and video_frame_index > last_video_frame_index
        ):
            video_frames.append(camera_rgb_frame(global_camera))
            last_video_frame_index = video_frame_index
        records.append(
            record_state(
                time_s,
                "settle",
                robot,
                cube,
                arm_ids,
                drive_id,
                gripper_body_id,
                finger_body_ids,
                contact_sensors,
            )
        )
    grasp_position_hand = torch.tensor(
        records[-1]["cube_position_hand_m"],
        dtype=torch.float32,
        device=sim.device,
    )

    open_target = torch.tensor(
        [[args_cli.partial_open_drive_rad]],
        device=sim.device,
    )
    dt = sim.get_physics_dt()
    last_control_index = -1
    catch_arm_target = None
    camera_measurements: list[dict[str, float]] = []
    camera_position_errors_m: list[float] = []
    ballistic_tracker = BallisticTracker() if global_camera is not None else None
    residual_policy = (
        None
        if args_cli.intercept_residual_model is None
        else InterceptResidualPolicy.load(args_cli.intercept_residual_model)
    )
    intercept_errors_before_m: list[float] = []
    intercept_errors_after_m: list[float] = []
    intercept_residual_actions_m: list[float] = []
    last_detected_camera_time_s = None
    encoder_prior_position = None
    encoder_prior_velocity = None
    encoder_prior_time_s = None
    while time_s <= duration + 1.0e-9:
        sample_index = min(
            int((time_s + 1.0e-9) / control_period),
            len(reference) - 1,
        )
        arm_target = torch.tensor(
            [reference[sample_index].joint_position_rad],
            dtype=torch.float32,
            device=sim.device,
        )
        catch_active = (
            args_cli.catch_servo_start_time_s is not None
            and time_s >= args_cli.catch_servo_start_time_s
        )
        vision_control_end_time_s = args_cli.vision_control_end_time_s
        if vision_control_end_time_s is None:
            vision_control_end_time_s = args_cli.catch_close_time_s
        vision_servo_active = (
            global_camera is None
            or vision_control_end_time_s is None
            or time_s <= vision_control_end_time_s + 1.0e-9
        )
        if (
            catch_active and vision_servo_active
            and sample_index != last_control_index
        ):
            hand_position, hand_quaternion, _ = hand_state(
                robot,
                gripper_body_id,
                finger_body_ids,
            )
            grasp_offset_world = quat_apply(
                hand_quaternion.unsqueeze(0),
                grasp_position_hand.unsqueeze(0),
            )[0]
            hand_linear_velocity = robot.data.body_link_lin_vel_w.torch[
                0, gripper_body_id
            ]
            hand_angular_velocity = robot.data.body_link_ang_vel_w.torch[
                0, gripper_body_id
            ]
            encoder_cube_position = hand_position + grasp_offset_world
            encoder_cube_velocity = (
                hand_linear_velocity
                + torch.linalg.cross(
                    hand_angular_velocity,
                    grasp_offset_world,
                )
            )
            nominal_detach_time_s = (
                args_cli.release_time_s + args_cli.detach_delay_prior_s
            )
            gravity = torch.tensor([0.0, 0.0, -9.81], device=sim.device)
            if (
                encoder_prior_position is None
                or time_s <= nominal_detach_time_s + 1.0e-9
            ):
                encoder_prior_position = encoder_cube_position
                encoder_prior_velocity = encoder_cube_velocity
                encoder_prior_time_s = time_s
                if ballistic_tracker is not None:
                    ballistic_tracker.set_encoder_prior(
                        time_s,
                        encoder_prior_position.detach().cpu().numpy(),
                        encoder_prior_velocity.detach().cpu().numpy(),
                    )
            prior_age_s = max(0.0, time_s - encoder_prior_time_s)
            prior_cube_position = (
                encoder_prior_position
                + encoder_prior_velocity * prior_age_s
                + 0.5 * gravity * prior_age_s**2
            )
            prior_cube_velocity = encoder_prior_velocity + gravity * prior_age_s
            cube_position_truth = cube.data.root_link_pose_w.torch[0, :3]
            if global_camera is None:
                cube_position = cube_position_truth
                cube_velocity = cube.data.root_com_lin_vel_w.torch[0]
            else:
                measured_position, camera_metadata = (
                    estimate_cube_position_from_global_camera(
                        global_camera,
                        args_cli.cube_size_m,
                        prior_cube_position,
                    )
                )
                if measured_position is not None:
                    last_detected_camera_time_s = time_s
                    ballistic_tracker.add_camera_position(
                        time_s,
                        measured_position.detach().cpu().numpy(),
                    )
                estimate = ballistic_tracker.estimate(time_s)
                cube_position = torch.tensor(
                    estimate.position_m,
                    dtype=torch.float32,
                    device=sim.device,
                )
                cube_velocity = torch.tensor(
                    estimate.velocity_m_s,
                    dtype=torch.float32,
                    device=sim.device,
                )
                if measured_position is not None:
                    prediction_age_s = 0.0
                    state_source = (
                        "global_camera_ballistic_fit"
                        if estimate.camera_sample_count >= 2
                        else "global_camera_encoder_velocity"
                    )
                else:
                    state_source = (
                        "camera_ballistic_prediction"
                        if estimate.camera_sample_count > 0
                        else "encoder_prior"
                    )
                    prediction_age_s = (
                        0.0
                        if estimate.camera_sample_count == 0
                        else time_s - last_detected_camera_time_s
                    )
                position_error_m = float(
                    torch.linalg.vector_norm(
                        cube_position - cube_position_truth
                    ).item()
                )
                camera_metadata.update(
                    time_s=float(time_s),
                    position_error_m=position_error_m,
                    prediction_age_s=float(prediction_age_s),
                    ballistic_camera_sample_count=float(
                        estimate.camera_sample_count
                    ),
                    ballistic_fit_rms_m=estimate.fit_rms_m,
                    estimated_position_base_m=list(estimate.position_m),
                    estimated_velocity_base_m_s=list(estimate.velocity_m_s),
                    state_source=state_source,
                )
                camera_measurements.append(camera_metadata)
                camera_position_errors_m.append(position_error_m)
            prediction_horizon_s = args_cli.catch_prediction_horizon_s
            state_position_current = cube_position
            state_velocity_current = cube_velocity
            if prediction_horizon_s > 0.0:
                gravity = torch.tensor(
                    [0.0, 0.0, -9.81], device=sim.device
                )
                cube_position = (
                    state_position_current
                    + state_velocity_current * prediction_horizon_s
                    + 0.5 * gravity * prediction_horizon_s**2
                )
            residual_action = np.zeros(3, dtype=float)
            if (
                residual_policy is not None
                and global_camera is not None
                and estimate.camera_sample_count > 0
            ):
                feature = residual_features(
                    time_since_release_s=time_s - args_cli.release_time_s,
                    camera_sample_count=estimate.camera_sample_count,
                    fit_rms_m=estimate.fit_rms_m,
                    position_innovation_m=(
                        state_position_current - prior_cube_position
                    ).detach().cpu().numpy(),
                    velocity_innovation_m_s=(
                        state_velocity_current - prior_cube_velocity
                    ).detach().cpu().numpy(),
                )
                residual_action = residual_policy.predict(feature)
                cube_position = cube_position + torch.tensor(
                    residual_action,
                    dtype=torch.float32,
                    device=sim.device,
                )
            if global_camera is not None:
                true_velocity = cube.data.root_com_lin_vel_w.torch[0]
                true_intercept = (
                    cube_position_truth
                    + true_velocity * prediction_horizon_s
                    + 0.5 * gravity * prediction_horizon_s**2
                )
                nominal_intercept = cube_position - torch.tensor(
                    residual_action,
                    dtype=torch.float32,
                    device=sim.device,
                )
                intercept_error_before_m = float(
                    torch.linalg.vector_norm(
                        nominal_intercept - true_intercept
                    ).item()
                )
                intercept_error_after_m = float(
                    torch.linalg.vector_norm(
                        cube_position - true_intercept
                    ).item()
                )
                residual_action_norm_m = float(
                    np.linalg.norm(residual_action)
                )
                intercept_errors_before_m.append(intercept_error_before_m)
                intercept_errors_after_m.append(intercept_error_after_m)
                intercept_residual_actions_m.append(residual_action_norm_m)
                camera_metadata.update(
                    residual_action_m=residual_action.tolist(),
                    residual_action_norm_m=residual_action_norm_m,
                    intercept_error_before_residual_m=intercept_error_before_m,
                    intercept_error_after_residual_m=intercept_error_after_m,
                )
            desired_hand_position = cube_position - grasp_offset_world
            position_error = desired_hand_position - hand_position
            jacobian = robot.data.body_link_jacobian_w.torch[
                0, gripper_body_id - 1, :3, arm_ids
            ]
            damping = 1.0e-3 * torch.eye(3, device=sim.device)
            delta_joint = jacobian.T @ torch.linalg.solve(
                jacobian @ jacobian.T + damping,
                position_error,
            )
            delta_joint = torch.clamp(
                args_cli.catch_servo_gain * delta_joint,
                -args_cli.catch_max_joint_step_rad,
                args_cli.catch_max_joint_step_rad,
            )
            catch_arm_target = (
                robot.data.joint_pos.torch[0, arm_ids] + delta_joint
            ).unsqueeze(0)
        last_control_index = sample_index
        if catch_active and catch_arm_target is not None:
            arm_target = catch_arm_target
        robot.set_joint_position_target_index(
            target=arm_target,
            joint_ids=arm_ids,
        )
        phase = "throw"
        if time_s >= args_cli.release_time_s:
            phase = "flight"
            gripper_target = open_target
            if (
                args_cli.catch_close_time_s is not None
                and time_s >= args_cli.catch_close_time_s
            ):
                gripper_target = held_target
                phase = "catch"
            robot.set_joint_position_target_index(
                target=gripper_target,
                joint_ids=drive_ids,
            )
        else:
            robot.set_joint_position_target_index(
                target=held_target,
                joint_ids=drive_ids,
            )
        step_assets(sim, robot, cube, contact_sensors, global_camera)
        records.append(
            record_state(
                time_s,
                phase,
                robot,
                cube,
                arm_ids,
                drive_id,
                gripper_body_id,
                finger_body_ids,
                contact_sensors,
            )
        )
        video_frame_index = int((time_s + args_cli.settle_s) * 60.0)
        if (
            args_cli.video_path is not None
            and video_frame_index > last_video_frame_index
        ):
            video_frames.append(camera_rgb_frame(global_camera))
            last_video_frame_index = video_frame_index
        time_s += dt

    summary = summarize(records, placed_pose)
    detach_time_s = summary["detach_time_s"]
    post_detach_camera_updates = [
        measurement
        for measurement in camera_measurements
        if detach_time_s is not None
        and measurement["time_s"] > detach_time_s
        and measurement["state_source"] != "encoder_prior"
    ]
    learned_updates_after_detach = [
        measurement
        for measurement in camera_measurements
        if detach_time_s is not None
        and measurement["time_s"] > detach_time_s
        and measurement.get("residual_action_norm_m", 0.0) > 1.0e-6
    ]
    summary.update(
        observation_mode=args_cli.observation_mode,
        detach_delay_prior_s=args_cli.detach_delay_prior_s,
        intercept_residual_model=(
            None
            if args_cli.intercept_residual_model is None
            else str(args_cli.intercept_residual_model)
        ),
        learned_residual_action_count=sum(
            action > 1.0e-6 for action in intercept_residual_actions_m
        ),
        learned_control_updates_after_detach=len(learned_updates_after_detach),
        vision_control_end_time_s=args_cli.vision_control_end_time_s,
        camera_measurement_count=len(camera_measurements),
        camera_control_updates_after_detach=len(post_detach_camera_updates),
        first_camera_control_update_after_detach_s=(
            None
            if not post_detach_camera_updates
            else post_detach_camera_updates[0]["time_s"]
        ),
        camera_mean_position_error_m=(
            None
            if not camera_position_errors_m
            else float(np.mean(camera_position_errors_m))
        ),
        camera_max_position_error_m=(
            None
            if not camera_position_errors_m
            else float(np.max(camera_position_errors_m))
        ),
        intercept_mean_error_before_residual_m=(
            None if not intercept_errors_before_m
            else float(np.mean(intercept_errors_before_m))
        ),
        intercept_mean_error_after_residual_m=(
            None if not intercept_errors_after_m
            else float(np.mean(intercept_errors_after_m))
        ),
        intercept_residual_fraction_improved=(
            None if not intercept_errors_before_m
            else float(np.mean(np.asarray(intercept_errors_after_m) < np.asarray(intercept_errors_before_m)))
        ),
        video_path=(
            None if args_cli.video_path is None else str(args_cli.video_path)
        ),
        video_frame_count=len(video_frames),
    )
    if args_cli.video_path is not None:
        import imageio.v2 as imageio

        args_cli.video_path.parent.mkdir(parents=True, exist_ok=True)
        with imageio.get_writer(
            args_cli.video_path, fps=60, codec="libx264", quality=8
        ) as writer:
            for frame in video_frames:
                writer.append_data(frame)
    if camera_measurements:
        (args_cli.output / "camera_measurements.json").write_text(
            json.dumps(camera_measurements, indent=2) + "\n",
            encoding="utf-8",
        )
    (args_cli.output / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n",
        encoding="utf-8",
    )
    (args_cli.output / "trajectory.json").write_text(
        json.dumps(records, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, indent=2))
    if not summary["prethrow_stable"]:
        raise RuntimeError("cube did not remain stably grasped through throw")
    if not summary["detach_detected"]:
        raise RuntimeError("partial-open command did not produce detach")
    if (
        args_cli.catch_servo_start_time_s is not None
        and not summary["catch_stable"]
    ):
        raise RuntimeError("closed-loop servo did not produce a stable catch")
    print("native_release_smoke=PASS")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        simulation_app.close()
