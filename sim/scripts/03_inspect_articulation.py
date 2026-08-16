#!/usr/bin/env python3
"""Inspect the imported xArm6+G1 articulation and its G1 mimic motion."""

from __future__ import annotations

import argparse
from pathlib import Path

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser()
parser.add_argument(
    "--usd",
    type=Path,
    default=(
        Path(__file__).resolve().parents[1]
        / "assets"
        / "xarm6_g1"
        / "xarm6_g1.usd"
        / "xarm6_g1"
        / "xarm6_g1.usda"
    ),
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg


ARM_START = (
    0.061075,
    -0.103783710,
    -1.121817371,
    0.022688,
    2.359877067,
    0.331612,
)
GRIPPER_JOINT_PATTERN = (
    "drive_joint|left_finger_joint|left_inner_knuckle_joint|"
    "right_outer_knuckle_joint|right_finger_joint|right_inner_knuckle_joint"
)


def robot_cfg(usd_path: Path) -> ArticulationCfg:
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
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                **{
                    f"joint{index + 1}": value
                    for index, value in enumerate(ARM_START)
                },
                "drive_joint": 0.37,
                "left_finger_joint": 0.37,
                "left_inner_knuckle_joint": 0.37,
                "right_outer_knuckle_joint": 0.37,
                "right_finger_joint": 0.37,
                "right_inner_knuckle_joint": 0.37,
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
                joint_names_expr=[
                    "left_finger_joint",
                    "left_inner_knuckle_joint",
                    "right_outer_knuckle_joint",
                    "right_finger_joint",
                    "right_inner_knuckle_joint",
                ],
                effort_limit_sim=1.0,
                velocity_limit_sim=2.0,
                stiffness=0.0,
                damping=0.0,
            ),
        },
    )


def step(
    sim: sim_utils.SimulationContext,
    robot: Articulation,
    count: int,
) -> None:
    dt = sim.get_physics_dt()
    for _ in range(count):
        robot.write_data_to_sim()
        sim.step(render=False)
        robot.update(dt)


def joint_snapshot(robot: Articulation) -> dict[str, float]:
    values = robot.data.joint_pos.torch[0]
    return {
        name: float(values[index].item())
        for index, name in enumerate(robot.joint_names)
    }


def main() -> int:
    if not args_cli.usd.is_file():
        raise FileNotFoundError(args_cli.usd)

    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=0.005, device=args_cli.device)
    )
    robot = Articulation(robot_cfg(args_cli.usd))
    sim.reset()

    print(f"usd={args_cli.usd.resolve()}")
    print(f"joint_names={robot.joint_names}")
    print(f"body_names={robot.body_names}")
    print(f"actuators={list(robot.actuators)}")
    print(f"link_paths={robot.root_physx_view.link_paths}")

    arm_ids, arm_names = robot.find_joints("joint[1-6]")
    gripper_ids, gripper_names = robot.find_joints(GRIPPER_JOINT_PATTERN)
    drive_ids, _ = robot.find_joints("drive_joint")
    finger_body_ids, finger_body_names = robot.find_bodies(
        "left_finger|right_finger")
    print(f"arm_joint_ids={arm_ids}, names={arm_names}")
    stage = sim_utils.get_current_stage()
    for path in robot.root_physx_view.link_paths[0][-2:]:
        print(
            f"contact_schema path={path} "
            f"schemas={stage.GetPrimAtPath(path).GetAppliedSchemas()}"
        )
    print(f"gripper_joint_ids={gripper_ids}, names={gripper_names}")
    if len(arm_ids) != 6 or len(gripper_ids) != 6 or len(drive_ids) != 1:
        raise RuntimeError("unexpected xArm6/G1 articulation layout")

    q = robot.data.default_joint_pos.torch.clone()
    dq = robot.data.default_joint_vel.torch.clone()
    robot.write_joint_position_to_sim_index(position=q)
    robot.write_joint_velocity_to_sim_index(velocity=dq)
    robot.set_joint_position_target_index(target=q)
    robot.reset()
    step(sim, robot, 40)
    held = joint_snapshot(robot)
    finger_poses = robot.data.body_pose_w.torch[0, finger_body_ids]
    print(
        "finger_body_poses="
        + str({
            name: [round(float(value), 6) for value in pose.tolist()]
            for name, pose in zip(finger_body_names, finger_poses, strict=True)
        })
    )

    drive_target = torch.tensor([[0.52]], device=sim.device)
    robot.set_joint_position_target_index(
        target=drive_target,
        joint_ids=drive_ids,
    )
    step(sim, robot, 80)
    opened = joint_snapshot(robot)

    print("held_g1=" + str({name: round(held[name], 6) for name in gripper_names}))
    print("opened_g1=" + str({name: round(opened[name], 6) for name in gripper_names}))
    deltas = {
        name: opened[name] - held[name]
        for name in gripper_names
    }
    print("g1_delta=" + str({name: round(value, 6) for name, value in deltas.items()}))

    if abs(opened["drive_joint"] - 0.52) > 0.03:
        raise RuntimeError("G1 drive_joint did not reach partial-open target")
    passive_motion = [
        abs(deltas[name])
        for name in gripper_names
        if name != "drive_joint"
    ]
    if min(passive_motion) < 0.05:
        raise RuntimeError("one or more G1 mimic joints did not follow drive_joint")

    print("articulation_inspection=PASS")
    simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
