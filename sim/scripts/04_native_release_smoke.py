#!/usr/bin/env python3
"""Native xArm6+G1 grasp and partial-release smoke with a light cube.

The cube pose and velocity are written exactly once at episode initialization.
Every subsequent state is produced by the arm controller, G1 contacts and
PhysX rigid-body dynamics.
"""

from __future__ import annotations

import argparse
import json
import math
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
parser.add_argument(
    "--cube-rotation-hand-y-deg",
    type=float,
    default=0.0,
    help="Fixed cube rotation about hand-local Y at the one-time placement.",
)
parser.add_argument(
    "--cube-friction-combine-mode",
    choices=("average", "min", "multiply", "max"),
    default="max",
    help="PhysX material combine mode; insert studies use multiply.",
)
parser.add_argument(
    "--cube-static-friction",
    type=float,
    default=None,
    help="Override config cube static friction for sensitivity/domain randomization.",
)
parser.add_argument(
    "--cube-dynamic-friction",
    type=float,
    default=None,
    help="Override config cube dynamic friction for sensitivity/domain randomization.",
)
parser.add_argument(
    "--release-insert-side",
    choices=("none", "left", "right", "both"),
    default="none",
    help="Attach a passive rolling-release insert to one or both G1 fingers.",
)
parser.add_argument(
    "--release-insert-shape",
    choices=(
        "dome", "ridge_x", "ramp_x", "roller_x", "d_roller_x",
        "spinner_y", "flipper_y",
    ),
    default="dome",
    help="Use a fixed insert, roller, spin disk, or eccentric release flipper.",
)
parser.add_argument(
    "--release-d-roller-chord-m",
    type=float,
    default=0.002,
    help="Flat-face offset from the D-roller rotation axis toward the cube.",
)
parser.add_argument(
    "--release-insert-ramp-deg",
    type=float,
    default=10.0,
    help="Ramp rotation about finger-local X; sign controls tumble direction.",
)
parser.add_argument(
    "--release-insert-ramp-height-m",
    type=float,
    default=0.020,
    help="Ramp extent along finger-local Z.",
)
parser.add_argument(
    "--release-insert-radius-m",
    type=float,
    default=0.002,
    help="Radius of the passive fingertip release ridge.",
)
parser.add_argument(
    "--release-flipper-pivot-sign",
    type=int,
    choices=(-1, 1),
    default=1,
    help="Select the -Z or +Z end pivot for the eccentric Y-axis flipper.",
)
parser.add_argument(
    "--release-insert-length-m",
    type=float,
    default=0.024,
    help="Cylinder length along the finger-local X axis.",
)
parser.add_argument(
    "--release-insert-finger-x-m",
    type=float,
    default=0.0,
    help="Insert center along finger-local X; use an edge patch for tumble leverage.",
)
parser.add_argument(
    "--release-insert-finger-z-m",
    type=float,
    default=0.030,
    help="Ridge center along the selected finger-local Z axis.",
)
parser.add_argument(
    "--release-insert-static-friction",
    type=float,
    default=1.3,
)
parser.add_argument(
    "--release-insert-dynamic-friction",
    type=float,
    default=0.9,
)
parser.add_argument(
    "--release-roller-one-way",
    action="store_true",
    help="Lock negative roller rotation while allowing positive release rolling.",
)
parser.add_argument(
    "--release-roller-open-triggered-clutch",
    action="store_true",
    help="Lock the roller while held and disengage at measured G1 opening onset.",
)
parser.add_argument(
    "--release-roller-cam-travel-rad",
    type=float,
    default=0.0,
    help="G1-coupled D-roller travel after physical opening starts; zero is free-wheel.",
)
parser.add_argument(
    "--release-roller-cam-direction",
    type=int,
    choices=(-1, 1),
    default=-1,
    help="Signed D-roller rotation direction for the G1-coupled release cam.",
)
parser.add_argument(
    "--release-roller-cam-lead-s",
    type=float,
    default=0.0,
    help="Lead before physical G1 opening for a triggered preloaded spin insert.",
)
parser.add_argument(
    "--release-roller-cam-delay-s",
    type=float,
    default=0.0,
    help="Delay after physical G1 opening before the spin insert starts.",
)
parser.add_argument(
    "--release-roller-cam-freewheel-after-transition",
    action="store_true",
    help="Disengage the preloaded spinner drive after its kick completes.",
)
parser.add_argument(
    "--release-roller-cam-transition-s",
    type=float,
    default=0.040,
    help="Time for the G1-driven passive cam to complete its roller travel.",
)
parser.add_argument(
    "--release-roller-cam-effort-limit-nm",
    type=float,
    default=0.020,
    help="Torque cap representing force available through the printable G1 cam.",
)
parser.add_argument(
    "--release-roller-cam-direct-effort",
    action="store_true",
    help="Apply a constant-torque spring pulse instead of an implicit position drive.",
)
parser.add_argument(
    "--release-roller-cam-stiffness-nm-rad",
    type=float,
    default=0.50,
)
parser.add_argument(
    "--release-roller-cam-damping-nm-s-rad",
    type=float,
    default=0.01,
)
parser.add_argument(
    "--release-retract-pad-right",
    action="store_true",
    help="Add a G1-coupled right flat pad that retracts before left-roller detach.",
)
parser.add_argument(
    "--release-retract-pad-axis",
    choices=("X", "Y", "Z"),
    default="Y",
    help="Pad travel axis; X/Z probe a passive tangential flick tab.",
)
parser.add_argument(
    "--release-retract-pad-outward-angle-deg",
    type=float,
    default=0.0,
    help="For X travel, angle the negative stroke toward right-finger retract -Y.",
)
parser.add_argument(
    "--release-retract-pad-thickness-m",
    type=float,
    default=0.008,
)
parser.add_argument(
    "--release-retract-pad-height-m",
    type=float,
    default=0.020,
)
parser.add_argument(
    "--release-retract-pad-static-friction",
    type=float,
    default=None,
    help="Override the right flick-pad friction independently of the left support.",
)
parser.add_argument(
    "--release-retract-pad-dynamic-friction",
    type=float,
    default=None,
)
parser.add_argument(
    "--release-retract-pad-length-m",
    type=float,
    default=None,
)
parser.add_argument(
    "--release-retract-pad-finger-x-m",
    type=float,
    default=None,
)
parser.add_argument(
    "--release-retract-pad-finger-y-m",
    type=float,
    default=None,
)
parser.add_argument(
    "--release-retract-pad-finger-z-m",
    type=float,
    default=None,
)
parser.add_argument(
    "--release-retract-pad-travel-m",
    type=float,
    default=0.010,
)
parser.add_argument(
    "--release-retract-pad-delay-s",
    type=float,
    default=0.0,
    help="Delay right-pad retraction until the G1-coupled roller kick completes.",
)
parser.add_argument(
    "--release-retract-pad-transition-s",
    type=float,
    default=0.040,
)
parser.add_argument(
    "--release-retract-pad-return-transition-s",
    type=float,
    default=None,
)
parser.add_argument(
    "--release-retract-pad-hold-s",
    type=float,
    default=0.0,
)
parser.add_argument(
    "--release-retract-pad-radial-clear-m",
    type=float,
    default=0.0,
)
parser.add_argument(
    "--release-retract-pad-radial-clear-delay-s",
    type=float,
    default=0.005,
)
parser.add_argument(
    "--release-retract-pad-radial-clear-transition-s",
    type=float,
    default=0.002,
)
parser.add_argument(
    "--release-retract-pad-radial-support",
    action="store_true",
    help=(
        "Use the radial carriage as a separate grasp support so the tangential "
        "striker can fire only after the support unloads."
    ),
)
parser.add_argument(
    "--release-radial-support-length-m",
    type=float,
    default=0.012,
)
parser.add_argument(
    "--release-radial-support-thickness-m",
    type=float,
    default=0.004,
)
parser.add_argument(
    "--release-radial-support-height-m",
    type=float,
    default=0.020,
)
parser.add_argument(
    "--release-radial-support-finger-x-m",
    type=float,
    default=0.0,
)
parser.add_argument(
    "--release-radial-support-finger-y-m",
    type=float,
    default=0.0260032,
)
parser.add_argument(
    "--release-radial-support-finger-z-m",
    type=float,
    default=0.030,
)
parser.add_argument(
    "--release-radial-support-static-friction",
    type=float,
    default=None,
)
parser.add_argument(
    "--release-radial-support-dynamic-friction",
    type=float,
    default=None,
)
parser.add_argument(
    "--release-retract-pad-effort-limit-n",
    type=float,
    default=10.0,
)
parser.add_argument(
    "--release-roller-radial-retract-m",
    type=float,
    default=0.0,
    help="Outward travel of the left D-roller prismatic carriage.",
)
parser.add_argument(
    "--release-roller-radial-retract-delay-s",
    type=float,
    default=0.025,
    help="Delay from measured G1 motion start to left carriage retraction.",
)
parser.add_argument(
    "--release-roller-radial-retract-transition-s",
    type=float,
    default=0.015,
)
parser.add_argument(
    "--release-roller-radial-retract-effort-limit-n",
    type=float,
    default=10.0,
)
parser.add_argument(
    "--release-roller-radial-retract-stiffness-n-m",
    type=float,
    default=2000.0,
)
parser.add_argument(
    "--release-roller-radial-retract-damping-n-s-m",
    type=float,
    default=10.0,
)
parser.add_argument(
    "--release-roller-radial-retract-kinematic",
    action="store_true",
    help="Capability probe for a stiff G1 cam that imposes carriage q/dq.",
)
parser.add_argument(
    "--release-retract-pad-force-limited",
    action="store_true",
    help="Keep radial clearance imposed but drive the striker through its force cap.",
)
parser.add_argument(
    "--release-retract-pad-freewheel-after-transition",
    action="store_true",
    help="Disengage the force-limited striker after its commanded kick completes.",
)
parser.add_argument(
    "--release-retract-pad-direct-effort",
    action="store_true",
    help="Drive the spring tab with a constant force pulse instead of a position target.",
)
parser.add_argument(
    "--release-retract-pad-direct-effort-direction",
    type=int,
    choices=(-1, 1),
    default=-1,
    help="Signed prismatic force direction for the direct-effort spring tab.",
)
parser.add_argument(
    "--release-retract-pad-one-way",
    action="store_true",
    help="Model a passive ratchet that prevents the spring tab from rebounding.",
)
parser.add_argument(
    "--release-retract-pad-stiffness-n-m",
    type=float,
    default=2000.0,
)
parser.add_argument(
    "--release-retract-pad-damping-n-s-m",
    type=float,
    default=10.0,
)
parser.add_argument("--held-drive-rad", type=float, default=0.37)
parser.add_argument("--held-gripper-effort-limit-n", type=float, default=None)
parser.add_argument("--partial-open-drive-rad", type=float, default=0.52)
parser.add_argument("--release-gripper-effort-limit-n", type=float, default=None)
parser.add_argument("--release-gripper-stiffness", type=float, default=None)
parser.add_argument("--catch-drive-rad", type=float, default=None)
parser.add_argument("--catch-gripper-effort-limit-n", type=float, default=None)
parser.add_argument("--catch-gripper-stiffness", type=float, default=None)
parser.add_argument("--joint6-roll-offset-rad", type=float, default=0.0)
parser.add_argument("--settle-s", type=float, default=0.40)
parser.add_argument("--post-release-s", type=float, default=0.50)
parser.add_argument("--release-time-s", type=float, default=0.60)
parser.add_argument(
    "--gripper-open-command-time-s",
    type=float,
    default=None,
    help="Allow the nonblocking G1 open command to lead kinematic release.",
)
parser.add_argument(
    "--gripper-preopen-command-time-s",
    type=float,
    default=None,
    help="Optional first-stage G1 command that unloads symmetric pinch before flick.",
)
parser.add_argument(
    "--gripper-preopen-drive-rad",
    type=float,
    default=None,
    help="G1 drive target for the first-stage rolling-release preload.",
)
parser.add_argument(
    "--gripper-preopen-transition-s",
    type=float,
    default=None,
    help="Measured-response transition duration for the first-stage preload.",
)
parser.add_argument(
    "--release-drive-transition-s",
    type=float,
    default=None,
    help="Replay a measured G1 held-to-partial-open drive transition.",
)
parser.add_argument(
    "--release-dynamics-after-transition",
    action="store_true",
    help="Keep held G1 gains during the drive transition, then apply release gains.",
)
parser.add_argument(
    "--release-dynamics-before-transition",
    action="store_true",
    help="Apply release gains before the G1 drive transition; overrides after-transition mode.",
)
parser.add_argument(
    "--release-drive-start-delay-s",
    type=float,
    default=0.0,
    help="Command-to-motion latency before replaying the G1 transition.",
)
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
    "--catch-preclose-time-s",
    type=float,
    default=None,
    help="Command an intermediate G1 aperture before final catch close.",
)
parser.add_argument(
    "--catch-preclose-drive-rad",
    type=float,
    default=None,
    help="Intermediate G1 drive target used by --catch-preclose-time-s.",
)
parser.add_argument(
    "--vision-control-end-time-s",
    type=float,
    default=None,
    help="Last episode time at which camera state may update the arm target.",
)
parser.add_argument("--catch-servo-gain", type=float, default=0.75)
parser.add_argument(
    "--catch-lock-wrist",
    action="store_true",
    help="Use joints 1-3 for catch translation and preserve the release wrist pose.",
)
parser.add_argument(
    "--catch-j5-weight",
    type=float,
    default=1.0,
    help="Weighted-pseudoinverse preference for J5 with --catch-allow-j5.",
)
parser.add_argument(
    "--catch-joint-target-rad",
    type=float,
    nargs=6,
    default=None,
    metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
    help="Explicit joint-space catch target used for IK-path validation.",
)
parser.add_argument(
    "--catch-joint-pretarget-rad",
    type=float,
    nargs=6,
    default=None,
    metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
    help="Joint-space clearance waypoint used before the intercept switch time.",
)
parser.add_argument(
    "--catch-allow-j5",
    action="store_true",
    help="With --catch-lock-wrist, use J1/J2/J3/J5 while preserving J4/J6.",
)
parser.add_argument(
    "--catch-lateral-only",
    action="store_true",
    help="Use joint1 for third-view lateral centering and keep nominal q2-q6.",
)
parser.add_argument(
    "--catch-hold-throw-joints",
    action="store_true",
    help="At catch-servo activation hold joints 2-6 at their measured pose.",
)
parser.add_argument(
    "--catch-preserve-nominal-momentum",
    action="store_true",
    help="Enter catch servo without resetting the nominal commanded q/dq.",
)
parser.add_argument(
    "--catch-joint1-start-time-s",
    type=float,
    default=None,
    help="Freeze J1 during early catch preposition, then enable lateral centering.",
)
parser.add_argument(
    "--catch-max-joint-step-rad",
    type=float,
    default=0.035,
)
parser.add_argument(
    "--catch-max-joint-speed-rad-s",
    type=float,
    default=None,
)
parser.add_argument(
    "--catch-max-joint-acceleration-rad-s2",
    type=float,
    default=None,
)
parser.add_argument(
    "--catch-prediction-horizon-s",
    type=float,
    default=0.0,
    help="Ballistic lookahead used for the arm intercept target.",
)
parser.add_argument(
    "--catch-position-bias-m",
    type=float,
    nargs=3,
    default=(0.0, 0.0, 0.0),
    metavar=("X", "Y", "Z"),
)
parser.add_argument(
    "--catch-preposition-bias-m",
    type=float,
    nargs=3,
    default=None,
    metavar=("X", "Y", "Z"),
    help="Temporary intercept bias for a collision-free pre-catch waypoint.",
)
parser.add_argument(
    "--catch-preposition-start-time-s",
    type=float,
    default=None,
    help="Episode time at which the temporary pre-catch bias becomes active.",
)
parser.add_argument(
    "--catch-preposition-end-time-s",
    type=float,
    default=None,
    help="Episode time at which the temporary pre-catch bias is removed.",
)
parser.add_argument(
    "--catch-intercept-time-s",
    type=float,
    default=None,
    help="Fixed episode time for the ballistic catch intercept.",
)
parser.add_argument(
    "--catch-preintercept-time-s",
    type=float,
    default=None,
    help="Early safe ballistic intercept used before the intercept switch time.",
)
parser.add_argument(
    "--catch-intercept-switch-time-s",
    type=float,
    default=None,
    help="Episode time at which catch targeting switches to --catch-intercept-time-s.",
)
parser.add_argument(
    "--detach-delay-prior-s",
    type=float,
    default=0.05,
    help="Encoder/FK prior for physical detach after the open command.",
)
parser.add_argument(
    "--detach-observer-drive-delta-rad",
    type=float,
    default=None,
    help="Freeze the FK release prior after this measured G1 opening travel.",
)
parser.add_argument(
    "--detach-observer-opening-effort-nm",
    type=float,
    default=None,
    help="Freeze the FK release prior on this opening-direction G1 effort.",
)
parser.add_argument(
    "--intercept-residual-model",
    type=Path,
    default=None,
    help="Frozen deployable delta-intercept checkpoint.",
)
parser.add_argument(
    "--residual-min-camera-samples",
    type=int,
    default=1,
    help="Do not apply the learned residual before this many camera samples.",
)
parser.add_argument(
    "--catch-min-camera-samples",
    type=int,
    default=1,
    help="Hold the measured arm pose until the camera belief has this many samples.",
)
parser.add_argument(
    "--catch-evidence-window-s",
    type=float,
    default=0.50,
)
parser.add_argument(
    "--observation-mode",
    choices=("physics", "proprioceptive", "global_camera", "policy_cameras"),
    default="physics",
    help=(
        "Use simulator truth, deployable q/dq release-state propagation, "
        "the third-view camera, or third-view+wrist policy cameras."
    ),
)
parser.add_argument(
    "--record-policy-cameras",
    action="store_true",
    help="Record third-view and wrist video without using either for control.",
)
parser.add_argument(
    "--wrist-camera-hardware-removed",
    action="store_true",
    help=(
        "Disable the wrist D435/mount collision proxy; recorded wrist view is virtual-only."
    ),
)
parser.add_argument("--arm-tracking-delay-s", type=float, default=0.09)
parser.add_argument(
    "--arm-drive-interpolation",
    choices=("hold", "linear"),
    default="hold",
    help="Low-level interpolation between 20 ms servo_j targets.",
)
parser.add_argument("--arm-sim-effort-scale", type=float, default=1.0)
parser.add_argument("--arm-sim-stiffness-scale", type=float, default=1.0)
parser.add_argument(
    "--probe-j-config",
    type=Path,
    default=None,
    help="Paired-signal Probe calibration and executable catch candidates.",
)
parser.add_argument("--camera-receive-latency-s", type=float, default=0.02)
parser.add_argument("--camera-dropout-probability", type=float, default=0.05)
parser.add_argument("--camera-seed", type=int, default=20260816)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg, RigidObject, RigidObjectCfg
from isaaclab.sim.spawners.meshes import MeshCylinderCfg
from isaaclab.sim.spawners.meshes.meshes import _spawn_mesh_geom_from_mesh
from pxr import Gf, Sdf, UsdGeom, UsdPhysics
from isaaclab.sensors import Camera, CameraCfg, ContactSensor, ContactSensorCfg
from isaaclab.utils.math import convert_quat, quat_apply, quat_apply_inverse, quat_mul
import trimesh


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
from xarm6_toss.probe_j import (  # noqa: E402
    estimate_probe_posterior,
    probe_joint_offset_rad,
    select_catch_candidate,
)
from xarm6_toss.release_insert import (  # noqa: E402
    d_roller_mesh,
    strike_return_command,
)
from xarm6_toss.flight import (  # noqa: E402
    continuous_free_flight_evidence,
    cube_ground_clearance_m,
    g1_release_response,
    release_transfer_evidence,
)
from xarm6_toss.motion_limits import (  # noqa: E402
    evaluate_joint_trajectory,
    evaluate_reference_samples,
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
LINK_EEF_PRIM = GRIPPER_PRIM.rsplit("/xarm_gripper_base_link", 1)[0]
LEFT_FINGER_PRIM = (
    GRIPPER_PRIM + "/left_outer_knuckle/left_finger"
)
RIGHT_FINGER_PRIM = (
    GRIPPER_PRIM + "/right_outer_knuckle/right_finger"
)

ROLLER_JOINT_NAME = "release_roller_joint"
ROLLER_JOINT_PRIM = "/World/XArm6/Physics/release_roller_joint"
RIGHT_ROLLER_JOINT_NAME = "right_release_roller_joint"
RIGHT_ROLLER_JOINT_PRIM = "/World/XArm6/Physics/right_release_roller_joint"
ROLLER_INSERT_SHAPES = ("roller_x", "d_roller_x", "spinner_y", "flipper_y")
RETRACT_JOINT_NAME = "right_release_retract_joint"
RETRACT_JOINT_PRIM = "/World/XArm6/Physics/right_release_retract_joint"
RETRACT_PAD_RADIAL_JOINT_NAME = "right_release_pad_radial_joint"
RETRACT_PAD_RADIAL_JOINT_PRIM = "/World/XArm6/Physics/right_release_pad_radial_joint"
RETRACT_PAD_CARRIAGE_PRIM = "/World/XArm6/ReleaseMechanism/RightPadCarriage"
ROLLER_RETRACT_JOINT_NAME = "left_roller_retract_joint"
ROLLER_RETRACT_JOINT_PRIM = "/World/XArm6/Physics/left_roller_retract_joint"
RIGHT_ROLLER_RETRACT_JOINT_NAME = "right_roller_retract_joint"
RIGHT_ROLLER_RETRACT_JOINT_PRIM = "/World/XArm6/Physics/right_roller_retract_joint"
ROLLER_CARRIAGE_PRIM = "/World/XArm6/ReleaseMechanism/LeftRollerCarriage"
RIGHT_ROLLER_CARRIAGE_PRIM = "/World/XArm6/ReleaseMechanism/RightRollerCarriage"
LINK6_PRIM = LINK_EEF_PRIM.rsplit("/link_eef", 1)[0]
LINK5_PRIM = LINK6_PRIM.rsplit("/link6", 1)[0]
LINK4_PRIM = LINK5_PRIM.rsplit("/link5", 1)[0]


def inverse_usd_quaternion(orientation_xyzw: list[float]) -> Gf.Quatf:
    """Map a spawned body orientation to its inverse joint-frame rotation."""
    x, y, z, w = orientation_xyzw
    return Gf.Quatf(float(w), float(-x), float(-y), float(-z))


def root_relative_pose_from_parent(
    parent_prim_path: str,
    local_translation_m: list[float] | tuple[float, float, float],
    local_orientation_xyzw: list[float] | tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    """Compose a finger-local pose without nesting a rigid body below the finger."""
    stage = sim_utils.get_current_stage()
    cache = UsdGeom.XformCache()
    parent_world = cache.GetLocalToWorldTransform(
        stage.GetPrimAtPath(parent_prim_path)
    )
    root_world = cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/XArm6"))
    x, y, z, w = local_orientation_xyzw
    local_transform = Gf.Transform()
    local_transform.SetTranslation(Gf.Vec3d(*local_translation_m))
    local_transform.SetRotation(
        Gf.Rotation(Gf.Quatd(float(w), Gf.Vec3d(float(x), float(y), float(z))))
    )
    root_relative = local_transform.GetMatrix() * parent_world * root_world.GetInverse()
    translation = root_relative.ExtractTranslation()
    rotation = root_relative.ExtractRotationQuat()
    imaginary = rotation.GetImaginary()
    return (
        tuple(float(value) for value in translation),
        (float(imaginary[0]), float(imaginary[1]), float(imaginary[2]), float(rotation.GetReal())),
    )


ROBOT_CONTACT_PRIMS = {
    "left_finger": LEFT_FINGER_PRIM,
    "right_finger": RIGHT_FINGER_PRIM,
    "left_outer_knuckle": GRIPPER_PRIM + "/left_outer_knuckle",
    "left_inner_knuckle": GRIPPER_PRIM + "/left_inner_knuckle",
    "right_outer_knuckle": GRIPPER_PRIM + "/right_outer_knuckle",
    "right_inner_knuckle": GRIPPER_PRIM + "/right_inner_knuckle",
    "gripper_base": GRIPPER_PRIM,
    "link6": LINK6_PRIM,
    "link5": LINK5_PRIM,
    "link4": LINK4_PRIM,
}
# D435 outer envelope in its optical frame. This is deliberately larger than
# the imager origin; it is a robot-cube collision proxy, not a visual marker.
WRIST_CAMERA_PROXY_HALF_EXTENTS_M = torch.tensor(
    [0.0475, 0.0175, 0.0175], dtype=torch.float32
)
ROBOT_CONTACT_FORCE_THRESHOLD_N = 0.05

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
WRIST_CAMERA_POSITION_EEF_M = (0.06950393, 0.03858712, 0.02487193)
WRIST_CAMERA_QUAT_XYZW = (0.01337523, -0.00833076, -0.70187405, 0.71212676)
SPECTATOR_CAMERA_POSITION_M = (0.905, -0.680, 0.694)
SPECTATOR_CAMERA_QUAT_XYZW = (0.80707536, 0.21277731, -0.14040912, -0.53257906)
SPECTATOR_CAMERA_K = np.asarray(
    [[700.0, 0.0, 480.0], [0.0, 700.0, 270.0], [0.0, 0.0, 1.0]],
    dtype=np.float32,
)


def robot_cfg(usd_path: Path, held_drive_rad: float) -> ArticulationCfg:
    config = json.loads(args_cli.config.read_text(encoding="utf-8"))
    start_q = config["reference_segments"][0]["start_joint_rad"]
    arm_velocity_limit = float(
        config.get("limits", {}).get("joint_speed_rad_s", 0.45)
    )
    gripper_sim = config.get("gripper_sim", {})
    return ArticulationCfg(
        prim_path="/World/XArm6",
        spawn=sim_utils.UsdFileCfg(
            usd_path=str(usd_path),
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=float(
                    gripper_sim.get("max_depenetration_velocity_m_s", 2.0)
                ),
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=4,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                contact_offset=float(gripper_sim.get("contact_offset_m", 0.001)),
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
                **(
                    {
                        name: 0.0
                        for name in (
                            (ROLLER_JOINT_NAME, RIGHT_ROLLER_JOINT_NAME)
                            if args_cli.release_insert_side == "both"
                            else (ROLLER_JOINT_NAME,)
                        )
                    }
                    if args_cli.release_insert_side != "none"
                    and args_cli.release_insert_shape in ROLLER_INSERT_SHAPES
                    else {}
                ),
                **(
                    {RETRACT_JOINT_NAME: 0.0} if args_cli.release_retract_pad_right else {}
                ),
                **(
                    {RETRACT_PAD_RADIAL_JOINT_NAME: 0.0}
                    if args_cli.release_retract_pad_radial_clear_m > 0.0
                    else {}
                ),
                **(
                    {
                        name: 0.0
                        for name in (
                            (ROLLER_RETRACT_JOINT_NAME, RIGHT_ROLLER_RETRACT_JOINT_NAME)
                            if args_cli.release_insert_side == "both"
                            else (ROLLER_RETRACT_JOINT_NAME,)
                        )
                    }
                    if args_cli.release_roller_radial_retract_m > 0.0
                    else {}
                ),
            },
        ),
        actuators={
            "arm": ImplicitActuatorCfg(
                joint_names_expr=["joint[1-6]"],
                effort_limit_sim={
                    "joint[1-2]": 50.0 * args_cli.arm_sim_effort_scale,
                    "joint[3-5]": 32.0 * args_cli.arm_sim_effort_scale,
                    "joint6": 20.0 * args_cli.arm_sim_effort_scale,
                },
                velocity_limit_sim=arm_velocity_limit,
                stiffness=400.0 * args_cli.arm_sim_stiffness_scale,
                damping=40.0 * args_cli.arm_sim_stiffness_scale**0.5,
            ),
            "gripper_drive": ImplicitActuatorCfg(
                joint_names_expr=["drive_joint"],
                effort_limit_sim=float(
                    gripper_sim.get("effort_limit_n", 50.0)
                    if args_cli.held_gripper_effort_limit_n is None
                    else args_cli.held_gripper_effort_limit_n
                ),
                velocity_limit_sim=5.0,
                stiffness=float(
                    gripper_sim.get("stiffness_n_m", 120.0)
                ),
                damping=float(
                    gripper_sim.get("damping_n_s_m", 5.0)
                ),
            ),
            "gripper_passive": ImplicitActuatorCfg(
                joint_names_expr=list(GRIPPER_PASSIVE_JOINTS),
                effort_limit_sim=1.0,
                velocity_limit_sim=2.0,
                stiffness=0.0,
                damping=0.0,
            ),
            **(
                {
                    "release_roller": ImplicitActuatorCfg(
                        joint_names_expr=list(
                            (
                                ROLLER_JOINT_NAME,
                                RIGHT_ROLLER_JOINT_NAME,
                            )
                            if args_cli.release_insert_side == "both"
                            else (ROLLER_JOINT_NAME,)
                        ),
                        effort_limit_sim=(
                            args_cli.release_roller_cam_effort_limit_nm
                            if args_cli.release_roller_cam_travel_rad > 0.0
                            else 0.20
                        ),
                        velocity_limit_sim=50.0,
                        stiffness=(
                            args_cli.release_roller_cam_stiffness_nm_rad
                            if args_cli.release_roller_cam_travel_rad > 0.0
                            else 0.0
                        ),
                        damping=(
                            args_cli.release_roller_cam_damping_nm_s_rad
                            if args_cli.release_roller_cam_travel_rad > 0.0
                            else 0.0
                        ),
                    )
                }
                if (
                    args_cli.release_insert_side != "none"
                    and args_cli.release_insert_shape in ROLLER_INSERT_SHAPES
                )
                else {}
            ),
            **(
                {
                    "release_retract_pad": ImplicitActuatorCfg(
                        joint_names_expr=[RETRACT_JOINT_NAME],
                        effort_limit_sim=args_cli.release_retract_pad_effort_limit_n,
                        velocity_limit_sim=1.0,
                        stiffness=args_cli.release_retract_pad_stiffness_n_m,
                        damping=args_cli.release_retract_pad_damping_n_s_m,
                    )
                }
                if args_cli.release_retract_pad_right else {}
            ),
            **(
                {
                    "release_retract_pad_radial": ImplicitActuatorCfg(
                        joint_names_expr=[RETRACT_PAD_RADIAL_JOINT_NAME],
                        effort_limit_sim=args_cli.release_retract_pad_effort_limit_n,
                        velocity_limit_sim=2.0,
                        stiffness=args_cli.release_retract_pad_stiffness_n_m,
                        damping=args_cli.release_retract_pad_damping_n_s_m,
                    )
                }
                if (
                    args_cli.release_retract_pad_right
                    and args_cli.release_retract_pad_radial_clear_m > 0.0
                )
                else {}
            ),
            **(
                {
                    "release_roller_retract": ImplicitActuatorCfg(
                        joint_names_expr=list(
                            (ROLLER_RETRACT_JOINT_NAME, RIGHT_ROLLER_RETRACT_JOINT_NAME)
                            if args_cli.release_insert_side == "both"
                            else (ROLLER_RETRACT_JOINT_NAME,)
                        ),
                        effort_limit_sim=(
                            args_cli.release_roller_radial_retract_effort_limit_n
                        ),
                        velocity_limit_sim=1.0,
                        stiffness=(
                            args_cli.release_roller_radial_retract_stiffness_n_m
                        ),
                        damping=(
                            args_cli.release_roller_radial_retract_damping_n_s_m
                        ),
                    )
                }
                if args_cli.release_roller_radial_retract_m > 0.0 else {}
            ),
        },
    )


def cube_cfg(
    size_m: float, mass_kg: float, cube_physics: dict[str, object]
) -> RigidObjectCfg:
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
                contact_offset=float(
                    cube_physics.get("contact_offset_m", 0.001)
                ),
                rest_offset=0.0,
            ),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=float(
                    cube_physics.get("static_friction", 1.2)
                    if args_cli.cube_static_friction is None
                    else args_cli.cube_static_friction
                ),
                dynamic_friction=float(
                    cube_physics.get("dynamic_friction", 0.9)
                    if args_cli.cube_dynamic_friction is None
                    else args_cli.cube_dynamic_friction
                ),
                restitution=0.02,
                friction_combine_mode=args_cli.cube_friction_combine_mode,
                restitution_combine_mode="min",
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.95, 0.72, 0.05),
                roughness=0.6,
            ),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 2.0)),
    )


def spawn_cube_rotation_marker(size_m: float) -> None:
    """Add a visual-only asymmetric corner marker for spectator evidence."""
    marker = sim_utils.CuboidCfg(
        size=(0.0015, 0.010, 0.010),
        visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=(0.85, 0.03, 0.03),
            roughness=0.5,
        ),
    )
    marker.func(
        "/World/Cube/RotationMarker",
        marker,
        translation=(0.5 * size_m + 0.00075, 0.009, 0.009),
    )


def release_insert_geometry() -> dict[str, object] | None:
    if args_cli.release_insert_side == "none":
        return None
    is_left = args_cli.release_insert_side != "right"
    parent_prim = LEFT_FINGER_PRIM if is_left else RIGHT_FINGER_PRIM
    # The received STL inner surfaces are y=-26.003 mm on the left and
    # y=+26.003 mm on the right. The X offset controls tumble leverage.
    local_translation_m = (
        args_cli.release_insert_finger_x_m,
        -0.0260032 if is_left else 0.0260032,
        args_cli.release_insert_finger_z_m,
    )
    shape = args_cli.release_insert_shape
    ramp_angle_rad = (
        math.radians(args_cli.release_insert_ramp_deg)
        if shape == "ramp_x"
        else 0.0
    )
    orientation_xyzw = (
        (1.0, 0.0, 0.0, 0.0)
        if shape == "d_roller_x" and not is_left
        else (
            math.sin(0.5 * ramp_angle_rad),
            0.0,
            0.0,
            math.cos(0.5 * ramp_angle_rad),
        )
    )
    return {
        "side": args_cli.release_insert_side,
        "prim_path": (
            f"/World/XArm6/ReleaseMechanism/{'Left' if is_left else 'Right'}Roller"
            if shape in ROLLER_INSERT_SHAPES
            else f"{parent_prim}/ReleaseInsert"
        ),
        "shape": shape,
        "parent_prim_path": parent_prim,
        "joint_prim_path": ROLLER_JOINT_PRIM if shape in ROLLER_INSERT_SHAPES else None,
        "mirror_prim_path": (
            "/World/XArm6/ReleaseMechanism/RightRoller"
            if args_cli.release_insert_side == "both" and shape in ROLLER_INSERT_SHAPES
            else (f"{RIGHT_FINGER_PRIM}/ReleaseInsert" if args_cli.release_insert_side == "both" else None)
        ),
        "mirror_parent_prim_path": (
            RIGHT_FINGER_PRIM if args_cli.release_insert_side == "both" else None
        ),
        "mirror_joint_prim_path": (
            RIGHT_ROLLER_JOINT_PRIM
            if args_cli.release_insert_side == "both" and shape in ROLLER_INSERT_SHAPES
            else None
        ),
        "one_way_negative_lock": (
            bool(args_cli.release_roller_one_way) if shape in ROLLER_INSERT_SHAPES else None
        ),
        "open_triggered_clutch": (
            bool(args_cli.release_roller_open_triggered_clutch)
            if shape in ROLLER_INSERT_SHAPES else None
        ),
        "axis": (
            "Y" if shape in ("spinner_y", "flipper_y")
            else None if shape == "dome"
            else "X"
        ),
        "radius_m": args_cli.release_insert_radius_m,
        "flipper_pivot_sign": (
            args_cli.release_flipper_pivot_sign
            if shape == "flipper_y" else None
        ),
        "d_chord_m": args_cli.release_d_roller_chord_m if shape == "d_roller_x" else None,
        "length_m": args_cli.release_insert_length_m,
        "finger_x_m": args_cli.release_insert_finger_x_m,
        "ramp_angle_deg": args_cli.release_insert_ramp_deg if shape == "ramp_x" else None,
        "ramp_height_m": args_cli.release_insert_ramp_height_m if shape == "ramp_x" else None,
        "local_translation_m": list(local_translation_m),
        "local_orientation_xyzw": list(orientation_xyzw),
        "mirror_local_orientation_xyzw": (
            [1.0, 0.0, 0.0, 0.0]
            if shape == "d_roller_x" else list(orientation_xyzw)
        ),
        "cam_travel_rad": args_cli.release_roller_cam_travel_rad,
        "cam_direction": args_cli.release_roller_cam_direction,
        "cam_lead_s": args_cli.release_roller_cam_lead_s,
        "cam_delay_s": args_cli.release_roller_cam_delay_s,
        "cam_freewheel_after_transition": args_cli.release_roller_cam_freewheel_after_transition,
        "cam_direct_effort": args_cli.release_roller_cam_direct_effort,
        "cam_transition_s": args_cli.release_roller_cam_transition_s,
        "cam_effort_limit_nm": args_cli.release_roller_cam_effort_limit_nm,
        "static_friction": args_cli.release_insert_static_friction,
        "dynamic_friction": args_cli.release_insert_dynamic_friction,
    }


def spawn_release_insert() -> dict[str, object] | None:
    geometry = release_insert_geometry()
    if geometry is None:
        return None
    collision_props = sim_utils.CollisionPropertiesCfg(
        contact_offset=0.0002,
        rest_offset=0.0,
    )
    physics_material = sim_utils.RigidBodyMaterialCfg(
        static_friction=float(geometry["static_friction"]),
        dynamic_friction=float(geometry["dynamic_friction"]),
        restitution=0.02,
        friction_combine_mode=args_cli.cube_friction_combine_mode,
        restitution_combine_mode="min",
    )
    visual_material = sim_utils.PreviewSurfaceCfg(
        diffuse_color=(0.05, 0.65, 0.95),
        roughness=0.45,
    )
    if geometry["shape"] == "dome":
        insert = sim_utils.SphereCfg(
            radius=float(geometry["radius_m"]),
            collision_props=collision_props,
            physics_material=physics_material,
            visual_material=visual_material,
        )
    elif geometry["shape"] in ROLLER_INSERT_SHAPES:
        roller_rigid_props = sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            angular_damping=0.0001,
            max_angular_velocity=100.0,
        )
        roller_mass_props = sim_utils.MassPropertiesCfg(mass=0.002)
        if geometry["shape"] == "flipper_y":
            insert = sim_utils.CuboidCfg(
                size=(
                    0.004,
                    float(geometry["length_m"]),
                    2.0 * float(geometry["radius_m"]),
                ),
                rigid_props=roller_rigid_props,
                mass_props=roller_mass_props,
                collision_props=collision_props,
                physics_material=physics_material,
                visual_material=visual_material,
            )
        else:
            insert_cfg = dict(
                radius=float(geometry["radius_m"]),
                height=float(geometry["length_m"]),
                axis=str(geometry["axis"]),
                rigid_props=roller_rigid_props,
                mass_props=roller_mass_props,
                collision_props=collision_props,
                physics_material=physics_material,
                visual_material=visual_material,
            )
            insert = (
                MeshCylinderCfg(**insert_cfg)
                if geometry["shape"] == "d_roller_x"
                else sim_utils.CylinderCfg(**insert_cfg)
            )
    elif geometry["shape"] == "ramp_x":
        insert = sim_utils.CuboidCfg(
            size=(
                float(geometry["length_m"]),
                2.0 * float(geometry["radius_m"]),
                float(geometry["ramp_height_m"]),
            ),
            collision_props=collision_props,
            physics_material=physics_material,
            visual_material=visual_material,
        )
    else:  # ridge_x
        insert = sim_utils.CylinderCfg(
            radius=float(geometry["radius_m"]),
            height=float(geometry["length_m"]),
            axis="X",
            collision_props=collision_props,
            physics_material=physics_material,
            visual_material=visual_material,
        )
    spawn_translation = tuple(geometry["local_translation_m"])
    spawn_orientation = tuple(geometry["local_orientation_xyzw"])
    if geometry["shape"] in ROLLER_INSERT_SHAPES:
        spawn_translation, spawn_orientation = root_relative_pose_from_parent(
            str(geometry["parent_prim_path"]),
            geometry["local_translation_m"],
            geometry["local_orientation_xyzw"],
        )
    if geometry["shape"] == "d_roller_x":
        vertices, faces = d_roller_mesh(
            float(geometry["radius_m"]),
            float(geometry["d_chord_m"]),
            float(geometry["length_m"]),
        )
        mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
        _spawn_mesh_geom_from_mesh(
            str(geometry["prim_path"]), insert, mesh,
            translation=spawn_translation,
            orientation=spawn_orientation,
        )
    else:
        insert.func(
            str(geometry["prim_path"]), insert,
            translation=spawn_translation,
            orientation=spawn_orientation,
        )
    if geometry["shape"] in ROLLER_INSERT_SHAPES:
        stage = sim_utils.get_current_stage()
        radial_retract = args_cli.release_roller_radial_retract_m > 0.0
        if radial_retract:
            carriage = sim_utils.CuboidCfg(
                size=(0.002, 0.002, 0.002),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    disable_gravity=True,
                    max_linear_velocity=1.0,
                ),
                mass_props=sim_utils.MassPropertiesCfg(mass=0.001),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.15, 0.15, 0.15),
                    roughness=0.8,
                ),
            )
            carriage.func(
                ROLLER_CARRIAGE_PRIM,
                carriage,
                translation=spawn_translation,
                orientation=spawn_orientation,
            )
            retract_joint = UsdPhysics.PrismaticJoint.Define(
                stage, ROLLER_RETRACT_JOINT_PRIM
            )
            retract_joint.CreateBody0Rel().SetTargets(
                [Sdf.Path(str(geometry["parent_prim_path"]))]
            )
            retract_joint.CreateBody1Rel().SetTargets(
                [Sdf.Path(ROLLER_CARRIAGE_PRIM)]
            )
            retract_joint.CreateAxisAttr("Y")
            retract_joint.CreateLocalPos0Attr(
                Gf.Vec3f(*geometry["local_translation_m"])
            )
            retract_joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            retract_joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
            retract_joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            retract_joint.CreateLowerLimitAttr(0.0)
            retract_joint.CreateUpperLimitAttr(0.0)
        joint = UsdPhysics.RevoluteJoint.Define(
            stage, str(geometry["joint_prim_path"])
        )
        joint.CreateBody0Rel().SetTargets(
            [
                Sdf.Path(ROLLER_CARRIAGE_PRIM)
                if radial_retract
                else Sdf.Path(str(geometry["parent_prim_path"]))
            ]
        )
        joint.CreateBody1Rel().SetTargets(
            [Sdf.Path(str(geometry["prim_path"]))]
        )
        joint.CreateAxisAttr(str(geometry["axis"]))
        pivot_offset = (
            (
                0.0,
                0.0,
                float(geometry["flipper_pivot_sign"]) * float(geometry["radius_m"]),
            )
            if geometry["shape"] == "flipper_y" else (0.0, 0.0, 0.0)
        )
        joint.CreateLocalPos0Attr(
            Gf.Vec3f(*pivot_offset)
            if radial_retract
            else Gf.Vec3f(*(
                geometry["local_translation_m"][index] + pivot_offset[index]
                for index in range(3)
            ))
        )
        joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        joint.CreateLocalPos1Attr(Gf.Vec3f(*pivot_offset))
        joint.CreateLocalRot1Attr(
            inverse_usd_quaternion(geometry["local_orientation_xyzw"])
        )
        if geometry["one_way_negative_lock"]:
            joint.CreateLowerLimitAttr(0.0)
        elif geometry["open_triggered_clutch"]:
            joint.CreateLowerLimitAttr(0.0)
            joint.CreateUpperLimitAttr(0.0)
    if geometry["side"] == "both":
        mirror_translation = (
            args_cli.release_insert_finger_x_m,
            0.0260032,
            args_cli.release_insert_finger_z_m,
        )
        mirror_spawn_translation, mirror_spawn_orientation = root_relative_pose_from_parent(
            str(geometry["mirror_parent_prim_path"]),
            mirror_translation,
            geometry["mirror_local_orientation_xyzw"],
        )
        if geometry["shape"] == "d_roller_x":
            _spawn_mesh_geom_from_mesh(
                str(geometry["mirror_prim_path"]), insert, mesh,
                translation=mirror_spawn_translation,
                orientation=mirror_spawn_orientation,
            )
        else:
            insert.func(
                str(geometry["mirror_prim_path"]), insert,
                translation=mirror_spawn_translation,
                orientation=mirror_spawn_orientation,
            )
        if radial_retract:
            carriage.func(
                RIGHT_ROLLER_CARRIAGE_PRIM,
                carriage,
                translation=mirror_spawn_translation,
                orientation=mirror_spawn_orientation,
            )
            right_retract_joint = UsdPhysics.PrismaticJoint.Define(
                stage, RIGHT_ROLLER_RETRACT_JOINT_PRIM
            )
            right_retract_joint.CreateBody0Rel().SetTargets(
                [Sdf.Path(str(geometry["mirror_parent_prim_path"]))]
            )
            right_retract_joint.CreateBody1Rel().SetTargets(
                [Sdf.Path(RIGHT_ROLLER_CARRIAGE_PRIM)]
            )
            right_retract_joint.CreateAxisAttr("Y")
            right_retract_joint.CreateLocalPos0Attr(
                Gf.Vec3f(*mirror_translation)
            )
            right_retract_joint.CreateLocalRot0Attr(
                Gf.Quatf(1.0, 0.0, 0.0, 0.0)
            )
            right_retract_joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
            right_retract_joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            right_retract_joint.CreateLowerLimitAttr(0.0)
            right_retract_joint.CreateUpperLimitAttr(0.0)
        if geometry["shape"] in ROLLER_INSERT_SHAPES:
            stage = sim_utils.get_current_stage()
            mirror_joint = UsdPhysics.RevoluteJoint.Define(
                stage, str(geometry["mirror_joint_prim_path"])
            )
            mirror_joint.CreateBody0Rel().SetTargets(
                [
                    Sdf.Path(RIGHT_ROLLER_CARRIAGE_PRIM)
                    if radial_retract
                    else Sdf.Path(str(geometry["mirror_parent_prim_path"]))
                ]
            )
            mirror_joint.CreateBody1Rel().SetTargets(
                [Sdf.Path(str(geometry["mirror_prim_path"]))]
            )
            mirror_joint.CreateAxisAttr(str(geometry["axis"]))
            mirror_joint.CreateLocalPos0Attr(
                Gf.Vec3f(*pivot_offset)
                if radial_retract
                else Gf.Vec3f(*(
                    mirror_translation[index] + pivot_offset[index]
                    for index in range(3)
                ))
            )
            mirror_joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            mirror_joint.CreateLocalPos1Attr(Gf.Vec3f(*pivot_offset))
            mirror_joint.CreateLocalRot1Attr(
                inverse_usd_quaternion(geometry["mirror_local_orientation_xyzw"])
            )
            if geometry["one_way_negative_lock"]:
                mirror_joint.CreateLowerLimitAttr(0.0)
            elif geometry["open_triggered_clutch"]:
                mirror_joint.CreateLowerLimitAttr(0.0)
                mirror_joint.CreateUpperLimitAttr(0.0)
    return geometry


def release_retract_pad_geometry() -> dict[str, object] | None:
    if not args_cli.release_retract_pad_right:
        return None
    pad_x = args_cli.release_insert_finger_x_m if args_cli.release_retract_pad_finger_x_m is None else args_cli.release_retract_pad_finger_x_m
    pad_y = 0.0260032 if args_cli.release_retract_pad_finger_y_m is None else args_cli.release_retract_pad_finger_y_m
    pad_z = args_cli.release_insert_finger_z_m if args_cli.release_retract_pad_finger_z_m is None else args_cli.release_retract_pad_finger_z_m
    pad_length = args_cli.release_insert_length_m if args_cli.release_retract_pad_length_m is None else args_cli.release_retract_pad_length_m
    translation = (
        pad_x,
        pad_y,
        pad_z,
    )
    return {
        "prim_path": "/World/XArm6/ReleaseMechanism/RightRetractPad",
        "parent_prim_path": RIGHT_FINGER_PRIM,
        "joint_prim_path": RETRACT_JOINT_PRIM,
        "axis": args_cli.release_retract_pad_axis,
        "outward_angle_deg": args_cli.release_retract_pad_outward_angle_deg,
        "local_translation_m": list(translation),
        "size_m": [
            pad_length,
            args_cli.release_retract_pad_thickness_m,
            args_cli.release_retract_pad_height_m,
        ],
        "travel_m": args_cli.release_retract_pad_travel_m,
        "delay_s": args_cli.release_retract_pad_delay_s,
        "transition_s": args_cli.release_retract_pad_transition_s,
        "return_transition_s": args_cli.release_retract_pad_return_transition_s,
        "hold_s": args_cli.release_retract_pad_hold_s,
        "radial_clear_m": args_cli.release_retract_pad_radial_clear_m,
        "radial_clear_delay_s": args_cli.release_retract_pad_radial_clear_delay_s,
        "radial_clear_transition_s": (
            args_cli.release_retract_pad_radial_clear_transition_s
        ),
        "radial_support": bool(args_cli.release_retract_pad_radial_support),
        "radial_support_prim_path": RETRACT_PAD_CARRIAGE_PRIM,
        "radial_support_local_translation_m": [
            args_cli.release_radial_support_finger_x_m,
            args_cli.release_radial_support_finger_y_m,
            args_cli.release_radial_support_finger_z_m,
        ],
        "radial_support_size_m": [
            args_cli.release_radial_support_length_m,
            args_cli.release_radial_support_thickness_m,
            args_cli.release_radial_support_height_m,
        ],
        "force_limited": bool(
            args_cli.release_retract_pad_force_limited
        ),
        "direct_effort": bool(args_cli.release_retract_pad_direct_effort),
        "direct_effort_direction": (
            args_cli.release_retract_pad_direct_effort_direction
        ),
        "one_way": bool(args_cli.release_retract_pad_one_way),
        "freewheel_after_transition": bool(
            args_cli.release_retract_pad_freewheel_after_transition
        ),
        "effort_limit_n": args_cli.release_retract_pad_effort_limit_n,
        "radial_support_static_friction": (
            args_cli.release_insert_static_friction
            if args_cli.release_radial_support_static_friction is None
            else args_cli.release_radial_support_static_friction
        ),
        "radial_support_dynamic_friction": (
            args_cli.release_insert_dynamic_friction
            if args_cli.release_radial_support_dynamic_friction is None
            else args_cli.release_radial_support_dynamic_friction
        ),
        "static_friction": (
            args_cli.release_insert_static_friction
            if args_cli.release_retract_pad_static_friction is None
            else args_cli.release_retract_pad_static_friction
        ),
        "dynamic_friction": (
            args_cli.release_insert_dynamic_friction
            if args_cli.release_retract_pad_dynamic_friction is None
            else args_cli.release_retract_pad_dynamic_friction
        ),
    }


def spawn_release_retract_pad() -> dict[str, object] | None:
    geometry = release_retract_pad_geometry()
    if geometry is None:
        return None
    pad = sim_utils.CuboidCfg(
        size=tuple(geometry["size_m"]),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=True,
            max_linear_velocity=1.0,
        ),
        mass_props=sim_utils.MassPropertiesCfg(mass=0.004),
        collision_props=sim_utils.CollisionPropertiesCfg(
            contact_offset=0.0002,
            rest_offset=0.0,
        ),
        physics_material=sim_utils.RigidBodyMaterialCfg(
            static_friction=float(geometry["static_friction"]),
            dynamic_friction=float(geometry["dynamic_friction"]),
            restitution=0.02,
            friction_combine_mode=args_cli.cube_friction_combine_mode,
            restitution_combine_mode="min",
        ),
        visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=(0.95, 0.65, 0.05),
            roughness=0.45,
        ),
    )
    spawn_translation, spawn_orientation = root_relative_pose_from_parent(
        str(geometry["parent_prim_path"]),
        geometry["local_translation_m"],
        (0.0, 0.0, 0.0, 1.0),
    )
    radial_clear = float(geometry["radial_clear_m"]) > 0.0
    radial_support = bool(geometry["radial_support"])
    if radial_clear:
        carriage_translation = spawn_translation
        carriage_orientation = spawn_orientation
        carriage_size = (0.002, 0.002, 0.002)
        if radial_support:
            carriage_translation, carriage_orientation = root_relative_pose_from_parent(
                str(geometry["parent_prim_path"]),
                geometry["radial_support_local_translation_m"],
                (0.0, 0.0, 0.0, 1.0),
            )
            carriage_size = tuple(geometry["radial_support_size_m"])
        carriage = sim_utils.CuboidCfg(
            size=carriage_size,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_linear_velocity=2.0,
            ),
            mass_props=sim_utils.MassPropertiesCfg(
                mass=0.004 if radial_support else 0.001
            ),
            collision_props=(
                sim_utils.CollisionPropertiesCfg(
                    contact_offset=0.0002,
                    rest_offset=0.0,
                )
                if radial_support else None
            ),
            physics_material=(
                sim_utils.RigidBodyMaterialCfg(
                    static_friction=float(geometry["radial_support_static_friction"]),
                    dynamic_friction=float(geometry["radial_support_dynamic_friction"]),
                    restitution=0.02,
                    friction_combine_mode=args_cli.cube_friction_combine_mode,
                    restitution_combine_mode="min",
                )
                if radial_support else None
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=(0.15, 0.15, 0.15), roughness=0.8,
            ),
        )
        carriage.func(
            RETRACT_PAD_CARRIAGE_PRIM, carriage,
            translation=carriage_translation, orientation=carriage_orientation,
        )
    pad.func(
        str(geometry["prim_path"]),
        pad,
        translation=spawn_translation,
        orientation=spawn_orientation,
    )
    stage = sim_utils.get_current_stage()
    if radial_clear:
        radial_joint = UsdPhysics.PrismaticJoint.Define(
            stage, RETRACT_PAD_RADIAL_JOINT_PRIM
        )
        radial_joint.CreateBody0Rel().SetTargets([Sdf.Path(RIGHT_FINGER_PRIM)])
        radial_joint.CreateBody1Rel().SetTargets(
            [Sdf.Path(RETRACT_PAD_CARRIAGE_PRIM)]
        )
        radial_joint.CreateAxisAttr("Y")
        radial_joint_translation = (
            geometry["radial_support_local_translation_m"]
            if radial_support else geometry["local_translation_m"]
        )
        radial_joint.CreateLocalPos0Attr(Gf.Vec3f(*radial_joint_translation))
        radial_joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        radial_joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        radial_joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        radial_joint.CreateLowerLimitAttr(0.0)
        radial_joint.CreateUpperLimitAttr(0.0)
    joint = UsdPhysics.PrismaticJoint.Define(stage, RETRACT_JOINT_PRIM)
    joint.CreateBody0Rel().SetTargets([
        Sdf.Path(
            RETRACT_PAD_CARRIAGE_PRIM
            if radial_clear and not radial_support else RIGHT_FINGER_PRIM
        )
    ])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(str(geometry["prim_path"]))])
    joint.CreateAxisAttr(str(geometry["axis"]))
    outward_angle_rad = (
        math.radians(float(geometry["outward_angle_deg"]))
        if geometry["axis"] == "X" else 0.0
    )
    joint_frame_orientation = Gf.Quatf(
        math.cos(0.5 * outward_angle_rad),
        0.0, 0.0, math.sin(0.5 * outward_angle_rad),
    )
    joint.CreateLocalPos0Attr(
        Gf.Vec3f(0.0, 0.0, 0.0)
        if radial_clear and not radial_support
        else Gf.Vec3f(*geometry["local_translation_m"])
    )
    joint.CreateLocalRot0Attr(joint_frame_orientation)
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(joint_frame_orientation)
    joint.CreateLowerLimitAttr(0.0)
    joint.CreateUpperLimitAttr(0.0)
    return geometry


def load_reference(config_path: Path):
    config = json.loads(config_path.read_text(encoding="utf-8"))
    for segment in config["reference_segments"]:
        segment["start_joint_rad"][5] += args_cli.joint6_roll_offset_rad
        segment["end_joint_rad"][5] += args_cli.joint6_roll_offset_rad
    segments = tuple(
        QuinticJointSegment(**segment)
        for segment in config["reference_segments"]
    )
    samples = generate_joint_reference(
        segments,
        float(config["control_period_s"]),
    )
    limit_evidence = evaluate_reference_samples(samples)
    if not limit_evidence["joint_mechanical_limits_pass"]:
        raise ValueError(
            "reference exceeds the verified xArm6 transfer envelope: "
            f"{limit_evidence}"
        )
    return config, samples, limit_evidence


def step_assets(
    sim: sim_utils.SimulationContext,
    robot: Articulation,
    cube: RigidObject,
    contact_sensors: dict[str, ContactSensor],
    cameras: tuple[Camera, ...],
    roller_clutch_locked: bool = True,
    roller_cam_target_rad: float = 0.0,
    retract_pad_target_m: float = 0.0,
    retract_pad_velocity_m_s: float = 0.0,
    retract_pad_radial_target_m: float = 0.0,
    retract_pad_radial_velocity_m_s: float = 0.0,
    roller_retract_target_m: float = 0.0,
) -> None:
    if (
        args_cli.release_insert_side != "none"
        and args_cli.release_insert_shape in ROLLER_INSERT_SHAPES
        and (
            args_cli.release_roller_one_way
            or args_cli.release_roller_open_triggered_clutch
        )
    ):
        roller_joint_ids, _ = robot.find_joints(
            "|".join(
                (ROLLER_JOINT_NAME, RIGHT_ROLLER_JOINT_NAME)
                if args_cli.release_insert_side == "both"
                else (ROLLER_JOINT_NAME,)
            )
        )
        if args_cli.release_roller_one_way:
            roller_velocity = robot.data.joint_vel.torch[:, roller_joint_ids]
            clutch_effort = torch.clamp(
                -0.02 * roller_velocity, min=0.0, max=0.05
            )
        else:
            if not roller_clutch_locked:
                stage = sim_utils.get_current_stage()
                joint_paths = (
                    (ROLLER_JOINT_PRIM, RIGHT_ROLLER_JOINT_PRIM)
                    if args_cli.release_insert_side == "both"
                    else (ROLLER_JOINT_PRIM,)
                )
                for joint_path in joint_paths:
                    roller_joint = UsdPhysics.RevoluteJoint.Get(stage, joint_path)
                    roller_joint.GetLowerLimitAttr().Set(-1.0e6)
                    roller_joint.GetUpperLimitAttr().Set(1.0e6)
            clutch_effort = torch.zeros(
                (1, len(roller_joint_ids)), device=robot.device
            )
        if args_cli.release_roller_cam_travel_rad > 0.0:
            roller_target = torch.full(
                (1, len(roller_joint_ids)),
                roller_cam_target_rad,
                device=robot.device,
            )
            cam_complete = (
                abs(roller_cam_target_rad)
                >= args_cli.release_roller_cam_travel_rad - 1.0e-9
            )
            if args_cli.release_roller_cam_direct_effort:
                robot.write_joint_stiffness_to_sim_index(
                    stiffness=0.0, joint_ids=roller_joint_ids
                )
                robot.write_joint_damping_to_sim_index(
                    damping=0.0, joint_ids=roller_joint_ids
                )
                pulse_active = not roller_clutch_locked and not (
                    args_cli.release_roller_cam_freewheel_after_transition
                    and cam_complete
                )
                cam_effort = torch.full_like(
                    roller_target,
                    args_cli.release_roller_cam_direction
                    * args_cli.release_roller_cam_effort_limit_nm
                    if pulse_active else 0.0,
                )
                robot.set_joint_effort_target_index(
                    target=cam_effort, joint_ids=roller_joint_ids
                )
            else:
                robot.set_joint_position_target_index(
                    target=roller_target, joint_ids=roller_joint_ids
                )
            if (
                not args_cli.release_roller_cam_direct_effort
                and
                args_cli.release_roller_radial_retract_kinematic
                and not (
                    args_cli.release_roller_cam_freewheel_after_transition
                    and cam_complete
                )
            ):
                robot.write_joint_position_to_sim_index(
                    position=roller_target,
                    joint_ids=roller_joint_ids,
                )
                imposed_velocity_rad_s = (
                    args_cli.release_roller_cam_direction
                    * args_cli.release_roller_cam_travel_rad
                    / args_cli.release_roller_cam_transition_s
                    if 0.0 < abs(roller_cam_target_rad)
                    < args_cli.release_roller_cam_travel_rad
                    else 0.0
                )
                robot.write_joint_velocity_to_sim_index(
                    velocity=torch.full_like(
                        roller_target, imposed_velocity_rad_s
                    ),
                    joint_ids=roller_joint_ids,
                )
            if (
                args_cli.release_roller_cam_freewheel_after_transition
                and cam_complete
            ):
                robot.write_joint_stiffness_to_sim_index(
                    stiffness=0.0, joint_ids=roller_joint_ids
                )
                robot.write_joint_damping_to_sim_index(
                    damping=0.0, joint_ids=roller_joint_ids
                )
                robot.set_joint_effort_target_index(
                    target=clutch_effort, joint_ids=roller_joint_ids
                )
        else:
            robot.set_joint_effort_target_index(
                target=clutch_effort, joint_ids=roller_joint_ids
            )
    if args_cli.release_retract_pad_right:
        retract_joint_ids, _ = robot.find_joints(RETRACT_JOINT_NAME)
        if not roller_clutch_locked:
            stage = sim_utils.get_current_stage()
            retract_joint = UsdPhysics.PrismaticJoint.Get(
                stage, RETRACT_JOINT_PRIM
            )
            retract_joint.GetLowerLimitAttr().Set(
                -args_cli.release_retract_pad_travel_m
            )
            retract_joint.GetUpperLimitAttr().Set(
                args_cli.release_retract_pad_travel_m
                if args_cli.release_retract_pad_force_limited
                else 0.0
            )
        if args_cli.release_retract_pad_one_way and not roller_clutch_locked:
            current_retract_q = float(
                robot.data.joint_pos.torch[0, retract_joint_ids[0]].item()
            )
            retract_joint.GetUpperLimitAttr().Set(
                min(0.0, current_retract_q)
            )
        retract_target = torch.full(
            (1, len(retract_joint_ids)),
            (
                -retract_pad_target_m
                if args_cli.release_retract_pad_force_limited
                else retract_pad_target_m
            ),
            device=robot.device,
        )
        if args_cli.release_retract_pad_direct_effort:
            robot.write_joint_stiffness_to_sim_index(
                stiffness=0.0, joint_ids=retract_joint_ids
            )
            robot.write_joint_damping_to_sim_index(
                damping=0.0, joint_ids=retract_joint_ids
            )
            rebound_brake_active = (
                args_cli.release_retract_pad_one_way
                and float(
                    robot.data.joint_vel.torch[0, retract_joint_ids[0]].item()
                ) > 0.0
            )
            direct_force = torch.full_like(
                retract_target,
                -args_cli.release_retract_pad_effort_limit_n
                if rebound_brake_active else (
                    args_cli.release_retract_pad_direct_effort_direction
                    * args_cli.release_retract_pad_effort_limit_n
                    if retract_pad_velocity_m_s < -1.0e-9 else 0.0
                ),
            )
            robot.set_joint_effort_target_index(
                target=direct_force, joint_ids=retract_joint_ids
            )
        else:
            robot.set_joint_position_target_index(
                target=retract_target,
                joint_ids=retract_joint_ids,
            )
        strike_complete = (
            abs(retract_pad_target_m)
            >= args_cli.release_retract_pad_travel_m - 1.0e-9
            and abs(retract_pad_velocity_m_s) <= 1.0e-9
        )
        if (
            args_cli.release_retract_pad_force_limited
            and args_cli.release_retract_pad_freewheel_after_transition
            and strike_complete
        ):
            retract_joint.GetUpperLimitAttr().Set(
                args_cli.release_retract_pad_travel_m
            )
            robot.write_joint_stiffness_to_sim_index(
                stiffness=0.0, joint_ids=retract_joint_ids
            )
            robot.write_joint_damping_to_sim_index(
                damping=0.0, joint_ids=retract_joint_ids
            )
            if not args_cli.release_retract_pad_one_way:
                robot.set_joint_effort_target_index(
                    target=torch.zeros_like(retract_target),
                    joint_ids=retract_joint_ids,
                )
        elif (
            args_cli.release_roller_radial_retract_kinematic
            and not args_cli.release_retract_pad_force_limited
        ):
            robot.write_joint_position_to_sim_index(
                position=retract_target,
                joint_ids=retract_joint_ids,
            )
            robot.write_joint_velocity_to_sim_index(
                velocity=torch.full_like(
                    retract_target, retract_pad_velocity_m_s
                ),
                joint_ids=retract_joint_ids,
            )
    if args_cli.release_retract_pad_radial_clear_m > 0.0:
        radial_joint_ids, _ = robot.find_joints(RETRACT_PAD_RADIAL_JOINT_NAME)
        if retract_pad_radial_target_m < 0.0:
            stage = sim_utils.get_current_stage()
            radial_joint = UsdPhysics.PrismaticJoint.Get(
                stage, RETRACT_PAD_RADIAL_JOINT_PRIM
            )
            radial_joint.GetLowerLimitAttr().Set(
                -args_cli.release_retract_pad_radial_clear_m
            )
            radial_joint.GetUpperLimitAttr().Set(0.0)
        radial_target = torch.full(
            (1, len(radial_joint_ids)),
            retract_pad_radial_target_m,
            device=robot.device,
        )
        robot.set_joint_position_target_index(
            target=radial_target,
            joint_ids=radial_joint_ids,
        )
        if args_cli.release_roller_radial_retract_kinematic:
            robot.write_joint_position_to_sim_index(
                position=radial_target,
                joint_ids=radial_joint_ids,
            )
            robot.write_joint_velocity_to_sim_index(
                velocity=torch.full_like(
                    radial_target, retract_pad_radial_velocity_m_s
                ),
                joint_ids=radial_joint_ids,
            )
    if args_cli.release_roller_radial_retract_m > 0.0:
        roller_retract_names = (
            (ROLLER_RETRACT_JOINT_NAME, RIGHT_ROLLER_RETRACT_JOINT_NAME)
            if args_cli.release_insert_side == "both"
            else (ROLLER_RETRACT_JOINT_NAME,)
        )
        roller_retract_ids, matched_retract_names = robot.find_joints(
            "|".join(roller_retract_names)
        )
        if roller_retract_target_m > 0.0:
            stage = sim_utils.get_current_stage()
            roller_retract_joint = UsdPhysics.PrismaticJoint.Get(
                stage, ROLLER_RETRACT_JOINT_PRIM
            )
            roller_retract_joint.GetLowerLimitAttr().Set(0.0)
            roller_retract_joint.GetUpperLimitAttr().Set(
                args_cli.release_roller_radial_retract_m
            )
            if args_cli.release_insert_side == "both":
                right_retract_joint = UsdPhysics.PrismaticJoint.Get(
                    stage, RIGHT_ROLLER_RETRACT_JOINT_PRIM
                )
                right_retract_joint.GetLowerLimitAttr().Set(
                    -args_cli.release_roller_radial_retract_m
                )
                right_retract_joint.GetUpperLimitAttr().Set(0.0)
        retract_signs = [
            -1.0 if name == RIGHT_ROLLER_RETRACT_JOINT_NAME else 1.0
            for name in matched_retract_names
        ]
        roller_retract_target = torch.tensor(
            [[sign * roller_retract_target_m for sign in retract_signs]],
            dtype=torch.float32,
            device=robot.device,
        )
        robot.set_joint_position_target_index(
            target=roller_retract_target,
            joint_ids=roller_retract_ids,
        )
        if args_cli.release_roller_radial_retract_kinematic:
            robot.write_joint_position_to_sim_index(
                position=roller_retract_target,
                joint_ids=roller_retract_ids,
            )
            imposed_velocity_m_s = (
                args_cli.release_roller_radial_retract_m
                / args_cli.release_roller_radial_retract_transition_s
                if (
                    0.0 < roller_retract_target_m < args_cli.release_roller_radial_retract_m
                )
                else 0.0
            )
            roller_retract_velocity = torch.tensor(
                [[sign * imposed_velocity_m_s for sign in retract_signs]],
                dtype=torch.float32,
                device=robot.device,
            )
            robot.write_joint_velocity_to_sim_index(
                velocity=roller_retract_velocity,
                joint_ids=roller_retract_ids,
            )
    robot.write_data_to_sim()
    cube.write_data_to_sim()
    sim.step(render=bool(cameras))
    dt = sim.get_physics_dt()
    robot.update(dt)
    cube.update(dt)
    for sensor in contact_sensors.values():
        sensor.update(dt, force_recompute=True)
    for camera in cameras:
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
    filtered_forces = sensor.data.force_matrix_w
    if filtered_forces is None:
        return 0.0
    return float(
        torch.linalg.vector_norm(filtered_forces.torch).item()
    )


def xyzw_quaternion_to_matrix(quaternion: torch.Tensor) -> torch.Tensor:
    x, y, z, w = quaternion.unbind()
    return torch.stack(
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        )
    ).reshape(3, 3)


def estimate_cube_position_from_camera(
    camera: Camera,
    cube_size_m: float,
    expected_position_base_m: torch.Tensor,
    source_camera: str,
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
    intrinsic = camera.data.intrinsic_matrices.torch[0]
    rotation = xyzw_quaternion_to_matrix(camera.data.quat_w_ros.torch[0])
    translation = camera.data.pos_w.torch[0]
    expected_camera = rotation.T @ (
        expected_position_base_m - translation
    )
    metadata = {"source_camera": source_camera}
    if float(expected_camera[2].item()) <= 0.0:
        return None, {
            **metadata,
            "detected": 0.0,
            "failure": "expected_point_behind_camera",
            "expected_position_camera_m": [
                float(value) for value in expected_camera.tolist()
            ],
            "camera_position_world_m": [
                float(value) for value in translation.tolist()
            ],
            "camera_quaternion_ros_xyzw": [
                float(value)
                for value in camera.data.quat_w_ros.torch[0].tolist()
            ],
        }
    expected_u = (
        intrinsic[0, 0] * expected_camera[0] / expected_camera[2]
        + intrinsic[0, 2]
    )
    expected_v = (
        intrinsic[1, 1] * expected_camera[1] / expected_camera[2]
        + intrinsic[1, 2]
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
            **metadata,
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
            (u - intrinsic[0, 2]) / intrinsic[0, 0] * z_center,
            (v - intrinsic[1, 2]) / intrinsic[1, 1] * z_center,
            z_center,
        )
    )
    position_base = rotation @ camera_point + translation
    return position_base, {
        **metadata,
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


def wrist_camera_world_pose(
    hand_position: torch.Tensor,
    hand_quaternion_xyzw: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor]:
    offset_position = torch.tensor(
        WRIST_CAMERA_POSITION_EEF_M,
        dtype=torch.float32,
        device=hand_position.device,
    )
    camera_position = hand_position + quat_apply(
        hand_quaternion_xyzw.unsqueeze(0), offset_position.unsqueeze(0)
    )[0]
    offset_xyzw = torch.tensor(
        WRIST_CAMERA_QUAT_XYZW,
        dtype=torch.float32,
        device=hand_position.device,
    )
    camera_xyzw = quat_mul(hand_quaternion_xyzw, offset_xyzw)
    return camera_position, camera_xyzw


def wrist_camera_cube_clearance_m(
    hand_position: torch.Tensor,
    hand_quaternion_xyzw: torch.Tensor,
    cube_position: torch.Tensor,
    cube_size_m: float,
) -> float:
    camera_position, camera_xyzw = wrist_camera_world_pose(
        hand_position, hand_quaternion_xyzw
    )
    cube_camera = quat_apply_inverse(
        camera_xyzw.unsqueeze(0),
        (cube_position - camera_position).unsqueeze(0),
    )[0]
    half_extents = WRIST_CAMERA_PROXY_HALF_EXTENTS_M.to(cube_position.device)
    outside = torch.clamp(torch.abs(cube_camera) - half_extents, min=0.0)
    center_to_box = torch.linalg.vector_norm(outside)
    cube_bounding_radius = 0.5 * float(cube_size_m) * np.sqrt(3.0)
    return float(center_to_box.item() - cube_bounding_radius)


def update_wrist_camera_pose(
    robot: Articulation,
    gripper_body_id: int,
    wrist_camera: Camera | None,
) -> None:
    if wrist_camera is None:
        return
    gripper_pose = robot.data.body_pose_w.torch[0, gripper_body_id]
    hand_position = gripper_pose[:3]
    hand_quaternion_xyzw = gripper_pose[3:7]
    camera_position, camera_xyzw = wrist_camera_world_pose(
        hand_position, hand_quaternion_xyzw
    )
    wrist_camera.set_world_poses(
        camera_position.unsqueeze(0),
        camera_xyzw.unsqueeze(0),
        convention="ros",
    )


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
    half_angle_rad = 0.5 * np.deg2rad(args_cli.cube_rotation_hand_y_deg)
    local_rotation_xyzw = torch.tensor(
        [
            0.0,
            np.sin(half_angle_rad),
            0.0,
            np.cos(half_angle_rad),
        ],
        dtype=torch.float32,
        device=robot.device,
    )
    pose = cube.data.default_root_pose.torch.clone()
    pose[0, :3] = finger_midpoint + world_offset
    pose[0, 3:7] = quat_mul(hand_quaternion, local_rotation_xyzw)
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
    roller_joint_id: int | None,
    retract_joint_id: int | None,
    roller_retract_joint_id: int | None,
    release_body_ids: dict[str, int],
    gripper_body_id: int,
    finger_body_ids: list[int],
    contact_sensors: dict[str, ContactSensor],
) -> dict[str, object]:
    hand_position, hand_quaternion, finger_midpoint = hand_state(
        robot,
        gripper_body_id,
        finger_body_ids,
    )
    cube_pose = cube.data.root_link_pose_w.torch[0]
    cube_quaternion_wxyz = convert_quat(
        cube_pose[3:7], to="wxyz"
    )
    hand_quaternion_wxyz = convert_quat(
        hand_quaternion, to="wxyz"
    )
    cube_linear = cube.data.root_com_lin_vel_w.torch[0]
    cube_angular = cube.data.root_com_ang_vel_w.torch[0]
    hand_linear = robot.data.body_link_lin_vel_w.torch[0, gripper_body_id]
    hand_angular = robot.data.body_link_ang_vel_w.torch[0, gripper_body_id]
    hand_jacobian = robot.data.body_link_jacobian_w.torch[
        0, gripper_body_id - 1, :, arm_ids
    ]
    relative_hand = quat_apply_inverse(
        hand_quaternion.unsqueeze(0),
        (cube_pose[:3] - hand_position).unsqueeze(0),
    )[0]
    contact_forces = {
        name: cube_contact_force_n(sensor)
        for name, sensor in contact_sensors.items()
    }
    # link_eef is an empty fixed frame in the received URDF. Its adjacent
    # collision bodies are link6 and the gripper base, so their union is exact.
    contact_forces["link_eef"] = max(
        contact_forces["link6"], contact_forces["gripper_base"]
    )
    if args_cli.wrist_camera_hardware_removed:
        wrist_clearance_m = None
        contact_forces["wrist_camera_proxy"] = 0.0
    else:
        wrist_clearance_m = wrist_camera_cube_clearance_m(
            hand_position, hand_quaternion, cube_pose[:3], args_cli.cube_size_m
        )
        contact_forces["wrist_camera_proxy"] = (
            1.0 if wrist_clearance_m <= 0.0 else 0.0
        )
    # The ground cuboid does not expose a rigid-body prim that accepts an
    # Isaac ContactSensor. End ballistic flight when the cube bottom reaches it.
    ground_clearance_m = cube_ground_clearance_m(
        cube_pose[:3].tolist(),
        cube_quaternion_wxyz.tolist(),
        args_cli.cube_size_m,
    )
    contact_forces["ground"] = 1.0 if ground_clearance_m <= 0.001 else 0.0
    return {
        "time_s": time_s,
        "phase": phase,
        "cube_position_w_m": [
            float(value) for value in cube_pose[:3].tolist()
        ],
        "cube_quaternion_wxyz": [
            float(value) for value in cube_quaternion_wxyz.tolist()
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
        "hand_quaternion_wxyz": [
            float(value) for value in hand_quaternion_wxyz.tolist()
        ],
        "hand_linear_velocity_w_m_s": [
            float(value) for value in hand_linear.tolist()
        ],
        "hand_angular_velocity_w_rad_s": [
            float(value) for value in hand_angular.tolist()
        ],
        "hand_linear_jacobian_w": [
            [float(value) for value in row]
            for row in hand_jacobian[:3].tolist()
        ],
        "hand_angular_jacobian_w": [
            [float(value) for value in row]
            for row in hand_jacobian[3:].tolist()
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
        "arm_joint_effort_nm": [
            float(value)
            for value in robot.data.applied_torque.torch[0, arm_ids].tolist()
        ],
        "gripper_effort_nm": float(
            robot.data.applied_torque.torch[0, drive_id].item()
        ),
        "gripper_drive_rad": float(
            robot.data.joint_pos.torch[0, drive_id].item()
        ),
        "release_roller_joint_position_rad": (
            None
            if roller_joint_id is None
            else float(robot.data.joint_pos.torch[0, roller_joint_id].item())
        ),
        "release_roller_joint_velocity_rad_s": (
            None
            if roller_joint_id is None
            else float(robot.data.joint_vel.torch[0, roller_joint_id].item())
        ),
        "release_roller_joint_effort_nm": (
            None
            if roller_joint_id is None
            else float(robot.data.applied_torque.torch[0, roller_joint_id].item())
        ),
        "release_retract_joint_position_m": (
            None
            if retract_joint_id is None
            else float(robot.data.joint_pos.torch[0, retract_joint_id].item())
        ),
        "release_retract_joint_velocity_m_s": (
            None
            if retract_joint_id is None
            else float(robot.data.joint_vel.torch[0, retract_joint_id].item())
        ),
        "release_retract_joint_effort_n": (
            None
            if retract_joint_id is None
            else float(
                robot.data.applied_torque.torch[0, retract_joint_id].item()
            )
        ),
        "release_roller_retract_joint_position_m": (
            None
            if roller_retract_joint_id is None
            else float(
                robot.data.joint_pos.torch[0, roller_retract_joint_id].item()
            )
        ),
        "release_roller_retract_joint_velocity_m_s": (
            None
            if roller_retract_joint_id is None
            else float(
                robot.data.joint_vel.torch[0, roller_retract_joint_id].item()
            )
        ),
        "release_roller_retract_joint_effort_n": (
            None if roller_retract_joint_id is None else float(
                robot.data.applied_torque.torch[0, roller_retract_joint_id].item()
            )
        ),
        "release_mechanism_body_positions_w_m": {
            name: [float(value) for value in robot.data.body_link_pos_w.torch[0, body_id].tolist()]
            for name, body_id in release_body_ids.items()
        },
        "release_mechanism_body_quaternions_wxyz": {
            name: [
                float(value) for value in convert_quat(
                    robot.data.body_link_quat_w.torch[0, body_id], to="wxyz"
                ).tolist()
            ]
            for name, body_id in release_body_ids.items()
        },
        "robot_cube_contact_forces_n": contact_forces,
        "wrist_camera_proxy_clearance_m": wrist_clearance_m,
        "ground_clearance_m": ground_clearance_m,
        "left_finger_cube_contact_force_n": contact_forces["left_finger"],
        "right_finger_cube_contact_force_n": contact_forces["right_finger"],
    }


def probe_signal_record(
    robot: Articulation,
    arm_ids: list[int],
    drive_id: int,
) -> dict[str, object]:
    applied_torque = robot.data.applied_torque.torch[0]
    return {
        "arm_effort_nm": [
            float(value) for value in applied_torque[arm_ids].tolist()
        ],
        "gripper_effort_nm": float(applied_torque[drive_id].item()),
        "joint_velocity_rad_s": [
            float(value)
            for value in robot.data.joint_vel.torch[0, arm_ids].tolist()
        ],
        "gripper_position": float(
            robot.data.joint_pos.torch[0, drive_id].item()
        ),
    }


def all_robot_contact_free(record: dict[str, object]) -> bool:
    contact_forces = record.get("robot_cube_contact_forces_n")
    if isinstance(contact_forces, dict):
        return all(
            float(force) <= ROBOT_CONTACT_FORCE_THRESHOLD_N
            for force in contact_forces.values()
        )
    return bool(
        float(record["left_finger_cube_contact_force_n"]) <= ROBOT_CONTACT_FORCE_THRESHOLD_N
        and float(record["right_finger_cube_contact_force_n"]) <= ROBOT_CONTACT_FORCE_THRESHOLD_N
    )


def relative_orientation_wxyz(record: dict[str, object]) -> np.ndarray:
    hand = np.asarray(record["hand_quaternion_wxyz"], dtype=float)
    cube = np.asarray(record["cube_quaternion_wxyz"], dtype=float)
    hand = hand / np.linalg.norm(hand)
    cube = cube / np.linalg.norm(cube)
    hand_w, hand_xyz = hand[0], hand[1:]
    cube_w, cube_xyz = cube[0], cube[1:]
    return np.concatenate(
        (
            [hand_w * cube_w + float(np.dot(hand_xyz, cube_xyz))],
            hand_w * cube_xyz - cube_w * hand_xyz - np.cross(hand_xyz, cube_xyz),
        )
    )


def quaternion_distance_deg(first: np.ndarray, second: np.ndarray) -> float:
    first = first / np.linalg.norm(first)
    second = second / np.linalg.norm(second)
    half_angle_cosine = float(np.clip(abs(np.dot(first, second)), 0.0, 1.0))
    return float(np.rad2deg(2.0 * np.arccos(half_angle_cosine)))


def summarize(
    records: list[dict[str, object]],
    placed_pose: list[float],
    reference_limit_evidence: dict[str, object],
) -> dict[str, object]:
    settle_records = [
        record for record in records
        if record["phase"] == "settle"
    ]
    if not settle_records:
        raise RuntimeError("held Probe/settle records are required")
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
    baseline_orientation = relative_orientation_wxyz(settle_records[-1])
    prethrow_orientation_errors_deg = [
        quaternion_distance_deg(
            relative_orientation_wxyz(record),
            baseline_orientation,
        )
        for record in throw_records
    ]
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
    contact_free_start_s = None
    for record in postrelease_records:
        contact_free = all_robot_contact_free(record)
        if contact_free and contact_free_start_s is None:
            contact_free_start_s = float(record["time_s"])
        elif not contact_free:
            contact_free_start_s = None
        if (
            contact_free_start_s is not None
            and float(record["time_s"]) - contact_free_start_s >= 0.005
        ):
            detach_time_s = contact_free_start_s
            break

    release_record = throw_records[-1]
    release_height = float(release_record["cube_position_w_m"][2])
    release_hand_position = np.asarray(
        release_record["hand_position_w_m"], dtype=float
    )
    w, x, y, z = np.asarray(
        release_record["hand_quaternion_wxyz"], dtype=float
    )
    release_tool_axis = np.asarray(
        [2.0 * (x * z + w * y), 2.0 * (y * z - w * x), 1.0 - 2.0 * (x * x + y * y)]
    )
    release_tcp_radius_m = float(np.linalg.norm(release_hand_position[:2]))
    release_outward_dot_m = float(
        np.dot(release_tool_axis[:2], release_hand_position[:2])
    )
    maximum_height = max(
        float(record["cube_position_w_m"][2])
        for record in postrelease_records
    )
    final_record = records[-1]
    maximum_separation_m = max(
        float(
            np.linalg.norm(
                np.asarray(record["cube_position_hand_m"], dtype=float)
                - baseline
            )
        )
        for record in postrelease_records
    )
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
    motion_records: list[dict[str, object]] = []
    for record in records:
        if (
            motion_records
            and float(record["time_s"]) == float(motion_records[-1]["time_s"])
        ):
            motion_records[-1] = record
        else:
            motion_records.append(record)
    actual_joint_positions = np.asarray(
        [record["arm_joint_position_rad"] for record in motion_records], dtype=float
    )
    actual_joint_velocities = np.asarray(
        [record["arm_joint_velocity_rad_s"] for record in motion_records],
        dtype=float,
    )
    actual_joint_accelerations = np.zeros_like(actual_joint_velocities)
    if len(motion_records) >= 2:
        times = np.asarray(
            [record["time_s"] for record in motion_records], dtype=float
        )
        actual_joint_accelerations[1:] = np.diff(actual_joint_velocities, axis=0) / np.diff(
            times
        )[:, None]
    actual_joint_efforts = np.asarray(
        [record["arm_joint_effort_nm"] for record in motion_records], dtype=float
    )
    calibrated_joint_efforts = actual_joint_efforts / args_cli.arm_sim_effort_scale
    actual_limit_evidence = evaluate_joint_trajectory(
        actual_joint_positions,
        actual_joint_velocities,
        actual_joint_accelerations,
        efforts_nm=calibrated_joint_efforts,
    )
    sim_actual_state_limits_pass = bool(
        actual_limit_evidence["joint_hard_bounds_pass"]
        and actual_limit_evidence["joint_speed_pass"]
        and actual_limit_evidence["effort_pass"]
    )
    raw_sim_peak_effort_nm = np.max(np.abs(actual_joint_efforts), axis=0).tolist()
    legacy_detach_time_s = detach_time_s
    free_flight_evidence = continuous_free_flight_evidence(
        records,
        baseline,
        release_height_m=release_height,
    )
    detach_time_s = free_flight_evidence.get(
        "continuous_free_flight_start_time_s"
    )
    release_transfer = release_transfer_evidence(
        records,
        release_motion_start_time_s=(
            args_cli.gripper_open_command_time_s
            + args_cli.release_drive_start_delay_s
        ),
        detach_time_s=detach_time_s,
        target_axis_world=free_flight_evidence.get("tumble_axis_world"),
    )
    return {
        "schema": "xarm6_native_release_smoke_v1",
        "trajectory_quaternion_order": "wxyz",
        "cube_state_writes_after_initialization": 0,
        "placed_cube_pose_w": placed_pose,
        "prethrow_max_relative_error_m": max(prethrow_errors),
        "prethrow_max_relative_orientation_error_deg": max(
            prethrow_orientation_errors_deg
        ),
        "prethrow_stable": max(prethrow_errors) <= 0.008
        and max(prethrow_orientation_errors_deg) <= 8.0,
        "detach_detected": detach_time_s is not None,
        "detach_time_s": detach_time_s,
        "legacy_short_contact_detach_time_s": legacy_detach_time_s,
        **free_flight_evidence,
        "release_transfer_evidence": release_transfer,
        "release_command_time_s": args_cli.gripper_open_command_time_s,
        "kinematic_release_time_s": args_cli.release_time_s,
        "detach_delay_s": (
            None
            if detach_time_s is None
            else detach_time_s - args_cli.gripper_open_command_time_s
        ),
        "release_drive_transition_s": args_cli.release_drive_transition_s,
        "release_dynamics_after_transition": bool(
            args_cli.release_dynamics_after_transition
        ),
        "release_dynamics_before_transition": bool(
            args_cli.release_dynamics_before_transition
        ),
        "release_drive_start_delay_s": args_cli.release_drive_start_delay_s,
        "release_height_m": release_height,
        "release_hand_position_m": release_hand_position.tolist(),
        "release_tool_axis_world": release_tool_axis.tolist(),
        "release_tcp_horizontal_radius_m": release_tcp_radius_m,
        "release_outward_dot_m": release_outward_dot_m,
        "maximum_height_m": maximum_height,
        "free_vertical_displacement_m": maximum_height - release_height,
        "maximum_separation_m": maximum_separation_m,
        "catch_servo_enabled": args_cli.catch_servo_start_time_s is not None,
        "catch_servo_start_time_s": args_cli.catch_servo_start_time_s,
        "catch_close_time_s": args_cli.catch_close_time_s,
        "catch_preclose_time_s": args_cli.catch_preclose_time_s,
        "catch_preclose_drive_rad": args_cli.catch_preclose_drive_rad,
        "catch_prediction_horizon_s": args_cli.catch_prediction_horizon_s,
        "catch_intercept_time_s": args_cli.catch_intercept_time_s,
        "catch_max_relative_error_m": catch_max_relative_error,
        "catch_max_relative_motion_m": catch_max_relative_motion,
        "catch_stable": catch_stable,
        "reference_joint_limit_evidence": reference_limit_evidence,
        "actual_joint_limit_evidence": actual_limit_evidence,
        "sim_effort_calibration_scale": args_cli.arm_sim_effort_scale,
        "raw_sim_per_joint_peak_effort_nm": raw_sim_peak_effort_nm,
        "sim_actual_state_limits_pass": sim_actual_state_limits_pass,
        "sim_actual_acceleration_matches_transfer_envelope": bool(
            actual_limit_evidence["joint_acceleration_pass"]
            and actual_limit_evidence["qdot_change_pass"]
        ),
        "joint_mechanical_limits_pass": bool(
            reference_limit_evidence["joint_mechanical_limits_pass"]
            and sim_actual_state_limits_pass
        ),
        "obvious_toss_success": bool(
            free_flight_evidence.get("obvious_free_flight", False)
            and catch_stable
        ),
        "visible_spin_toss_success": bool(
            free_flight_evidence.get("obvious_free_flight", False)
            and free_flight_evidence.get("visible_spin", False) and catch_stable
        ),
        "tumble_toss_success": bool(
            free_flight_evidence.get("obvious_free_flight", False)
            and free_flight_evidence.get("target_axis_tumble", False)
            and catch_stable
            and reference_limit_evidence["joint_mechanical_limits_pass"]
            and sim_actual_state_limits_pass
        ),
        "catch_evidence_window_s": args_cli.catch_evidence_window_s,
        "bilateral_contact_fraction": bilateral_contact_fraction,
        "final_cube_position_w_m": final_record["cube_position_w_m"],
        "final_cube_linear_velocity_w_m_s": (
            final_record["cube_linear_velocity_w_m_s"]
        ),
        "cube_size_m": args_cli.cube_size_m,
        "cube_mass_kg": args_cli.cube_mass_kg,
        "wrist_camera_hardware_removed": bool(
            args_cli.wrist_camera_hardware_removed
        ),
        "wrist_camera_recording_virtual_only": bool(args_cli.wrist_camera_hardware_removed),
        "held_drive_rad": args_cli.held_drive_rad,
        "held_gripper_effort_limit_n": args_cli.held_gripper_effort_limit_n,
        "gripper_preopen_command_time_s": args_cli.gripper_preopen_command_time_s,
        "gripper_preopen_drive_rad": args_cli.gripper_preopen_drive_rad,
        "gripper_preopen_transition_s": args_cli.gripper_preopen_transition_s,
        "partial_open_drive_rad": args_cli.partial_open_drive_rad,
        "release_gripper_effort_limit_n": (
            args_cli.release_gripper_effort_limit_n
        ),
        "release_gripper_stiffness": args_cli.release_gripper_stiffness,
        "catch_drive_rad": (
            args_cli.held_drive_rad
            if args_cli.catch_drive_rad is None else args_cli.catch_drive_rad
        ),
        "catch_gripper_effort_limit_n": (
            args_cli.catch_gripper_effort_limit_n
        ),
        "catch_gripper_stiffness": args_cli.catch_gripper_stiffness,
        "catch_lock_wrist": bool(args_cli.catch_lock_wrist),
        "catch_allow_j5": bool(args_cli.catch_allow_j5),
        "catch_j5_weight": float(args_cli.catch_j5_weight),
        "catch_joint_target_rad": args_cli.catch_joint_target_rad,
        "catch_joint_pretarget_rad": args_cli.catch_joint_pretarget_rad,
        "catch_lateral_only": bool(args_cli.catch_lateral_only),
        "catch_hold_throw_joints": bool(
            args_cli.catch_hold_throw_joints
        ),
        "catch_preserve_nominal_momentum": bool(
            args_cli.catch_preserve_nominal_momentum
        ),
        "catch_joint1_start_time_s": args_cli.catch_joint1_start_time_s,
        "actual_max_joint_speed_rad_s": actual_limit_evidence["max_joint_speed_rad_s"],
        "actual_max_joint_acceleration_rad_s2": actual_limit_evidence["max_joint_acceleration_rad_s2"],
        "cube_offset_hand_m": list(args_cli.cube_offset_hand_m),
        "cube_rotation_hand_y_deg": args_cli.cube_rotation_hand_y_deg,
        "release_insert_geometry": release_insert_geometry(),
        "release_retract_pad_geometry": release_retract_pad_geometry(),
    }


def main() -> int:
    if not args_cli.usd.is_file():
        raise FileNotFoundError(args_cli.usd)
    if args_cli.release_insert_side != "none":
        if (
            args_cli.release_insert_side == "both"
            and args_cli.release_insert_shape not in ROLLER_INSERT_SHAPES
        ):
            raise ValueError("dual inserts are supported for roller shapes")
        max_release_insert_radius_m = (
            0.015
            if args_cli.release_insert_shape in ("spinner_y", "flipper_y")
            else 0.006
        )
        if not 0.0005 <= args_cli.release_insert_radius_m <= max_release_insert_radius_m:
            raise ValueError(
                f"release insert radius must lie in [0.5, {1e3 * max_release_insert_radius_m:g}] mm"
            )
        if (
            args_cli.release_insert_shape == "d_roller_x"
            and not 0.0 <= args_cli.release_d_roller_chord_m < args_cli.release_insert_radius_m
        ):
            raise ValueError("D-roller chord must satisfy 0 <= chord < radius")
        if not 0.006 <= args_cli.release_insert_length_m <= 0.032:
            raise ValueError("release insert length must lie in [6, 32] mm")
        if not -0.016 <= args_cli.release_insert_finger_x_m <= 0.016:
            raise ValueError("release insert finger-local X must lie in [-16, 16] mm")
        if args_cli.release_insert_shape == "ramp_x":
            if not 0.005 <= args_cli.release_insert_ramp_height_m <= 0.035:
                raise ValueError("release ramp height must lie in [5, 35] mm")
            if not 1.0 <= abs(args_cli.release_insert_ramp_deg) <= 30.0:
                raise ValueError(
                    "release ramp angle magnitude must lie in [1, 30] degrees"
                )
        if not 0.006 <= args_cli.release_insert_finger_z_m <= 0.060:
            raise ValueError("release insert finger-local Z must lie in [6, 60] mm")
        if not (
            0.0
            <= args_cli.release_insert_dynamic_friction
            <= args_cli.release_insert_static_friction
        ):
            raise ValueError(
                "release insert friction must satisfy 0 <= dynamic <= static"
            )
        if args_cli.release_insert_static_friction > 2.0:
            raise ValueError("release insert static friction must not exceed 2.0")
        if args_cli.release_roller_cam_travel_rad > 0.0:
            if args_cli.release_insert_shape not in (
                "d_roller_x", "spinner_y", "flipper_y"
            ):
                raise ValueError(
                    "G1 cam coupling requires a D roller, spin disk, or flipper"
                )
            if not args_cli.release_roller_open_triggered_clutch:
                raise ValueError("G1 cam coupling requires the open-triggered clutch")
            if args_cli.release_roller_cam_lead_s < 0.0:
                raise ValueError("G1 cam lead must be nonnegative")
            if args_cli.release_roller_cam_delay_s < 0.0:
                raise ValueError("G1 cam delay must be nonnegative")
            if args_cli.release_roller_cam_transition_s <= 0.0:
                raise ValueError("G1 cam transition must be positive")
            if args_cli.release_roller_cam_effort_limit_nm <= 0.0:
                raise ValueError("G1 cam effort limit must be positive")
        if args_cli.release_retract_pad_right:
            if (
                args_cli.release_insert_side != "left"
                or args_cli.release_insert_shape not in (
                    "d_roller_x", "spinner_y", "flipper_y"
                )
            ):
                raise ValueError("right retract pad requires one left release wheel")
            if not args_cli.release_roller_open_triggered_clutch:
                raise ValueError("right retract pad requires the open-triggered clutch")
            if args_cli.release_retract_pad_travel_m <= 0.0:
                raise ValueError("right retract pad travel must be positive")
            if args_cli.release_retract_pad_delay_s < 0.0:
                raise ValueError("right retract pad delay must be nonnegative")
            if args_cli.release_retract_pad_transition_s <= 0.0:
                raise ValueError("right retract pad transition must be positive")
            if args_cli.release_retract_pad_effort_limit_n <= 0.0:
                raise ValueError("right retract pad effort limit must be positive")
            if (
                args_cli.release_retract_pad_radial_support
                and args_cli.release_retract_pad_radial_clear_m <= 0.0
            ):
                raise ValueError(
                    "separate radial support requires positive radial clear travel"
                )
        if args_cli.release_roller_radial_retract_m > 0.0:
            if (
                args_cli.release_insert_side not in ("left", "both")
                or args_cli.release_insert_shape not in (
                    "d_roller_x", "spinner_y", "flipper_y"
                )
            ):
                raise ValueError("radial retract requires one or two release wheels")
            if args_cli.release_roller_radial_retract_delay_s < 0.0:
                raise ValueError("roller retract delay must be nonnegative")
            if args_cli.release_roller_radial_retract_transition_s <= 0.0:
                raise ValueError("roller retract transition must be positive")
    probe_j_config = (
        None if args_cli.probe_j_config is None
        else json.loads(args_cli.probe_j_config.read_text(encoding="utf-8"))
    )
    config, reference, reference_limit_evidence = load_reference(args_cli.config)
    if (
        args_cli.release_gripper_effort_limit_n is not None
        and args_cli.release_gripper_effort_limit_n > 50.0
    ):
        raise ValueError("release G1 effort exceeds the URDF 50 limit")
    control_period = float(config["control_period_s"])
    limits = config.get("limits", {})
    if args_cli.catch_max_joint_speed_rad_s is None:
        args_cli.catch_max_joint_speed_rad_s = float(
            limits.get("joint_speed_rad_s", 0.45)
        )
    if args_cli.catch_max_joint_acceleration_rad_s2 is None:
        args_cli.catch_max_joint_acceleration_rad_s2 = float(
            limits.get("joint_acceleration_rad_s2", 1.5)
        )
    if args_cli.gripper_open_command_time_s is None:
        args_cli.gripper_open_command_time_s = args_cli.release_time_s
    preopen_values = (
        args_cli.gripper_preopen_command_time_s,
        args_cli.gripper_preopen_drive_rad,
        args_cli.gripper_preopen_transition_s,
    )
    preopen_enabled = all(value is not None for value in preopen_values)
    if any(value is not None for value in preopen_values) and not preopen_enabled:
        raise ValueError("G1 preopen command time, drive and transition must be set together")
    if preopen_enabled:
        if args_cli.gripper_preopen_transition_s <= 0.0:
            raise ValueError("G1 preopen transition must be positive")
        if not (
            min(args_cli.partial_open_drive_rad, args_cli.held_drive_rad)
            <= args_cli.gripper_preopen_drive_rad
            <= max(args_cli.partial_open_drive_rad, args_cli.held_drive_rad)
        ):
            raise ValueError("G1 preopen target must lie between held and open targets")
        preopen_motion_start_s = (
            args_cli.gripper_preopen_command_time_s
            + args_cli.release_drive_start_delay_s
        )
        full_open_motion_start_s = (
            args_cli.gripper_open_command_time_s
            + args_cli.release_drive_start_delay_s
        )
        if (
            preopen_motion_start_s + args_cli.gripper_preopen_transition_s
            > full_open_motion_start_s
        ):
            raise ValueError("G1 preopen transition must finish before final open starts")
    preposition_pair_complete = (
        args_cli.catch_preposition_bias_m is None
    ) == (args_cli.catch_preposition_end_time_s is None)
    if not preposition_pair_complete:
        raise ValueError("catch preposition bias and end time must be paired")
    if (
        args_cli.catch_preposition_start_time_s is not None
        and args_cli.catch_preposition_bias_m is None
    ):
        raise ValueError("catch preposition start time requires a bias and end time")
    if args_cli.catch_preposition_end_time_s is not None and (
        args_cli.catch_servo_start_time_s is None
        or args_cli.catch_preposition_end_time_s
        <= args_cli.catch_servo_start_time_s
    ):
        raise ValueError("catch preposition must end after catch servo starts")
    if args_cli.catch_preposition_start_time_s is not None and (
        args_cli.catch_preposition_start_time_s
        < args_cli.catch_servo_start_time_s
        or args_cli.catch_preposition_start_time_s
        >= args_cli.catch_preposition_end_time_s
    ):
        raise ValueError("catch preposition start must lie within the servo window")
    preintercept_pair_complete = (
        args_cli.catch_preintercept_time_s is None
    ) == (args_cli.catch_intercept_switch_time_s is None)
    if not preintercept_pair_complete:
        raise ValueError("catch preintercept and switch time must be paired")
    if args_cli.catch_preintercept_time_s is not None and (
        args_cli.catch_intercept_time_s is None
        or args_cli.catch_servo_start_time_s is None
    ):
        raise ValueError("two-stage catch intercept requires catch servo and final intercept")
    if args_cli.catch_intercept_switch_time_s is not None and (
        args_cli.catch_intercept_switch_time_s
        <= args_cli.catch_servo_start_time_s
        or args_cli.catch_intercept_switch_time_s
        >= args_cli.catch_intercept_time_s
        or args_cli.catch_preintercept_time_s
        <= args_cli.catch_intercept_switch_time_s
    ):
        raise ValueError(
            "catch intercept switch must follow servo start and precede both intercepts"
        )
    duration = (
        float(reference[-1].time_s) + args_cli.arm_tracking_delay_s
        + args_cli.post_release_s
    )
    if probe_j_config is not None:
        duration = max(
            duration,
            max(
                float(item["controller"]["catch_close_time_s"])
                for item in probe_j_config["catch_candidates"]
            ) + args_cli.catch_evidence_window_s + 0.10,
        )
    elif args_cli.catch_servo_start_time_s is not None:
        if args_cli.catch_close_time_s is None:
            raise ValueError("catch servo requires --catch-close-time-s")
        if args_cli.catch_close_time_s < args_cli.catch_servo_start_time_s:
            raise ValueError("catch close time must not precede servo start")
        if (args_cli.catch_preclose_time_s is None) != (
            args_cli.catch_preclose_drive_rad is None
        ):
            raise ValueError("catch preclose time and drive must be set together")
        if args_cli.catch_preclose_time_s is not None:
            if args_cli.catch_preclose_time_s < args_cli.catch_servo_start_time_s:
                raise ValueError("catch preclose must not precede servo start")
            if args_cli.catch_preclose_time_s > args_cli.catch_close_time_s:
                raise ValueError("catch preclose must not follow final close")
            catch_drive = (
                args_cli.held_drive_rad
                if args_cli.catch_drive_rad is None
                else args_cli.catch_drive_rad
            )
            if not (
                min(args_cli.partial_open_drive_rad, catch_drive)
                <= args_cli.catch_preclose_drive_rad
                <= max(args_cli.partial_open_drive_rad, catch_drive)
            ):
                raise ValueError(
                    "catch preclose drive must lie between open and closed targets"
                )
        duration = max(
            duration,
            args_cli.catch_close_time_s + args_cli.catch_evidence_window_s + 0.10,
        )

    args_cli.output.mkdir(parents=True, exist_ok=True)
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(
            dt=float(config.get("physics_dt_s", 0.005)),
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
    insert_geometry = spawn_release_insert()
    retract_pad_geometry = spawn_release_retract_pad()
    robot_contact_prims = dict(ROBOT_CONTACT_PRIMS)
    if retract_pad_geometry is not None:
        robot_contact_prims["release_retract_pad_right"] = str(retract_pad_geometry["prim_path"])
        if retract_pad_geometry["radial_support"]:
            robot_contact_prims["release_radial_support_right"] = str(
                retract_pad_geometry["radial_support_prim_path"]
            )
    if (
        insert_geometry is not None
        and insert_geometry["shape"] in ROLLER_INSERT_SHAPES
    ):
        robot_contact_prims["release_roller_left"] = str(
            insert_geometry["prim_path"]
        )
        if insert_geometry["side"] == "both":
            robot_contact_prims["release_roller_right"] = str(
                insert_geometry["mirror_prim_path"]
            )
    cube = RigidObject(
        cube_cfg(
            args_cli.cube_size_m,
            args_cli.cube_mass_kg,
            config.get("cube_physics", {}),
        )
    )
    spawn_cube_rotation_marker(args_cli.cube_size_m)
    global_camera = None
    wrist_camera = None
    spectator_camera = None
    camera_control_enabled = args_cli.observation_mode in (
        "global_camera", "policy_cameras"
    )
    if camera_control_enabled or args_cli.record_policy_cameras:
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
    if args_cli.observation_mode == "policy_cameras" or args_cli.record_policy_cameras:
        wrist_camera = Camera(
            CameraCfg(
                prim_path="/World/WristCamera",
                update_period=1.0 / 60.0,
                height=480,
                width=640,
                data_types=["rgb", "distance_to_image_plane"],
                update_latest_camera_pose=True,
                offset=CameraCfg.OffsetCfg(
                    pos=WRIST_CAMERA_POSITION_EEF_M,
                    rot=WRIST_CAMERA_QUAT_XYZW,
                    convention="ros",
                ),
                spawn=sim_utils.PinholeCameraCfg.from_intrinsic_matrix(
                    intrinsic_matrix=GLOBAL_CAMERA_K.reshape(-1).tolist(),
                    width=640,
                    height=480,
                    clipping_range=(0.03, 2.0),
                ),
            )
        )
    if args_cli.video_path is not None:
        spectator_camera = Camera(
            CameraCfg(
                prim_path="/World/SpectatorCamera",
                update_period=1.0 / 60.0,
                height=540,
                width=960,
                data_types=["rgb"],
                offset=CameraCfg.OffsetCfg(
                    pos=SPECTATOR_CAMERA_POSITION_M,
                    rot=SPECTATOR_CAMERA_QUAT_XYZW,
                    convention="ros",
                ),
                spawn=sim_utils.PinholeCameraCfg.from_intrinsic_matrix(
                    intrinsic_matrix=SPECTATOR_CAMERA_K.reshape(-1).tolist(),
                    width=960,
                    height=540,
                    clipping_range=(0.10, 3.0),
                ),
            )
        )
    cameras = tuple(
        camera
        for camera in (global_camera, wrist_camera, spectator_camera)
        if camera is not None
    )
    for prim_path in robot_contact_prims.values():
        sim_utils.activate_contact_sensors(prim_path)
    contact_sensors = {
        name: ContactSensor(
            ContactSensorCfg(
                prim_path=prim_path,
                update_period=0.0,
                debug_vis=False,
                filter_prim_paths_expr=["/World/Cube"],
            )
        )
        for name, prim_path in robot_contact_prims.items()
    }
    sim.reset()

    arm_ids, _ = robot.find_joints("joint[1-6]")
    drive_ids, _ = robot.find_joints("drive_joint")
    roller_joint_ids, _ = (
        robot.find_joints(
            "|".join(
                (ROLLER_JOINT_NAME, RIGHT_ROLLER_JOINT_NAME)
                if args_cli.release_insert_side == "both"
                else (ROLLER_JOINT_NAME,)
            )
        )
        if insert_geometry is not None
        and insert_geometry["shape"] in ROLLER_INSERT_SHAPES
        else ([], [])
    )
    roller_joint_id = roller_joint_ids[0] if roller_joint_ids else None
    retract_joint_ids, _ = (
        robot.find_joints(RETRACT_JOINT_NAME)
        if retract_pad_geometry is not None
        else ([], [])
    )
    retract_joint_id = retract_joint_ids[0] if retract_joint_ids else None
    roller_retract_joint_ids, _ = (
        robot.find_joints(ROLLER_RETRACT_JOINT_NAME)
        if args_cli.release_roller_radial_retract_m > 0.0
        else ([], [])
    )
    roller_retract_joint_id = roller_retract_joint_ids[0] if roller_retract_joint_ids else None
    release_body_patterns: list[str] = []
    if (
        insert_geometry is not None
        and insert_geometry["shape"] in ROLLER_INSERT_SHAPES
    ):
        release_body_patterns.extend(
            ["LeftRoller", "LeftRollerCarriage"]
        )
        if args_cli.release_insert_side == "both":
            release_body_patterns.extend(
                ["RightRoller", "RightRollerCarriage"]
            )
    if retract_pad_geometry is not None:
        release_body_patterns.extend(
            ["RightPadCarriage", "RightRetractPad"]
        )
    release_body_ids: dict[str, int] = {}
    if release_body_patterns:
        release_body_index_list, release_body_names = robot.find_bodies(
            "|".join(release_body_patterns)
        )
        release_body_ids = {
            name: body_id
            for name, body_id in zip(
                release_body_names, release_body_index_list
            )
        }
    if insert_geometry is not None:
        print(f"release_mechanism_bodies={release_body_ids}")
    gripper_joint_ids, _ = robot.find_joints(
        "drive_joint|" + "|".join(GRIPPER_PASSIVE_JOINTS)
    )
    gripper_body_ids, _ = robot.find_bodies("xarm_gripper_base_link")
    finger_body_ids, _ = robot.find_bodies("left_finger|right_finger")
    if (
        len(arm_ids) != 6
        or len(drive_ids) != 1
        or len(gripper_joint_ids) != 6
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
    update_wrist_camera_pose(robot, gripper_body_id, wrist_camera)
    step_assets(sim, robot, cube, contact_sensors, cameras)
    empty_probe_records: list[dict[str, object]] = []
    if probe_j_config is not None:
        probe = probe_j_config["probe"]
        probe_duration_s = float(probe["duration_s"])
        if probe_duration_s > args_cli.settle_s:
            raise ValueError("Probe duration must fit inside --settle-s")
        empty_probe_time_s = 0.0
        start_arm_target = q[0, arm_ids].clone()
        while empty_probe_time_s < args_cli.settle_s:
            probe_target = start_arm_target.clone()
            probe_target[int(probe["joint_index"])] += probe_joint_offset_rad(
                empty_probe_time_s,
                duration_s=probe_duration_s,
                amplitude_rad=float(probe["amplitude_rad"]),
                frequency_hz=float(probe["frequency_hz"]),
            )
            robot.set_joint_position_target_index(
                target=probe_target.unsqueeze(0), joint_ids=arm_ids
            )
            update_wrist_camera_pose(robot, gripper_body_id, wrist_camera)
            step_assets(sim, robot, cube, contact_sensors, cameras)
            empty_probe_records.append(
                probe_signal_record(robot, arm_ids, drive_id)
            )
            empty_probe_time_s += sim.get_physics_dt()
        robot.write_joint_position_to_sim_index(position=q)
        robot.write_joint_velocity_to_sim_index(velocity=dq)
        robot.set_joint_position_target_index(target=q)
        robot.reset()
        update_wrist_camera_pose(robot, gripper_body_id, wrist_camera)
        step_assets(sim, robot, cube, contact_sensors, cameras)
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
    third_view_video_frames: list[np.ndarray] = []
    wrist_video_frames: list[np.ndarray] = []
    video_frame_times_s: list[float] = []
    last_video_frame_index = -1
    records: list[dict[str, object]] = []
    time_s = -args_cli.settle_s
    while time_s < 0.0:
        if probe_j_config is not None:
            probe = probe_j_config["probe"]
            probe_target = q[0, arm_ids].clone()
            probe_target[int(probe["joint_index"])] += probe_joint_offset_rad(
                time_s + args_cli.settle_s,
                duration_s=float(probe["duration_s"]),
                amplitude_rad=float(probe["amplitude_rad"]),
                frequency_hz=float(probe["frequency_hz"]),
            )
            robot.set_joint_position_target_index(
                target=probe_target.unsqueeze(0), joint_ids=arm_ids
            )
        update_wrist_camera_pose(robot, gripper_body_id, wrist_camera)
        step_assets(sim, robot, cube, contact_sensors, cameras)
        time_s += sim.get_physics_dt()
        video_frame_index = int((time_s + args_cli.settle_s) * 60.0)
        if (
            args_cli.video_path is not None
            and video_frame_index > last_video_frame_index
        ):
            video_frames.append(camera_rgb_frame(spectator_camera))
            if global_camera is not None:
                third_view_video_frames.append(
                    camera_rgb_frame(global_camera)
                )
            if wrist_camera is not None:
                wrist_video_frames.append(camera_rgb_frame(wrist_camera))
            video_frame_times_s.append(float(time_s))
            last_video_frame_index = video_frame_index
        records.append(
            record_state(
                time_s,
                "settle",
                robot,
                cube,
                arm_ids,
                drive_id,
                roller_joint_id,
                retract_joint_id,
                roller_retract_joint_id,
                release_body_ids,
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
    probe_j_evidence = None
    if probe_j_config is not None:
        held_probe_records = [
            {
                "arm_effort_nm": record["arm_joint_effort_nm"],
                "gripper_effort_nm": record["gripper_effort_nm"],
                "joint_velocity_rad_s": record["arm_joint_velocity_rad_s"],
                "gripper_position": record["gripper_drive_rad"],
            }
            for record in records
            if record["phase"] == "settle"
        ]
        posterior = estimate_probe_posterior(
            empty_arm_effort_nm=[
                item["arm_effort_nm"] for item in empty_probe_records
            ],
            held_arm_effort_nm=[
                item["arm_effort_nm"] for item in held_probe_records
            ],
            empty_gripper_effort_nm=[
                item["gripper_effort_nm"] for item in empty_probe_records
            ],
            held_gripper_effort_nm=[
                item["gripper_effort_nm"] for item in held_probe_records
            ],
            held_joint_velocity_rad_s=[
                item["joint_velocity_rad_s"] for item in held_probe_records
            ],
            held_gripper_position=[
                item["gripper_position"] for item in held_probe_records
            ],
            projected_width_m=float(probe_j_config["projected_width_m"]),
            calibration=probe_j_config["calibration"],
        )
        selected_candidate, j_ranking = select_catch_candidate(
            posterior, probe_j_config["catch_candidates"]
        )
        controller = selected_candidate["controller"]
        args_cli.catch_servo_start_time_s = float(
            controller["catch_servo_start_time_s"]
        )
        args_cli.catch_close_time_s = float(controller["catch_close_time_s"])
        args_cli.vision_control_end_time_s = float(
            controller["vision_control_end_time_s"]
        )
        args_cli.catch_intercept_time_s = float(
            controller["catch_intercept_time_s"]
        )
        args_cli.catch_lateral_only = bool(controller["catch_lateral_only"])
        args_cli.catch_hold_throw_joints = bool(
            controller["catch_hold_throw_joints"]
        )
        args_cli.catch_lock_wrist = bool(
            controller.get("catch_lock_wrist", args_cli.catch_lock_wrist)
        )
        args_cli.catch_preclose_time_s = (
            None
            if controller.get("catch_preclose_time_s") is None
            else float(controller["catch_preclose_time_s"])
        )
        args_cli.catch_preclose_drive_rad = (
            None
            if controller.get("catch_preclose_drive_rad") is None
            else float(controller["catch_preclose_drive_rad"])
        )
        args_cli.catch_position_bias_m = tuple(
            controller["catch_position_bias_m"]
        )
        args_cli.catch_drive_rad = float(controller["catch_drive_rad"])
        probe_j_evidence = {
            "probe_used_for_control": True,
            "j_used_for_control": True,
            "probe_posterior": posterior.as_dict(),
            "catch_candidate_ranking": j_ranking,
            "selected_catch_candidate": selected_candidate["name"],
            "selected_controller": controller,
        }
        probe_j_evidence["probe_gate_passed"] = bool(
            posterior.held_probability
            >= float(probe_j_config["minimum_held_probability"])
            and posterior.slip_probability
            <= float(probe_j_config["maximum_slip_probability"])
        )
        (args_cli.output / "probe_j.json").write_text(
            json.dumps(probe_j_evidence, indent=2) + "\n", encoding="utf-8"
        )
        if not probe_j_evidence["probe_gate_passed"]:
            raise RuntimeError(
                "Probe held/slip gate failed before throw: "
                f"held={posterior.held_probability:.3f}, "
                f"slip={posterior.slip_probability:.3f}"
            )

    open_target = torch.tensor(
        [[args_cli.partial_open_drive_rad]],
        device=sim.device,
    )
    preopen_target = (
        None
        if not preopen_enabled
        else torch.tensor(
            [[args_cli.gripper_preopen_drive_rad]],
            device=sim.device,
        )
    )
    catch_target = torch.tensor(
        [[
            args_cli.held_drive_rad
            if args_cli.catch_drive_rad is None
            else args_cli.catch_drive_rad
        ]],
        device=sim.device,
    )
    catch_preclose_target = (
        None
        if args_cli.catch_preclose_drive_rad is None
        else torch.tensor(
            [[args_cli.catch_preclose_drive_rad]],
            device=sim.device,
        )
    )
    release_gripper_dynamics_applied = False
    catch_gripper_dynamics_applied = False
    dt = sim.get_physics_dt()
    last_control_tick_index = -1
    commanded_arm_position = torch.tensor(
        reference[0].joint_position_rad,
        dtype=torch.float32,
        device=sim.device,
    )
    drive_start_position = commanded_arm_position.clone()
    drive_goal_position = commanded_arm_position.clone()
    drive_tick_start_s = 0.0
    commanded_arm_velocity = torch.zeros(6, device=sim.device)
    commanded_speed_max_rad_s = 0.0
    commanded_acceleration_max_rad_s2 = 0.0
    catch_arm_target = None
    catch_was_active = False
    catch_wrist_position = None
    catch_fixed_joint_indices: list[int] = []
    camera_servo_suppressed_update_count = 0
    catch_control_update_count = 0
    catch_first_jacobian = None
    catch_first_position_error = None
    catch_first_joint_delta = None
    camera_measurements: list[dict[str, object]] = []
    camera_position_errors_m: list[float] = []
    ballistic_tracker = BallisticTracker() if camera_control_enabled else None
    policy_cameras = tuple(
        item
        for item in (
            ("third_view", global_camera, 0.0),
            ("wrist", wrist_camera, 1.0 / 120.0),
        )
        if camera_control_enabled and item[1] is not None
    )
    camera_rng = np.random.default_rng(args_cli.camera_seed)
    last_camera_capture_index = {
        source_camera: -1 for source_camera, _, _ in policy_cameras
    }
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
    detach_observer_enabled = bool(
        args_cli.detach_observer_drive_delta_rad is not None
        or args_cli.detach_observer_opening_effort_nm is not None
    )
    detach_observer_triggered = False
    detach_observer_time_s = None
    detach_observer_drive_progress_rad = None
    detach_observer_opening_effort_nm = None
    release_response_baseline_drive_rad = None
    while time_s <= duration + 1.0e-9:
        tracked_reference_time_s = max(
            0.0, time_s - args_cli.arm_tracking_delay_s
        )
        sample_index = min(
            int((tracked_reference_time_s + 1.0e-9) / control_period),
            len(reference) - 1,
        )
        control_tick_index = int(
            np.floor((time_s + 1.0e-9) / control_period)
        )
        new_control_tick = control_tick_index != last_control_tick_index
        arm_target = torch.tensor(
            [reference[sample_index].joint_position_rad],
            dtype=torch.float32,
            device=sim.device,
        )
        catch_active = (
            args_cli.catch_servo_start_time_s is not None
            and time_s >= args_cli.catch_servo_start_time_s
        )
        if catch_active and not catch_was_active:
            if (
                args_cli.catch_lateral_only
                and args_cli.catch_hold_throw_joints
            ):
                commanded_arm_position = robot.data.joint_pos.torch[0, arm_ids].clone()
                commanded_arm_velocity = robot.data.joint_vel.torch[0, arm_ids].clone()
                catch_wrist_position = commanded_arm_position[1:].clone()
                commanded_arm_velocity[1:] = 0.0
            elif not args_cli.catch_lateral_only:
                if not args_cli.catch_preserve_nominal_momentum:
                    commanded_arm_position = robot.data.joint_pos.torch[0, arm_ids].clone()
                    commanded_arm_velocity = robot.data.joint_vel.torch[0, arm_ids].clone()
                catch_fixed_joint_indices = (
                    [3, 5]
                    if args_cli.catch_lock_wrist and args_cli.catch_allow_j5
                    else [3, 4, 5]
                    if args_cli.catch_lock_wrist
                    else []
                )
                if catch_fixed_joint_indices:
                    catch_wrist_position = commanded_arm_position[
                        catch_fixed_joint_indices
                    ].clone()
                    commanded_arm_velocity[catch_fixed_joint_indices] = 0.0
        catch_was_active = catch_active

        vision_control_end_time_s = args_cli.vision_control_end_time_s
        if vision_control_end_time_s is None:
            vision_control_end_time_s = args_cli.catch_close_time_s
        vision_servo_active = (
            vision_control_end_time_s is None
            or time_s <= vision_control_end_time_s + 1.0e-9
        )
        tracking_active = (
            catch_active
            or (
                args_cli.observation_mode == "proprioceptive"
                and time_s >= args_cli.gripper_open_command_time_s
            )
            or (
                camera_control_enabled
                and time_s >= args_cli.gripper_open_command_time_s
            )
        )
        if (
            tracking_active and vision_servo_active
            and new_control_tick
        ):
            if catch_active:
                catch_control_update_count += 1
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
                args_cli.gripper_open_command_time_s
                + args_cli.detach_delay_prior_s
            )
            gravity = torch.tensor([0.0, 0.0, -9.81], device=sim.device)
            if (
                not detach_observer_triggered
                and (
                    encoder_prior_position is None
                    or time_s <= nominal_detach_time_s + 1.0e-9
                )
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
            if args_cli.observation_mode == "physics":
                cube_position = cube_position_truth
                cube_velocity = cube.data.root_com_lin_vel_w.torch[0]
            elif args_cli.observation_mode == "proprioceptive":
                cube_position = prior_cube_position
                cube_velocity = prior_cube_velocity
            else:
                new_camera_metadata = []
                delivered_sources = []
                for source_camera, camera, phase_s in policy_cameras:
                    capture_index = int(
                        np.floor(
                            (
                                time_s
                                - args_cli.camera_receive_latency_s
                                - phase_s
                            )
                            * 60.0
                            + 1.0e-9
                        )
                    )
                    if capture_index <= last_camera_capture_index[source_camera]:
                        continue
                    last_camera_capture_index[source_camera] = capture_index
                    capture_time_s = phase_s + capture_index / 60.0
                    dropped = (
                        camera_rng.random()
                        < args_cli.camera_dropout_probability
                    )
                    if dropped:
                        measured_position = None
                        camera_metadata = {
                            "source_camera": source_camera,
                            "detected": 0.0,
                            "failure": "modeled_transport_dropout",
                        }
                    else:
                        before_wrist_update = (
                            np.asarray(
                                ballistic_tracker.estimate(time_s).position_m
                            )
                            if source_camera == "wrist"
                            else None
                        )
                        measured_position, camera_metadata = (
                            estimate_cube_position_from_camera(
                                camera,
                                args_cli.cube_size_m,
                                prior_cube_position,
                                source_camera,
                            )
                        )
                    if measured_position is not None:
                        last_detected_camera_time_s = capture_time_s
                        ballistic_tracker.add_camera_position(
                            capture_time_s,
                            measured_position.detach().cpu().numpy(),
                        )
                        delivered_sources.append(source_camera)
                        if source_camera == "wrist":
                            after_wrist_update = np.asarray(
                                ballistic_tracker.estimate(time_s).position_m
                            )
                            camera_metadata["wrist_state_update_norm_m"] = float(
                                np.linalg.norm(
                                    after_wrist_update - before_wrist_update
                                )
                            )
                    camera_metadata.update(
                        time_s=float(time_s),
                        capture_time_s=float(capture_time_s),
                        receive_latency_s=float(
                            time_s - capture_time_s
                        ),
                    )
                    new_camera_metadata.append(camera_metadata)
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
                if delivered_sources:
                    prediction_age_s = 0.0
                    state_source = (
                        "+".join(delivered_sources) + "_ballistic_fit"
                        if estimate.camera_sample_count >= 2
                        else "+".join(delivered_sources) + "_encoder_velocity"
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
                for camera_metadata in new_camera_metadata:
                    camera_metadata.update(
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
                camera_metadata = (
                    new_camera_metadata[-1]
                    if new_camera_metadata
                    else {"state_source": state_source}
                )
            prediction_horizon_s = args_cli.catch_prediction_horizon_s
            active_catch_intercept_time_s = args_cli.catch_intercept_time_s
            if (
                args_cli.catch_intercept_switch_time_s is not None
                and time_s < args_cli.catch_intercept_switch_time_s
            ):
                active_catch_intercept_time_s = args_cli.catch_preintercept_time_s
            if active_catch_intercept_time_s is not None:
                prediction_horizon_s = max(
                    0.0, active_catch_intercept_time_s - time_s
                )
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
            residual_feature = None
            if (
                camera_control_enabled
                and estimate.camera_sample_count > 0
            ):
                residual_feature = residual_features(
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
                camera_metadata["residual_feature"] = (
                    residual_feature.tolist()
                )
            if (
                residual_policy is not None
                and residual_feature is not None
                and estimate.camera_sample_count
                >= args_cli.residual_min_camera_samples
            ):
                residual_action = residual_policy.predict(residual_feature)
                cube_position = cube_position + torch.tensor(
                    residual_action,
                    dtype=torch.float32,
                    device=sim.device,
                )
            if camera_control_enabled:
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
                    intercept_residual_target_m=(
                        true_intercept - nominal_intercept
                    ).detach().cpu().numpy().tolist(),
                    intercept_error_before_residual_m=intercept_error_before_m,
                    intercept_error_after_residual_m=intercept_error_after_m,
                )
            active_catch_position_bias_m = args_cli.catch_position_bias_m
            if (
                args_cli.catch_preposition_end_time_s is not None
                and (
                    args_cli.catch_preposition_start_time_s is None
                    or time_s >= args_cli.catch_preposition_start_time_s
                )
                and time_s < args_cli.catch_preposition_end_time_s
            ):
                active_catch_position_bias_m = args_cli.catch_preposition_bias_m
            catch_position_bias = torch.tensor(
                active_catch_position_bias_m,
                dtype=torch.float32,
                device=sim.device,
            )
            cube_position = cube_position + catch_position_bias
            camera_servo_ready = (
                not camera_control_enabled
                or estimate.camera_sample_count
                >= args_cli.catch_min_camera_samples
            )
            if catch_active and not camera_servo_ready:
                camera_servo_suppressed_update_count += 1
            if camera_control_enabled:
                camera_metadata["camera_servo_ready"] = camera_servo_ready
            desired_hand_position = (
                cube_position - grasp_offset_world
                if camera_servo_ready
                else hand_position
            )
            position_error = desired_hand_position - hand_position
            full_jacobian = robot.data.body_link_jacobian_w.torch[
                0, gripper_body_id - 1, :3, arm_ids
            ]
            controlled_joint_indices = (
                [0]
                if args_cli.catch_lateral_only
                else [0, 1, 2, 4]
                if args_cli.catch_lock_wrist and args_cli.catch_allow_j5
                else [0, 1, 2]
                if args_cli.catch_lock_wrist
                else [0, 1, 2, 3, 4, 5]
            )
            jacobian = full_jacobian[:, controlled_joint_indices]
            damping = 1.0e-3 * torch.eye(3, device=sim.device)
            joint_weights = torch.ones(jacobian.shape[1], device=sim.device)
            if args_cli.catch_allow_j5 and 4 in controlled_joint_indices:
                joint_weights[controlled_joint_indices.index(4)] = args_cli.catch_j5_weight
            weighted_jacobian_transpose = (
                joint_weights[:, None] * jacobian.T
            )
            controlled_delta = weighted_jacobian_transpose @ torch.linalg.solve(
                jacobian @ weighted_jacobian_transpose + damping,
                position_error,
            )
            controlled_delta = torch.clamp(
                args_cli.catch_servo_gain * controlled_delta,
                -args_cli.catch_max_joint_step_rad,
                args_cli.catch_max_joint_step_rad,
            )
            if (
                args_cli.catch_joint1_start_time_s is not None
                and time_s < args_cli.catch_joint1_start_time_s
            ):
                controlled_delta[0] = 0.0
            if catch_active and catch_control_update_count == 1:
                catch_first_jacobian = [
                    [float(value) for value in row]
                    for row in jacobian.detach().cpu().tolist()
                ]
                catch_first_position_error = position_error.detach().cpu().tolist()
                catch_first_joint_delta = controlled_delta.detach().cpu().tolist()
            delta_joint = torch.zeros(6, device=sim.device)
            delta_joint[controlled_joint_indices] = controlled_delta
            if args_cli.catch_lateral_only:
                proposed_catch_arm_target = arm_target.clone()
                proposed_catch_arm_target[0, 0] = (
                    commanded_arm_position[0] + delta_joint[0]
                )
                if (
                    args_cli.catch_hold_throw_joints
                    and catch_wrist_position is not None
                ):
                    proposed_catch_arm_target[0, 1:] = catch_wrist_position
            else:
                proposed_catch_arm_target = (
                    commanded_arm_position + delta_joint
                ).unsqueeze(0)
            if (
                not args_cli.catch_lateral_only
                and catch_wrist_position is not None
            ):
                proposed_catch_arm_target[0, catch_fixed_joint_indices] = (
                    catch_wrist_position
                )
            explicit_joint_target = None
            if (
                args_cli.catch_joint_pretarget_rad is not None
                and (
                    args_cli.catch_intercept_switch_time_s is None
                    or time_s < args_cli.catch_intercept_switch_time_s
                )
            ):
                explicit_joint_target = args_cli.catch_joint_pretarget_rad
            elif args_cli.catch_joint_target_rad is not None:
                explicit_joint_target = args_cli.catch_joint_target_rad
            if explicit_joint_target is not None:
                proposed_catch_arm_target = torch.tensor(
                    [explicit_joint_target],
                    dtype=torch.float32,
                    device=sim.device,
                )
            if catch_active:
                catch_arm_target = proposed_catch_arm_target
        if catch_active and catch_arm_target is not None:
            arm_target = catch_arm_target
        if new_control_tick:
            if catch_active and catch_arm_target is not None:
                raw_velocity = (
                    arm_target[0] - commanded_arm_position
                ) / control_period
                speed_limited = torch.clamp(
                    raw_velocity,
                    -args_cli.catch_max_joint_speed_rad_s,
                    args_cli.catch_max_joint_speed_rad_s,
                )
                acceleration = torch.clamp(
                    (speed_limited - commanded_arm_velocity)
                    / control_period,
                    -args_cli.catch_max_joint_acceleration_rad_s2,
                    args_cli.catch_max_joint_acceleration_rad_s2,
                )
                commanded_arm_velocity = (
                    commanded_arm_velocity
                    + acceleration * control_period
                )
                commanded_arm_position = (
                    commanded_arm_position
                    + commanded_arm_velocity * control_period
                )
            else:
                next_velocity = torch.tensor(
                    reference[sample_index].joint_velocity_rad_s,
                    dtype=torch.float32,
                    device=sim.device,
                )
                acceleration = (
                    next_velocity - commanded_arm_velocity
                ) / control_period
                commanded_arm_position = arm_target[0]
                commanded_arm_velocity = next_velocity
            commanded_speed_max_rad_s = max(
                commanded_speed_max_rad_s,
                float(torch.max(torch.abs(commanded_arm_velocity)).item()),
            )
            commanded_acceleration_max_rad_s2 = max(
                commanded_acceleration_max_rad_s2,
                float(torch.max(torch.abs(acceleration)).item()),
            )
            drive_start_position = drive_goal_position
            drive_goal_position = commanded_arm_position.clone()
            drive_tick_start_s = time_s
            last_control_tick_index = control_tick_index
        if args_cli.arm_drive_interpolation == "linear":
            drive_progress = float(
                np.clip(
                    (time_s - drive_tick_start_s + dt) / control_period,
                    0.0,
                    1.0,
                )
            )
            drive_position = (
                drive_start_position
                + drive_progress * (drive_goal_position - drive_start_position)
            )
        else:
            drive_position = commanded_arm_position
        robot.set_joint_position_target_index(
            target=drive_position.unsqueeze(0),
            joint_ids=arm_ids,
        )
        phase = "throw" if time_s < args_cli.release_time_s else "flight"
        physical_open_start_s = (
            args_cli.gripper_open_command_time_s
            + args_cli.detach_delay_prior_s
        )
        if args_cli.release_drive_transition_s is not None:
            physical_open_start_s = (
                args_cli.gripper_open_command_time_s
                + args_cli.release_drive_start_delay_s
            )
        roller_cam_start_s = (
            physical_open_start_s
            - args_cli.release_roller_cam_lead_s
            + args_cli.release_roller_cam_delay_s
        )
        release_dynamics_start_s = physical_open_start_s
        if (
            not args_cli.release_dynamics_before_transition
            and args_cli.release_dynamics_after_transition
            and args_cli.release_drive_transition_s is not None
        ):
            release_dynamics_start_s += args_cli.release_drive_transition_s
        if (
            time_s >= release_dynamics_start_s
            and not release_gripper_dynamics_applied
            and not (
                args_cli.catch_preclose_time_s is not None
                and time_s >= args_cli.catch_preclose_time_s
            )
            and not (
                args_cli.catch_close_time_s is not None
                and time_s >= args_cli.catch_close_time_s
            )
        ):
            if args_cli.release_gripper_effort_limit_n is not None:
                robot.write_joint_effort_limit_to_sim_index(
                    limits=args_cli.release_gripper_effort_limit_n,
                    joint_ids=drive_ids,
                )
            if args_cli.release_gripper_stiffness is not None:
                robot.write_joint_stiffness_to_sim_index(
                    stiffness=args_cli.release_gripper_stiffness,
                    joint_ids=drive_ids,
                )
            release_gripper_dynamics_applied = True
        preopen_motion_start_s = None
        preopen_motion_end_s = None
        if preopen_enabled:
            preopen_motion_start_s = (
                args_cli.gripper_preopen_command_time_s
                + args_cli.release_drive_start_delay_s
            )
            preopen_motion_end_s = (
                preopen_motion_start_s + args_cli.gripper_preopen_transition_s
            )
        gripper_target = (
            open_target
            if time_s >= physical_open_start_s
            else preopen_target
            if preopen_motion_end_s is not None and time_s >= preopen_motion_end_s
            else held_target
        )
        if (
            preopen_motion_start_s is not None
            and preopen_motion_start_s <= time_s < preopen_motion_end_s
        ):
            progress = (
                (time_s - preopen_motion_start_s)
                / args_cli.gripper_preopen_transition_s
            )
            blend = progress**3 * (10.0 + progress * (-15.0 + 6.0 * progress))
            gripper_target = held_target + blend * (
                preopen_target - held_target
            )
        if (
            args_cli.release_drive_transition_s is not None
            and physical_open_start_s <= time_s
            < physical_open_start_s + args_cli.release_drive_transition_s
        ):
            progress = (
                (time_s - physical_open_start_s)
                / args_cli.release_drive_transition_s
            )
            blend = progress**3 * (10.0 + progress * (-15.0 + 6.0 * progress))
            release_start_target = (
                held_target if preopen_target is None else preopen_target
            )
            measured_drive = release_start_target + blend * (
                open_target - release_start_target
            )
            gripper_target = measured_drive
        preclose_active = (
            args_cli.catch_preclose_time_s is not None
            and time_s >= args_cli.catch_preclose_time_s
        )
        final_close_active = (
            args_cli.catch_close_time_s is not None
            and time_s >= args_cli.catch_close_time_s
        )
        if preclose_active or final_close_active:
            if not catch_gripper_dynamics_applied:
                if args_cli.catch_gripper_effort_limit_n is not None:
                    robot.write_joint_effort_limit_to_sim_index(
                        limits=args_cli.catch_gripper_effort_limit_n,
                        joint_ids=drive_ids,
                    )
                if args_cli.catch_gripper_stiffness is not None:
                    robot.write_joint_stiffness_to_sim_index(
                        stiffness=args_cli.catch_gripper_stiffness,
                        joint_ids=drive_ids,
                    )
                catch_gripper_dynamics_applied = True
        if preclose_active:
            gripper_target = catch_preclose_target
            phase = "precatch"
        if final_close_active:
            gripper_target = catch_target
            phase = "catch"
        robot.set_joint_position_target_index(
            target=gripper_target,
            joint_ids=drive_ids,
        )
        update_wrist_camera_pose(robot, gripper_body_id, wrist_camera)
        retract_pad_target_m = 0.0
        retract_pad_velocity_m_s = 0.0
        if args_cli.release_retract_pad_right:
            (
                retract_pad_target_m,
                retract_pad_velocity_m_s,
            ) = strike_return_command(
                time_s - physical_open_start_s - args_cli.release_retract_pad_delay_s,
                args_cli.release_retract_pad_travel_m,
                args_cli.release_retract_pad_transition_s,
                args_cli.release_retract_pad_return_transition_s,
                args_cli.release_retract_pad_hold_s,
            )
        retract_pad_radial_target_m = 0.0
        retract_pad_radial_velocity_m_s = 0.0
        if args_cli.release_retract_pad_radial_clear_m > 0.0:
            radial_elapsed_s = (
                time_s - physical_open_start_s
                - args_cli.release_retract_pad_radial_clear_delay_s
            )
            radial_progress = min(
                1.0,
                max(0.0, radial_elapsed_s)
                / args_cli.release_retract_pad_radial_clear_transition_s,
            )
            retract_pad_radial_target_m = (
                -args_cli.release_retract_pad_radial_clear_m * radial_progress
            )
            if 0.0 < radial_progress < 1.0:
                retract_pad_radial_velocity_m_s = (
                    -args_cli.release_retract_pad_radial_clear_m
                    / args_cli.release_retract_pad_radial_clear_transition_s
                )
        step_assets(
            sim,
            robot,
            cube,
            contact_sensors,
            cameras,
            roller_clutch_locked=time_s < roller_cam_start_s,
            roller_cam_target_rad=(
                args_cli.release_roller_cam_direction
                * args_cli.release_roller_cam_travel_rad
                * min(
                    1.0,
                    max(0.0, time_s - roller_cam_start_s)
                    / args_cli.release_roller_cam_transition_s,
                )
                if args_cli.release_roller_cam_travel_rad > 0.0 else 0.0
            ),
            retract_pad_target_m=retract_pad_target_m,
            retract_pad_velocity_m_s=retract_pad_velocity_m_s,
            retract_pad_radial_target_m=retract_pad_radial_target_m,
            retract_pad_radial_velocity_m_s=(
                retract_pad_radial_velocity_m_s
            ),
            roller_retract_target_m=(
                args_cli.release_roller_radial_retract_m
                * min(
                    1.0,
                    max(
                        0.0,
                        time_s - physical_open_start_s
                        - args_cli.release_roller_radial_retract_delay_s,
                    ) / args_cli.release_roller_radial_retract_transition_s,
                )
                if args_cli.release_roller_radial_retract_m > 0.0 else 0.0
            ),
        )
        current_drive_rad = float(
            robot.data.joint_pos.torch[0, drive_id].item()
        )
        if time_s < physical_open_start_s:
            release_response_baseline_drive_rad = current_drive_rad
        elif release_response_baseline_drive_rad is None:
            release_response_baseline_drive_rad = current_drive_rad
        if (
            detach_observer_enabled
            and not detach_observer_triggered
            and time_s >= physical_open_start_s
        ):
            (
                response_triggered,
                drive_progress_rad,
                opening_effort_nm,
            ) = g1_release_response(
                release_response_baseline_drive_rad,
                args_cli.partial_open_drive_rad,
                current_drive_rad,
                float(robot.data.applied_torque.torch[0, drive_id].item()),
                drive_delta_threshold_rad=(
                    args_cli.detach_observer_drive_delta_rad
                ),
                opening_effort_threshold_nm=(
                    args_cli.detach_observer_opening_effort_nm
                ),
            )
            if response_triggered:
                detach_observer_triggered = True
                detach_observer_time_s = float(time_s)
                detach_observer_drive_progress_rad = drive_progress_rad
                detach_observer_opening_effort_nm = opening_effort_nm
                hand_position, hand_quaternion, _ = hand_state(
                    robot, gripper_body_id, finger_body_ids
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
                encoder_prior_position = hand_position + grasp_offset_world
                encoder_prior_velocity = hand_linear_velocity + torch.linalg.cross(
                    hand_angular_velocity, grasp_offset_world
                )
                encoder_prior_time_s = float(time_s)
                if ballistic_tracker is not None:
                    ballistic_tracker.set_encoder_prior(
                        encoder_prior_time_s,
                        encoder_prior_position.detach().cpu().numpy(),
                        encoder_prior_velocity.detach().cpu().numpy(),
                    )
        records.append(
            record_state(
                time_s,
                phase,
                robot,
                cube,
                arm_ids,
                drive_id,
                roller_joint_id,
                retract_joint_id,
                roller_retract_joint_id,
                release_body_ids,
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
            video_frames.append(camera_rgb_frame(spectator_camera))
            if global_camera is not None:
                third_view_video_frames.append(
                    camera_rgb_frame(global_camera)
                )
            if wrist_camera is not None:
                wrist_video_frames.append(camera_rgb_frame(wrist_camera))
            video_frame_times_s.append(float(time_s))
            last_video_frame_index = video_frame_index
        time_s += dt

    summary = summarize(records, placed_pose, reference_limit_evidence)
    summary.update(
        cube_static_friction=(
            config["cube_physics"].get("static_friction", 1.2)
            if args_cli.cube_static_friction is None
            else args_cli.cube_static_friction
        ),
        cube_dynamic_friction=(
            config["cube_physics"].get("dynamic_friction", 0.9)
            if args_cli.cube_dynamic_friction is None
            else args_cli.cube_dynamic_friction
        ),
        cube_friction_combine_mode=args_cli.cube_friction_combine_mode,
    )
    if probe_j_evidence is None:
        summary.update(
            probe_used_for_control=False,
            j_used_for_control=False,
        )
    else:
        summary.update(probe_j_evidence)
        (args_cli.output / "probe_j.json").write_text(
            json.dumps(probe_j_evidence, indent=2) + "\n", encoding="utf-8"
        )
    detach_time_s = summary["detach_time_s"]
    post_detach_camera_updates = [
        measurement
        for measurement in camera_measurements
        if detach_time_s is not None
        and measurement["time_s"] > detach_time_s
        and measurement.get("detected", 0.0) == 1.0
        and measurement["state_source"] != "encoder_prior"
    ]
    terminal_window_start_s = (
        None
        if args_cli.catch_close_time_s is None
        else args_cli.catch_close_time_s - 0.10
    )
    terminal_wrist_observations = [
        measurement
        for measurement in camera_measurements
        if terminal_window_start_s is not None
        and measurement.get("source_camera") == "wrist"
        and measurement.get("detected", 0.0) == 1.0
        and terminal_window_start_s
        <= measurement["capture_time_s"]
        <= args_cli.catch_close_time_s
    ]
    terminal_wrist_decision_changes = [
        measurement
        for measurement in terminal_wrist_observations
        if measurement.get("wrist_state_update_norm_m", 0.0) > 1.0e-4
    ]
    spectator_flight_end_s = (
        duration
        if args_cli.catch_close_time_s is None
        else args_cli.catch_close_time_s
    )
    spectator_post_detach_frames = [
        frame_time_s
        for frame_time_s in video_frame_times_s
        if detach_time_s is not None
        and detach_time_s < frame_time_s <= spectator_flight_end_s
    ]
    learned_updates_after_detach = [
        measurement
        for measurement in camera_measurements
        if detach_time_s is not None
        and measurement["time_s"] > detach_time_s
        and measurement.get("residual_action_norm_m", 0.0) > 1.0e-6
    ]
    policy_video_paths = {}
    if args_cli.video_path is not None:
        for source, frames in (
            ("third_view", third_view_video_frames),
            ("wrist", wrist_video_frames),
        ):
            if frames:
                path = args_cli.video_path.with_name(
                    f"{args_cli.video_path.stem}_{source}"
                    f"{args_cli.video_path.suffix}"
                )
                policy_video_paths[source] = str(path)
    summary.update(
        observation_mode=args_cli.observation_mode,
        policy_observation_sources=[
            source_camera for source_camera, _, _ in policy_cameras
        ],
        policy_cameras_recorded=bool(args_cli.record_policy_cameras),
        camera_control_enabled=bool(camera_control_enabled),
        camera_seed=int(args_cli.camera_seed),
        camera_dropout_probability=float(args_cli.camera_dropout_probability),
        camera_seed_affects_control=bool(camera_control_enabled),
        spectator_used_for_control=False,
        arm_tracking_delay_s=args_cli.arm_tracking_delay_s,
        arm_drive_interpolation=args_cli.arm_drive_interpolation,
        arm_sim_effort_scale=args_cli.arm_sim_effort_scale,
        arm_sim_stiffness_scale=args_cli.arm_sim_stiffness_scale,
        commanded_max_joint_speed_rad_s=commanded_speed_max_rad_s,
        commanded_max_joint_acceleration_rad_s2=(
            commanded_acceleration_max_rad_s2
        ),
        catch_joint_speed_limit_rad_s=(
            args_cli.catch_max_joint_speed_rad_s
        ),
        catch_joint_acceleration_limit_rad_s2=(
            args_cli.catch_max_joint_acceleration_rad_s2
        ),
        catch_control_update_count=catch_control_update_count,
        catch_first_jacobian=catch_first_jacobian,
        catch_first_position_error_m=catch_first_position_error,
        catch_first_joint_delta_rad=catch_first_joint_delta,
        catch_min_camera_samples=args_cli.catch_min_camera_samples,
        camera_servo_suppressed_update_count=(
            camera_servo_suppressed_update_count
        ),
        detach_delay_prior_s=args_cli.detach_delay_prior_s,
        detach_observer_enabled=detach_observer_enabled,
        detach_observer_triggered=detach_observer_triggered,
        detach_observer_time_s=detach_observer_time_s,
        detach_observer_drive_progress_rad=(
            detach_observer_drive_progress_rad
        ),
        detach_observer_opening_effort_nm=(
            detach_observer_opening_effort_nm
        ),
        detach_state_source=(
            "g1_release_response"
            if detach_observer_triggered
            else "fixed_detach_delay_prior"
        ),
        detach_observer_error_s=(
            None
            if detach_observer_time_s is None or detach_time_s is None
            else detach_observer_time_s - detach_time_s
        ),
        encoder_prior_time_s=encoder_prior_time_s,
        catch_position_bias_m=list(args_cli.catch_position_bias_m),
        catch_preposition_bias_m=(
            None
            if args_cli.catch_preposition_bias_m is None
            else list(args_cli.catch_preposition_bias_m)
        ),
        catch_preposition_start_time_s=(
            args_cli.catch_preposition_start_time_s
        ),
        catch_preposition_end_time_s=args_cli.catch_preposition_end_time_s,
        catch_preintercept_time_s=args_cli.catch_preintercept_time_s,
        catch_intercept_switch_time_s=(
            args_cli.catch_intercept_switch_time_s
        ),
        free_flight_before_close_s=(
            None
            if detach_time_s is None
            else spectator_flight_end_s - detach_time_s
        ),
        intercept_residual_model=(
            None
            if args_cli.intercept_residual_model is None
            else str(args_cli.intercept_residual_model)
        ),
        residual_min_camera_samples=args_cli.residual_min_camera_samples,
        learned_residual_action_count=sum(
            action > 1.0e-6 for action in intercept_residual_actions_m
        ),
        learned_control_updates_after_detach=len(learned_updates_after_detach),
        vision_control_end_time_s=args_cli.vision_control_end_time_s,
        camera_measurement_count=len(camera_measurements),
        camera_control_updates_after_detach=len(post_detach_camera_updates),
        terminal_wrist_observation_count=len(terminal_wrist_observations),
        terminal_wrist_changed_decision=(
            len(terminal_wrist_decision_changes) > 0
        ),
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
        policy_video_paths=policy_video_paths,
        spectator_post_detach_frame_count=len(spectator_post_detach_frames),
    )
    if args_cli.video_path is not None:
        import imageio.v2 as imageio

        args_cli.video_path.parent.mkdir(parents=True, exist_ok=True)
        with imageio.get_writer(
            args_cli.video_path, fps=60, codec="libx264", quality=8
        ) as writer:
            for frame in video_frames:
                writer.append_data(frame)
        for source, frames in (
            ("third_view", third_view_video_frames),
            ("wrist", wrist_video_frames),
        ):
            if not frames:
                continue
            path = Path(policy_video_paths[source])
            with imageio.get_writer(
                path, fps=60, codec="libx264", quality=8
            ) as writer:
                for frame in frames:
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
        exit_code = main()
    except BaseException:
        # Kit's shutdown hooks overwrite an unhandled exception with status 0
        # on this Isaac build. Emit the failure first, then preserve status 1.
        import os
        import sys
        import traceback

        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)
    else:
        simulation_app.close()
        raise SystemExit(exit_code)
