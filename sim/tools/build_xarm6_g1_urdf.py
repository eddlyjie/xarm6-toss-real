#!/usr/bin/env python3
"""Build a self-contained xArm6 + UFACTORY G1 URDF asset.

The arm comes from the RobotCamCalib URDF whose FK has already been checked
against this robot.  The G1 mechanism and meshes come from UFACTORY's official
xarm_ros2 description.  This builder expands only the G1 geometry macros; it
does not require ROS or xacro.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


XACRO = "http://ros.org/wiki/xacro"
XACRO_TAG = f"{{{XACRO}}}"
PROJECT_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_ARM_URDF = (
    PROJECT_ROOT
    / "RobotCamCalib"
    / "RobotCamCalib"
    / "assets"
    / "robots"
    / "xarm6"
    / "xarm6_wo_ee.urdf"
)
DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "assets" / "xarm6_g1"


def _substitute(value: str) -> str:
    replacements = {
        "${prefix}": "",
        "${attach_to}": "link_eef",
        "${attach_xyz}": "0 0 0",
        "${attach_rpy}": "0 0 0",
        "${name_suffix}": "",
    }
    for source, target in replacements.items():
        value = value.replace(source, target)
    return value


def _expand_attributes(element: ET.Element) -> None:
    for name, value in tuple(element.attrib.items()):
        element.set(name, _substitute(value))


def _mesh_element(filename: str) -> ET.Element:
    mesh_name = Path(filename).name
    geometry = ET.Element("geometry")
    ET.SubElement(
        geometry,
        "mesh",
        {"filename": f"meshes/gripper/{mesh_name}.stl"},
    )
    return geometry


def _expand_common_link(element: ET.Element, *, collision: bool) -> ET.Element:
    tag = "collision" if collision else "visual"
    result = ET.Element(tag)
    result.append(_mesh_element(_substitute(element.attrib["mesh_filename"])))
    ET.SubElement(
        result,
        "origin",
        {
            "xyz": _substitute(element.attrib["origin_xyz"]),
            "rpy": _substitute(element.attrib["origin_rpy"]),
        },
    )
    if not collision:
        ET.SubElement(
            result,
            "material",
            {"name": _substitute(element.attrib["material_name"])},
        )
    return result


def _expand_g1_child(source: ET.Element) -> list[ET.Element]:
    if source.tag == f"{XACRO_TAG}property":
        return []
    if source.tag == f"{XACRO_TAG}if":
        return []  # This branch contains the G2-only property overrides.
    if source.tag == f"{XACRO_TAG}unless":
        expanded: list[ET.Element] = []
        for child in source:
            expanded.extend(_expand_g1_child(child))
        return expanded
    if source.tag == f"{XACRO_TAG}common_link_visual":
        return [_expand_common_link(source, collision=False)]
    if source.tag == f"{XACRO_TAG}common_link_collision":
        return [_expand_common_link(source, collision=True)]

    target = ET.Element(source.tag)
    target.text = source.text
    target.tail = source.tail
    target.attrib.update(source.attrib)
    _expand_attributes(target)
    for child in source:
        for expanded in _expand_g1_child(child):
            target.append(expanded)
    return [target]


def _append_g1(robot: ET.Element, gripper_xacro: Path) -> None:
    source_root = ET.parse(gripper_xacro).getroot()
    macro = source_root.find(f"{XACRO_TAG}macro")
    if macro is None:
        raise ValueError("xarm_gripper.urdf.xacro has no xacro macro")
    for child in macro:
        for expanded in _expand_g1_child(child):
            robot.append(deepcopy(expanded))


def _copy_arm_meshes(robot: ET.Element, arm_urdf: Path, output: Path) -> None:
    visual_dir = output / "meshes" / "xarm6" / "visual"
    collision_dir = output / "meshes" / "xarm6" / "collision"
    visual_dir.mkdir(parents=True, exist_ok=True)
    collision_dir.mkdir(parents=True, exist_ok=True)

    for link in robot.findall("link"):
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue
            source = arm_urdf.parent / mesh.attrib["filename"]
            target = visual_dir / source.name
            shutil.copy2(source, target)
            mesh.set("filename", f"meshes/xarm6/visual/{target.name}")
        for collision in link.findall("collision"):
            mesh = collision.find("geometry/mesh")
            if mesh is None:
                continue
            source = arm_urdf.parent / mesh.attrib["filename"]
            convex_source = source.with_name(f"{source.name}.convex.stl")
            if convex_source.exists():
                source = convex_source
            target = collision_dir / source.name
            shutil.copy2(source, target)
            mesh.set("filename", f"meshes/xarm6/collision/{target.name}")


def _copy_gripper_meshes(ufactory_root: Path, output: Path) -> None:
    source_dir = (
        ufactory_root
        / "xarm_description"
        / "meshes"
        / "gripper"
        / "xarm"
    )
    target_dir = output / "meshes" / "gripper"
    target_dir.mkdir(parents=True, exist_ok=True)
    for source in sorted(source_dir.glob("*.stl")):
        shutil.copy2(source, target_dir / source.name)


def _validate(robot: ET.Element, output: Path) -> None:
    links = {element.attrib["name"] for element in robot.findall("link")}
    joints = {element.attrib["name"] for element in robot.findall("joint")}
    required_links = {
        "link_base",
        "link6",
        "link_eef",
        "xarm_gripper_base_link",
        "left_finger",
        "right_finger",
        "link_tcp",
    }
    required_joints = {
        "joint1",
        "joint6",
        "gripper_fix",
        "drive_joint",
        "joint_tcp",
    }
    if not required_links <= links or not required_joints <= joints:
        raise ValueError("generated URDF is missing required xArm6/G1 elements")
    mimic_joints = robot.findall("joint/mimic")
    if len(mimic_joints) != 5:
        raise ValueError("G1 must contain five joints mimicking drive_joint")
    for mesh in robot.findall(".//mesh"):
        if not (output / mesh.attrib["filename"]).is_file():
            raise ValueError(f"missing mesh: {mesh.attrib['filename']}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ufactory-root", type=Path, required=True)
    parser.add_argument("--arm-urdf", type=Path, default=DEFAULT_ARM_URDF)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    robot_tree = ET.parse(args.arm_urdf)
    robot = robot_tree.getroot()
    _copy_arm_meshes(robot, args.arm_urdf, args.output)
    gripper_xacro = (
        args.ufactory_root
        / "xarm_description"
        / "urdf"
        / "gripper"
        / "xarm_gripper.urdf.xacro"
    )
    _append_g1(robot, gripper_xacro)
    _copy_gripper_meshes(args.ufactory_root, args.output)
    shutil.copy2(args.ufactory_root / "LICENSE", args.output / "UF_LICENSE")

    ET.indent(robot_tree, space="  ")
    urdf_path = args.output / "xarm6_g1.urdf"
    robot_tree.write(urdf_path, encoding="utf-8", xml_declaration=True)
    _validate(robot, args.output)
    print(f"generated={urdf_path}")
    print("arm joints=6, G1 drive=1, G1 mimic joints=5, TCP offset=0.172 m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
