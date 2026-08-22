import os
import importlib.util
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def _expanded_robot():
    share = get_package_share_directory("mfr3duo_description")
    prefix = Path(share).parents[1]
    environment = os.environ.copy()
    environment["AMENT_PREFIX_PATH"] = f"{prefix}:/opt/ros/humble"
    environment["CMAKE_PREFIX_PATH"] = environment["AMENT_PREFIX_PATH"]
    result = subprocess.run(
        ["xacro", f"{share}/urdf/mfr3duo.urdf.xacro"],
        check=True,
        capture_output=True,
        text=True,
        env=environment,
    )
    return ET.fromstring(result.stdout)


def _static_robot():
    share = Path(get_package_share_directory("mfr3duo_description"))
    return ET.fromstring((share / "urdf" / "mfr3duo.urdf").read_text())


def test_expands_to_complete_default_robot():
    robot = _expanded_robot()
    assert robot.attrib["name"] == "mfr3duo"
    links = {element.attrib["name"] for element in robot.findall("link")}
    joints = {element.attrib["name"] for element in robot.findall("joint")}

    required_links = {
        "base_link",
        "imu_imu_sensor_frame",
        "front_realsense_d455_color_optical_frame",
        "rear_realsense_d455_color_optical_frame",
        "lidar_front_scan_frame",
        "lidar_rear_scan_frame",
        "head_zed_mini_center_frame",
        "left_d435_color_optical_frame",
        "right_d435_color_optical_frame",
    }
    assert required_links <= links
    assert "left_wrist_camera_mount_joint" in joints
    assert "right_wrist_camera_mount_joint" in joints
    assert len(links) == len(set(links))
    assert len(joints) == len(set(joints))

    origins = {
        joint.attrib["name"]: joint.find("origin").attrib
        for joint in robot.findall("joint")
        if joint.find("origin") is not None
    }
    # Current wrist-camera transforms are explicitly provisional figure-fit data.
    assert origins["left_wrist_camera_mount_joint"]["xyz"] == "-0.035 0 0"
    assert origins["right_wrist_camera_mount_joint"]["xyz"] == "-0.035 0 0"
    assert origins["left_d435_bottom_screw_frame_joint"]["xyz"] == "0 0 0.055"
    assert origins["left_d435_bottom_screw_frame_joint"]["rpy"] == "0 -1.5707963267948966 0"


def test_committed_default_urdf_matches_xacro_and_is_self_contained():
    static = _static_robot()
    expanded = _expanded_robot()
    assert ET.tostring(static) == ET.tostring(expanded)

    share = Path(get_package_share_directory("mfr3duo_description"))
    content = (share / "urdf" / "mfr3duo.urdf").read_text()
    assert "<xacro:" not in content
    assert "$(find " not in content
    assert "franka_description" not in content
    assert "mobile_fr3_duo" not in content
    for mesh in static.findall(".//mesh"):
        assert mesh.attrib["filename"].startswith(
            "package://mfr3duo_description/urdf/meshes/"
        )


def test_model_has_no_external_description_references():
    share = Path(get_package_share_directory("mfr3duo_description"))
    package_root = share.parents[1]
    urdf_root = share / "urdf"

    for source in [path for path in urdf_root.rglob("*.xacro") if path.is_file()] + [share / "package.xml"]:
        content = source.read_text()
        assert "franka_description" not in content
        assert "mobile_fr3_duo" not in content

    robot = _expanded_robot()
    mesh_uris = [
        mesh.attrib["filename"]
        for mesh in robot.findall(".//mesh")
    ]
    assert mesh_uris
    expected_prefix = "package://mfr3duo_description/urdf/meshes/"
    for uri in mesh_uris:
        assert uri.startswith(expected_prefix)
        relative_path = uri.removeprefix("package://mfr3duo_description/")
        assert (package_root / "share" / "mfr3duo_description" / relative_path).is_file()


def test_urdf_source_layout_is_flat_and_parameter_data_is_inlined():
    source_urdf = Path(__file__).parents[1] / "urdf"
    assert {path.name for path in source_urdf.iterdir() if path.is_dir()} == {"meshes"}
    assert not list(source_urdf.rglob("*.yaml"))

    mesh_models = {
        path.name for path in (source_urdf / "meshes").iterdir() if path.is_dir()
    }
    model_xacros = {path.stem for path in source_urdf.glob("*.xacro")}
    assert mesh_models <= model_xacros
    assert model_xacros == {
        "mfr3duo.urdf",
        "mfr3duo",
        "franka_tmr",
        "franka_spine",
        "franka_head",
        "franka_fr3",
        "franka_hand",
        "nanoscan3",
        "realsense_d435",
        "realsense_d455",
        "wrist_camera_mount",
        "zed_mini",
        "imu",
    }

    for source in source_urdf.glob("*.xacro"):
        content = source.read_text()
        assert "load_yaml" not in content
        assert "/vendor/" not in content
        assert "/sensors/" not in content
        assert "/mounts/" not in content
        assert "/common/" not in content
        assert 'filename="$(find mfr3duo_description)/urdf/franka_parameters.xacro"' not in content

    package_root = Path(__file__).parents[1]
    assert not (package_root / "source").exists()
    assert not (package_root / "third_party").exists()


def test_frozen_franka_limits_and_inertials_are_preserved():
    robot = _expanded_robot()
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    limit = joints["left_fr3v2_1_joint1"].find("limit").attrib
    assert limit == {
        "effort": "87.0",
        "lower": "-2.9007400166666666",
        "upper": "2.9007400166666666",
        "velocity": "2.62",
    }

    links = {link.attrib["name"]: link for link in robot.findall("link")}
    left_link0_inertial = links["left_fr3v2_1_link0"].find("inertial")
    assert left_link0_inertial.find("mass").attrib["value"] == "2.3966"
    assert left_link0_inertial.find("origin").attrib["xyz"] == "-0.017200 0.000400 0.074500"


def test_runtime_meshes_are_local_and_not_legacy_obj_assets():
    share = Path(get_package_share_directory("mfr3duo_description"))
    package_root = share.parents[1]
    mesh_root = share / "urdf" / "meshes"

    for mesh in mesh_root.rglob("*"):
        if mesh.is_file():
            assert mesh.suffix.lower() != ".obj"

    assert not (share / "source").exists()
    assert not (share / "third_party").exists()
    for mesh in mesh_root.rglob("*"):
        if mesh.is_file():
            assert mesh.suffix.lower() in {".dae", ".stl"}


def test_visualization_script_and_rviz_config_are_installed():
    share = Path(get_package_share_directory("mfr3duo_description"))
    prefix = share.parents[1]
    script = prefix / "lib" / "mfr3duo_description" / "visualize_urdf"
    assert script.is_file()
    assert os.access(script, os.X_OK)

    script_content = script.read_text()
    assert "urdf/mfr3duo.urdf" in script_content
    assert "mktemp" in script_content
    assert "--params-file" in script_content
    assert "robot_description:=\"" not in script_content
    assert "trap cleanup" in script_content

    rviz_config = share / "rviz" / "mfr3duo.rviz"
    config_content = rviz_config.read_text()
    assert "Fixed Frame: base_link" in config_content
    assert "Value: /robot_description" in config_content


def test_mjcf_resources_are_self_contained_and_loadable():
    package_root = Path(__file__).parents[1]
    mjcf_root = package_root / "mjcf"
    components = {
        "franka_tmr", "franka_spine", "franka_head", "franka_fr3", "franka_hand",
        "imu", "nanoscan3", "realsense_d435", "realsense_d455",
        "wrist_camera_mount", "zed_mini",
    }
    assert {path.stem for path in mjcf_root.glob("*.xml")} == components | {
        "mfr3duo", "scene"
    }
    assert {path.name for path in (mjcf_root / "meshes").iterdir() if path.is_dir()} == components - {"imu"}

    for xml_path in mjcf_root.glob("*.xml"):
        content = xml_path.read_text()
        assert "mobile_fr3_duo" not in content
        root = ET.fromstring(content)
        compiler = root.find("compiler")
        meshdir = compiler.attrib.get("meshdir", ".") if compiler is not None else "."
        for mesh in root.findall(".//mesh"):
            if "file" in mesh.attrib:
                assert (xml_path.parent / meshdir / mesh.attrib["file"]).is_file()

    if importlib.util.find_spec("mujoco") is not None:
        import mujoco

        for name in ("mfr3duo.xml", "scene.xml"):
            mujoco.MjModel.from_xml_path(str(mjcf_root / name))

    documentation = (package_root / "docs" / "mjcf.md").read_text()
    for source in (
        "franka_description", "realsense-ros", "sick_safetyscanners2",
        "zed-ros2-description", "Franka 3D Assets",
    ):
        assert source in documentation
    assert "不构成模型的最终来源" in documentation
