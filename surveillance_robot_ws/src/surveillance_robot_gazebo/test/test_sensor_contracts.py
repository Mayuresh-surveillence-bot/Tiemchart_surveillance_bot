"""Source-tree checks only; these do not run ROS, Gazebo, or SDF conversion."""

import ast
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import pytest
import xacro
import xacro.substitution_args
import yaml


GAZEBO = Path(__file__).resolve().parents[1]
DESCRIPTION = GAZEBO.parent / "surveillance_robot_description"
BRINGUP = GAZEBO.parent / "surveillance_robot_bringup"
PACKAGES = {path.name: path for path in (GAZEBO, DESCRIPTION, BRINGUP)}
TOPICS = {
    "/scan": ("LaserScan", "LaserScan", "laser_link"),
    "/camera/image_raw": ("Image", "Image", "camera_optical_frame"),
    "/camera/camera_info": ("CameraInfo", "CameraInfo", "camera_optical_frame"),
    "/imu/data": ("Imu", "IMU", "imu_link"),
}


def load_yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def models():
    # Only package-share lookup is redirected; Xacro expansion itself is real.
    # This does not verify the ament index or an installed ROS workspace.
    with pytest.MonkeyPatch.context() as patch:
        patch.setattr(
            xacro.substitution_args, "_eval_find", lambda name: str(PACKAGES[name])
        )
        return {
            mode: ET.fromstring(
                xacro.process_file(
                    str(DESCRIPTION / "urdf/robot.urdf.xacro"),
                    mappings={"sim_mode": mode},
                ).toxml()
            )
            for mode in ("false", "true")
        }


@pytest.mark.parametrize("package", PACKAGES.values(), ids=PACKAGES.keys())
def test_resource_syntax(package):
    for pattern in ("**/*.xml", "**/*.xacro", "**/*.sdf"):
        for path in package.glob(pattern):
            ET.parse(path)
    for path in package.glob("launch/*.py"):
        compile(path.read_text(encoding="utf-8"), str(path), "exec")
    for pattern in ("config/*.yaml", "rviz/*.rviz"):
        for path in package.glob(pattern):
            assert load_yaml(path) is not None


def test_description_only_remains_sensor_free(models):
    visual, simulated = models["false"], models["true"]
    assert not visual.findall("gazebo")
    assert not visual.findall("ros2_control")
    for tag in ("link", "joint"):
        assert [ET.tostring(node) for node in visual.findall(tag)] == [
            ET.tostring(node) for node in simulated.findall(tag)
        ]
    assert len(simulated.findall("ros2_control")) == 1


def test_sensor_tf_tree(models):
    robot = models["true"]
    links = [link.attrib["name"] for link in robot.findall("link")]
    joints = robot.findall("joint")
    children = [joint.find("child").attrib["link"] for joint in joints]
    assert len(links) == len(set(links))
    assert len(children) == len(set(children))
    assert set(children) <= set(links)
    assert set(links) - set(children) == {"base_footprint"}
    assert not {"map", "odom"} & set(links)
    parents = {
        joint.find("child").attrib["link"]: joint.find("parent").attrib["link"]
        for joint in joints
    }
    for link in links:
        visited = set()
        while link in parents:
            assert link not in visited
            visited.add(link)
            link = parents[link]
        assert link == "base_footprint"
    for frame in ("laser_link", "camera_link", "imu_link"):
        joint = next(j for j in joints if j.find("child").attrib["link"] == frame)
        assert joint.attrib["type"] == "fixed"
        assert parents[frame] == "base_link"
        assert joint.find("origin").attrib["rpy"] == "0 0 0"
    optical = robot.find("joint[@name='camera_optical_joint']")
    assert parents["camera_optical_frame"] == "camera_link"
    assert optical.attrib["type"] == "fixed"
    assert optical.find("origin").attrib["xyz"] == "0 0 0"
    angles = [float(v) for v in optical.find("origin").attrib["rpy"].split()]
    assert angles == pytest.approx([-math.pi / 2, 0, -math.pi / 2])
    for name in ("laser_joint", "camera_joint", "imu_joint"):
        assert robot.findtext(f"gazebo[@reference='{name}']/preserveFixedJoint") == "true"


def test_sensor_bridge_topics_types_and_frames(models):
    robot = models["true"]
    sensors = robot.findall("gazebo/sensor")
    assert len(sensors) == 3
    assert {s.attrib["type"] for s in sensors} == {"gpu_lidar", "camera", "imu"}
    emitted = {}
    for sensor in sensors:
        assert sensor.findtext("pose") == "0 0 0 0 0 0"
        assert sensor.findtext("always_on") == "true"
        frame = sensor.findtext("gz_frame_id")
        assert robot.find(f"link[@name='{frame}']") is not None
        emitted[sensor.findtext("topic")] = frame
        camera = sensor.find("camera")
        if camera is not None:
            emitted[camera.findtext("camera_info_topic")] = camera.findtext("optical_frame_id")
    assert emitted == {topic: types[2] for topic, types in TOPICS.items()}
    bridges = load_yaml(GAZEBO / "config/sensor_bridges.yaml")
    assert len(bridges) == len(TOPICS)
    assert {b["ros_topic_name"] for b in bridges} == set(TOPICS)
    for bridge in bridges:
        topic = bridge["ros_topic_name"]
        ros_type, gz_type, _ = TOPICS[topic]
        assert bridge["gz_topic_name"] == topic
        assert bridge["ros_type_name"] == f"sensor_msgs/msg/{ros_type}"
        assert bridge["gz_type_name"] == f"gz.msgs.{gz_type}"
        assert bridge["direction"] == "GZ_TO_ROS"
        assert bridge["qos_profile"] == "SENSOR_DATA"
        assert bridge["lazy"] is False
        assert "frame_id" not in bridge


def test_sensor_parameters(models):
    robot = models["true"]
    lidar = robot.find("gazebo[@reference='laser_link']/sensor")
    assert float(lidar.findtext("update_rate")) == 10.0
    horizontal = lidar.find("lidar/scan/horizontal")
    samples = int(horizontal.findtext("samples"))
    minimum = float(horizontal.findtext("min_angle"))
    maximum = float(horizontal.findtext("max_angle"))
    step = (maximum - minimum) / (samples - 1)
    assert samples == 720
    assert step == pytest.approx(math.radians(0.5))
    assert minimum == pytest.approx(-math.pi)
    assert maximum == pytest.approx(math.pi - step)
    assert minimum + 360 * step == pytest.approx(0)
    assert lidar.findtext("lidar/scan/vertical/samples") == "1"
    assert lidar.findtext("lidar/scan/vertical/min_angle") == "0"
    assert lidar.findtext("lidar/scan/vertical/max_angle") == "0"
    assert float(lidar.findtext("lidar/range/min")) == 0.12
    assert float(lidar.findtext("lidar/range/max")) == 12.0
    camera = robot.find("gazebo[@reference='camera_link']/sensor")
    assert float(camera.findtext("update_rate")) == 15.0
    assert camera.findtext("camera/image/width") == "640"
    assert camera.findtext("camera/image/height") == "480"
    assert camera.findtext("camera/image/format") == "R8G8B8"
    assert float(camera.findtext("camera/horizontal_fov")) == pytest.approx(math.radians(80))
    assert 0 < float(camera.findtext("camera/clip/near")) < float(camera.findtext("camera/clip/far"))
    imu = robot.find("gazebo[@reference='imu_link']/sensor")
    assert float(imu.findtext("update_rate")) == 50.0
    assert imu.findtext("imu/orientation_reference_frame/localization") == "ENU"
    for quantity in ("angular_velocity", "linear_acceleration"):
        for axis in ("x", "y", "z"):
            assert float(imu.findtext(f"imu/{quantity}/{axis}/noise/stddev")) > 0


def test_world_sensor_systems():
    world = ET.parse(GAZEBO / "worlds/test_world.sdf").getroot().find("world")
    plugins = world.findall("plugin")
    names = [plugin.attrib["name"] for plugin in plugins]
    assert len(names) == len(set(names))
    assert {f"gz::sim::systems::{name}" for name in (
        "Physics", "UserCommands", "SceneBroadcaster", "Contact", "Sensors", "Imu"
    )} <= set(names)
    rendering = world.find("plugin[@name='gz::sim::systems::Sensors']")
    assert rendering.attrib["filename"] == "gz-sim-sensors-system"
    assert rendering.findtext("render_engine") == "ogre2"
    assert world.find("plugin[@name='gz::sim::systems::Imu']").attrib["filename"] == "gz-sim-imu-system"


def test_existing_launch_wires_sensor_bridge():
    tree = ast.parse((GAZEBO / "launch/simulation.launch.py").read_text())
    assignments = {
        node.targets[0].id: node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)
    }
    bridge = assignments["sensor_bridge"]
    assert bridge.func.id == "Node"
    keywords = {entry.arg: entry.value for entry in bridge.keywords}
    assert ast.literal_eval(keywords["package"]) == "ros_gz_bridge"
    assert ast.literal_eval(keywords["executable"]) == "parameter_bridge"
    parameters = keywords["parameters"].elts[0]
    values = {ast.literal_eval(key): value for key, value in zip(parameters.keys, parameters.values)}
    assert values["config_file"].id == "sensor_bridges_file"
    assert ast.literal_eval(values["use_sim_time"]) is True
    assert assignments["sensor_bridges_file"].args[0].elts[-1].value == "sensor_bridges.yaml"
    launch = next(node.value for node in ast.walk(tree) if isinstance(node, ast.Return))
    actions = [node.id for node in launch.args[0].elts]
    for name in ("sensor_bridge", "clock_bridge", "robot_state_publisher", "spawn_robot",
                 "load_jsb_after_spawn", "load_diff_after_jsb"):
        assert actions.count(name) == 1


def test_rviz_uses_sensor_qos_and_existing_base_frame():
    rviz = load_yaml(DESCRIPTION / "rviz/robot_description.rviz")["Visualization Manager"]
    assert rviz["Global Options"]["Fixed Frame"] == "base_footprint"
    displays = {display["Class"]: display for display in rviz["Displays"]}
    for kind, topic in (("LaserScan", "/scan"), ("Image", "/camera/image_raw")):
        display = displays[f"rviz_default_plugins/{kind}"]
        assert display["Enabled"] is True
        assert display["Topic"]["Value"] == topic
        assert display["Topic"]["Reliability Policy"] == "Best Effort"
        assert display["Topic"]["Durability Policy"] == "Volatile"


def test_install_rules_and_dependencies():
    for package in PACKAGES.values():
        cmake = (package / "CMakeLists.txt").read_text()
        for block in re.findall(r"install\((.*?)\)", cmake, re.DOTALL):
            resources = re.search(r"(?:DIRECTORY|PROGRAMS|FILES)\s+(.*?)\s+DESTINATION", block, re.DOTALL)
            assert resources is not None
            for resource in resources.group(1).split():
                assert (package / resource).exists(), resource
        for test in re.findall(r"ament_add_pytest_test\(\S+\s+(\S+)\)", cmake):
            assert (package / test).is_file(), test
    package = ET.parse(GAZEBO / "package.xml").getroot()
    dependencies = {entry.text for entry in package.findall("exec_depend")}
    assert {"ros_gz_sim", "ros_gz_bridge", "sensor_msgs", "rosgraph_msgs", "xacro",
            "python3-yaml", "launch", "launch_ros", "surveillance_robot_description",
            "gz_ros2_control", "controller_manager", "diff_drive_controller",
            "joint_state_broadcaster", "robot_state_publisher", "rviz2"} <= dependencies
    assert {"ament_cmake_pytest", "python3-pytest"} <= {
        entry.text for entry in package.findall("test_depend")
    }
