"""Offline contracts only: these tests do not simulate ROS or Gazebo."""

import ast
import hashlib
import json
import math
from pathlib import Path
import struct
import tempfile
import unittest
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).absolute().parents[1]
LAUNCH = PACKAGE / "launch" / "simulation.launch.py"
WORLD = PACKAGE / "worlds" / "construction_site_chunk5.sdf"
ASSETS = PACKAGE / "models" / "construction_site_chunk5"


def prepare_world(world_file, mode):
    # Execute the actual pure XML/path helper without pretending ROS is installed.
    module = ast.parse(LAUNCH.read_text())
    helper = next(node for node in module.body
                  if isinstance(node, ast.FunctionDef) and node.name == "_prepare_world_tree")
    namespace = {"ET": ET}
    exec(compile(ast.Module(body=[helper], type_ignores=[]), str(LAUNCH), "exec"), namespace)
    return namespace[helper.name](world_file, mode)


def read_mesh(path):
    data = path.read_bytes()
    magic, version, total = struct.unpack_from("<4sII", data)
    assert (magic, version, total) == (b"glTF", 2, len(data))
    json_size, json_kind = struct.unpack_from("<I4s", data, 12)
    assert json_kind == b"JSON"
    document = json.loads(data[20:20 + json_size])
    binary_header = 20 + json_size
    binary_size, binary_kind = struct.unpack_from("<I4s", data, binary_header)
    assert binary_kind == b"BIN\x00"
    assert binary_header + 8 + binary_size == total
    assert all("uri" not in buffer for buffer in document["buffers"])
    assert not document.get("images")
    assert not document.get("extensionsRequired")
    assert len(document["meshes"]) == 1
    assert len(document["meshes"][0]["primitives"]) == 1
    for node in document["nodes"]:
        assert not any(key in node for key in ("matrix", "translation", "rotation", "scale"))

    def accessor(index):
        entry = document["accessors"][index]
        view = document["bufferViews"][entry["bufferView"]]
        assert view["buffer"] == 0
        fmt = "<" + {5125: "I", 5126: "f"}[entry["componentType"]] * {
            "SCALAR": 1, "VEC3": 3,
        }[entry["type"]]
        size = struct.calcsize(fmt)
        stride = view.get("byteStride", size)
        offset = entry.get("byteOffset", 0)
        assert offset + (entry["count"] - 1) * stride + size <= view["byteLength"]
        start = binary_header + 8 + view.get("byteOffset", 0) + offset
        return [struct.unpack_from(fmt, data, start + i * stride)
                for i in range(entry["count"])]

    primitive = document["meshes"][0]["primitives"][0]
    assert primitive.get("mode", 4) == 4
    vertices = accessor(primitive["attributes"]["POSITION"])
    indices = [value[0] for value in accessor(primitive["indices"])]
    assert len(indices) % 3 == 0
    assert all(index < len(vertices) for index in indices)
    assert all(math.isfinite(value) for vertex in vertices for value in vertex)
    return vertices, indices


class Chunk5IntegrationTests(unittest.TestCase):
    def test_original_collision_assets_and_metadata_are_unchanged(self):
        checksums = {
            ASSETS / "collision" / "ground_collision.glb":
                "6c34cf174d602f70a94f2e94423b3f34708813df52afc33e53a3655b629e7adc",
            ASSETS / "collision" / "site_static_collision.glb":
                "7d002090a41857a2c803b92f7f58d887368b53657008e00b4656b563e5c7a95d",
            PACKAGE / "config" / "chunk5_metadata.json":
                "29e226c9a1902ee12d46e0cb7b51877c8d5be94e763eb2bd2b8e584690df377d",
        }
        for path, checksum in checksums.items():
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), checksum)

    def test_ground_geometry_and_spawn_clearance(self):
        vertices, indices = read_mesh(ASSETS / "collision" / "ground_collision.glb")
        self.assertEqual(len(vertices), 10201)
        self.assertEqual(len(indices) // 3, 20000)
        for axis in (0, 1):
            self.assertEqual(min(vertex[axis] for vertex in vertices), -25)
            self.assertEqual(max(vertex[axis] for vertex in vertices), 25)
        heights = [vertex[2] for vertex in vertices]
        self.assertGreater(min(heights), 0.068)
        self.assertLess(max(heights), 0.088)
        self.assertGreater(0.15, max(heights))
        site, site_indices = read_mesh(ASSETS / "collision" / "site_static_collision.glb")
        self.assertEqual(len(site_indices) // 3, 14684)
        # The entire robot footprint at (-12,-12) is outside the site's AABB.
        for axis in (0, 1):
            self.assertLess(-12 + 0.4, min(vertex[axis] for vertex in site))

    def test_world_parameters_and_single_static_model(self):
        root = ET.parse(WORLD).getroot()
        self.assertEqual(root.attrib["version"], "1.9")
        world = root.find("world")
        self.assertEqual(world.findtext("physics/max_step_size"), "0.001")
        models = world.findall("model")
        self.assertEqual(len(models), 1)
        self.assertEqual(models[0].findtext("static"), "true")
        self.assertNotIn("static", models[0].attrib)
        collisions = models[0].findall("link/collision")
        self.assertEqual({node.attrib["name"] for node in collisions},
                         {"ground_collision", "site_static_collision"})
        for collision in collisions:
            self.assertEqual(collision.findtext("surface/friction/ode/mu"), "0.75")
            self.assertEqual(collision.findtext("surface/friction/ode/mu2"), "0.75")
            self.assertIsNotNone(collision.find("geometry/mesh"))
            self.assertNotIn("visual", collision.findtext("geometry/mesh/uri"))

    def test_chunk4_world_systems_are_carried_forward_once(self):
        old = ET.parse(PACKAGE / "worlds" / "test_world.sdf")
        new = ET.parse(WORLD)
        expected = {plugin.attrib["name"] for plugin in old.findall("world/plugin")}
        plugins = new.findall("world/plugin")
        self.assertEqual({plugin.attrib["name"] for plugin in plugins}, expected)
        self.assertEqual(len(plugins), len(expected))
        self.assertEqual(new.findtext("world/plugin/render_engine"), "ogre2")

    def test_collision_only_has_no_broken_mesh_uris(self):
        tree, name, messages = prepare_world(WORLD, "false")
        self.assertEqual(name, "construction_site_chunk5")
        self.assertEqual(len(messages), 1)
        self.assertIn("COLLISION-ONLY", messages[0])
        self.assertFalse(tree.findall(".//visual"))
        uris = tree.findall(".//mesh/uri")
        self.assertEqual(len(uris), 2)
        for uri in uris:
            self.assertTrue(Path(uri.text).is_absolute())
            self.assertTrue(Path(uri.text).is_file())

    def test_visual_modes_and_missing_collision_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "world.sdf"
            visual = Path(directory) / "visual.glb"
            collision = ASSETS / "collision" / "ground_collision.glb"
            path.write_text(
                '<sdf version="1.9"><world name="test"><model name="site"><link name="site">'
                '<visual name="high_detail_visual"><geometry><mesh>'
                '<uri>visual.glb</uri></mesh></geometry></visual>'
                f'<collision name="ground"><geometry><mesh><uri>{collision}</uri>'
                '</mesh></geometry></collision></link></model></world></sdf>'
            )
            tree, _, messages = prepare_world(path, "auto")
            self.assertFalse(tree.findall(".//visual"))
            self.assertTrue(messages)
            with self.assertRaises(FileNotFoundError):
                prepare_world(path, "true")
            # An actual supplied GLB is used solely as a disposable path fixture.
            visual.symlink_to(collision)
            for mode in ("auto", "true"):
                tree, _, messages = prepare_world(path, mode)
                self.assertEqual(len(tree.findall(".//visual")), 1)
                self.assertFalse(messages)
            tree, _, _ = prepare_world(path, "false")
            self.assertFalse(tree.findall(".//visual"))
            path.write_text(path.read_text().replace(str(collision), "missing.glb"))
            with self.assertRaises(FileNotFoundError):
                prepare_world(path, "auto")
            with self.assertRaises(ValueError):
                prepare_world(path, "invalid")

    def test_symlink_install_uses_installed_asset_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            share = Path(directory)
            (share / "worlds").mkdir()
            (share / "models").symlink_to(PACKAGE / "models", target_is_directory=True)
            installed_world = share / "worlds" / WORLD.name
            installed_world.symlink_to(WORLD)
            tree, _, _ = prepare_world(installed_world, "false")
            for uri in tree.findall(".//mesh/uri"):
                self.assertTrue(uri.text.startswith(str(share)))
                self.assertTrue(Path(uri.text).is_file())

    def test_old_test_world_remains_selectable(self):
        tree, name, messages = prepare_world(PACKAGE / "worlds" / "test_world.sdf", "auto")
        self.assertEqual(name, "test_world")
        self.assertFalse(messages)
        self.assertEqual(len(tree.findall("world/model")), 3)

    def test_single_launch_owner_and_nonduplicated_bridges(self):
        source = LAUNCH.read_text()
        tree = ast.parse(source)
        includes = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                    and isinstance(node.func, ast.Name)
                    and node.func.id == "IncludeLaunchDescription"]
        self.assertEqual(len(includes), 1)
        nodes = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                 and isinstance(node.func, ast.Name) and node.func.id == "Node"]
        executables = [keyword.value.value for node in nodes for keyword in node.keywords
                       if keyword.arg == "executable" and isinstance(keyword.value, ast.Constant)]
        self.assertEqual(executables.count("create"), 1)
        self.assertEqual(executables.count("robot_state_publisher"), 1)
        self.assertEqual(executables.count("parameter_bridge"), 2)
        self.assertEqual(source.count("/clock@"), 1)
        self.assertIn("sensor_bridges.yaml", source)
        self.assertIn("-r ~/cmd_vel:=/cmd_vel -r ~/odom:=/odom", source)
        self.assertNotIn("static_transform_publisher", source)
        bridge = (PACKAGE / "config" / "sensor_bridges.yaml").read_text()
        topics = [line.split(":", 1)[1].strip() for line in bridge.splitlines()
                  if line.startswith("- ros_topic_name:")]
        self.assertEqual(set(topics), {"/scan", "/camera/image_raw", "/camera/camera_info", "/imu/data"})
        self.assertEqual(len(topics), len(set(topics)))
        self.assertEqual(bridge.count("direction: GZ_TO_ROS"), 4)

    def test_existing_sensor_and_tf_contracts(self):
        sensors = ET.parse(PACKAGE / "urdf" / "gazebo_sensors.xacro")
        frames = {sensor.findtext("topic"): sensor.findtext("gz_frame_id")
                  for sensor in sensors.findall(".//sensor")}
        self.assertEqual(frames, {"/scan": "laser_link", "/imu/data": "imu_link",
                                  "/camera/image_raw": "camera_optical_frame"})
        control = (PACKAGE / "config" / "controllers.yaml").read_text()
        for setting in ("odom_frame_id: odom", "base_frame_id: base_footprint",
                        "enable_odom_tf: true", "wheel_radius: 0.10", "wheel_separation: 0.45"):
            self.assertIn(setting, control)

    def test_launch_syntax_and_installation_targets(self):
        for path in (PACKAGE.parent / "surveillance_robot_bringup" / "launch").glob("*.py"):
            ast.parse(path.read_text())
        ast.parse(LAUNCH.read_text())
        cmake = (PACKAGE / "CMakeLists.txt").read_text()
        for directory in ("worlds", "models", "plugins", "config", "launch", "urdf"):
            self.assertTrue((PACKAGE / directory).is_dir())
        self.assertIn("test/test_chunk5_integration.py", cmake)
        self.assertNotIn("scripts/check_sensors.py", cmake)
        self.assertNotIn("test/test_sensor_contracts.py", cmake)


if __name__ == "__main__":
    unittest.main()
