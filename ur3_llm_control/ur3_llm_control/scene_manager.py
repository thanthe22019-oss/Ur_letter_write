"""Synchronize scene.yaml with the MoveIt Planning Scene and RViz markers."""

from typing import Iterable

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, ObjectColor, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from ur3_llm_control.scene_config import BoxSpec, load_scene
from visualization_msgs.msg import Marker
import yaml


def _pose(spec: BoxSpec) -> Pose:
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = spec.pose[:3]
    # Milestone 2 boxes are axis-aligned. Rotation support remains in Gazebo's
    # SDF pose and can be extended here when non-zero scene rotations are used.
    if any(abs(value) > 1.0e-9 for value in spec.pose[3:]):
        raise ValueError(f"MoveIt box '{spec.name}' must currently be axis-aligned")
    pose.orientation.w = 1.0
    return pose


def _collision_object(spec: BoxSpec, frame_id: str) -> CollisionObject:
    collision_object = CollisionObject()
    collision_object.header.frame_id = frame_id
    collision_object.id = spec.name
    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = spec.size
    collision_object.primitives.append(primitive)
    collision_object.primitive_poses.append(_pose(spec))
    collision_object.operation = CollisionObject.ADD
    return collision_object


def _object_color(spec: BoxSpec) -> ObjectColor:
    color = ObjectColor()
    color.id = spec.name
    color.color = ColorRGBA(
        r=spec.color[0],
        g=spec.color[1],
        b=spec.color[2],
        a=spec.color[3],
    )
    return color


class SceneManager(Node):
    """Apply configured collision geometry and keep zone markers available."""

    def __init__(self) -> None:
        super().__init__('scene_manager')
        self.declare_parameter('scene_file', '')
        scene_file = self.get_parameter('scene_file').get_parameter_value().string_value
        if not scene_file:
            raise ValueError("parameter 'scene_file' must not be empty")
        self.scene = load_scene(scene_file)

        marker_qos = QoSProfile(depth=100)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_publisher = self.create_publisher(Marker, 'scene_markers', marker_qos)
        self.apply_scene_client = self.create_client(
            ApplyPlanningScene, '/apply_planning_scene'
        )

    def apply(self) -> bool:
        """Wait for MoveIt, apply collision geometry, and publish zone markers."""
        self.get_logger().info('Waiting for MoveIt Planning Scene service...')
        if not self.apply_scene_client.wait_for_service(timeout_sec=60.0):
            self.get_logger().error('/apply_planning_scene was not available within 60 seconds')
            return False

        collision_specs = [self.scene.floor, self.scene.table]
        collision_specs.extend(self.scene.objects.values())
        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.name = 'ur3_llm_workspace'
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects = [
            _collision_object(spec, self.scene.frame_id) for spec in collision_specs
        ]
        request.scene.object_colors = [_object_color(spec) for spec in collision_specs]

        future = self.apply_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        response = future.result()
        if response is None or not response.success:
            self.get_logger().error('MoveIt rejected the configured Planning Scene')
            return False

        self._publish_zone_markers(self.scene.zones.values())
        object_count = len(self.scene.objects)
        zone_count = len(self.scene.zones)
        self.get_logger().info(
            f'Scene ready: floor + table + {object_count} cubes in MoveIt, '
            f'{zone_count} zones in RViz'
        )
        return True

    def _publish_zone_markers(self, zones: Iterable[BoxSpec]) -> None:
        for marker_index, zone in enumerate(zones):
            zone_marker = Marker()
            zone_marker.header.frame_id = self.scene.frame_id
            zone_marker.header.stamp = self.get_clock().now().to_msg()
            zone_marker.ns = 'target_zones'
            zone_marker.id = marker_index
            zone_marker.type = Marker.CUBE
            zone_marker.action = Marker.ADD
            zone_marker.pose = _pose(zone)
            zone_marker.scale.x, zone_marker.scale.y, zone_marker.scale.z = zone.size
            zone_marker.color = ColorRGBA(
                r=zone.color[0],
                g=zone.color[1],
                b=zone.color[2],
                a=zone.color[3],
            )
            self.marker_publisher.publish(zone_marker)

            label = Marker()
            label.header = zone_marker.header
            label.ns = 'target_zone_labels'
            label.id = marker_index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = _pose(zone)
            label.pose.position.z += 0.055
            label.scale.z = 0.025
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            label.text = zone.name
            self.marker_publisher.publish(label)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    exit_code = 1
    try:
        node = SceneManager()
        if node.apply():
            exit_code = 0
            rclpy.spin(node)
    except KeyboardInterrupt:
        exit_code = 0
    except (OSError, ValueError, yaml.YAMLError) as error:
        if node is not None:
            node.get_logger().fatal(f'Invalid scene configuration: {error}')
        else:
            print(f'Invalid scene configuration: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code != 0:
        raise SystemExit(exit_code)
