"""Apply the same boxes that were used to generate the Gazebo world."""
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

from .scene import load_objects


def main(args=None):
    rclpy.init(args=args)
    node = Node('scene_loader')
    node.declare_parameter('objects_file', '')
    node.declare_parameter('timeout', 60.0)
    success = False
    try:
        objects = load_objects(node.get_parameter('objects_file').value)
        client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
        deadline = time.monotonic() + node.get_parameter('timeout').value
        while rclpy.ok() and time.monotonic() < deadline and not client.wait_for_service(timeout_sec=0.2):
            pass
        if not client.service_is_ready():
            raise RuntimeError('MoveIt apply_planning_scene service unavailable')
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        for item in objects:
            obj = CollisionObject()
            obj.id = item['id']
            obj.header.frame_id = 'world'
            obj.operation = CollisionObject.ADD
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = item['size']
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = item['position']
            pose.orientation.w = 1.0
            obj.primitives = [box]
            obj.primitive_poses = [pose]
            request.scene.world.collision_objects.append(obj)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=max(0.0, deadline - time.monotonic()))
        success = future.done() and future.result() is not None and future.result().success
        if not success:
            raise RuntimeError('MoveIt did not acknowledge the planning scene')
        node.get_logger().info('PlanningScene applied: ' + ', '.join(o['id'] for o in objects))
    except Exception as exc:
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if success else 1
