import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint, BoundingVolume,
    MotionPlanRequest, PlanningOptions, CollisionObject, MoveItErrorCodes,
)

FRAME = "panda_link0"
TIP = "panda_link8"          # tip link of the panda_arm group
GROUP = "panda_arm"
TCP_OFFSET = 0.1034          # link8 origin to fingertip centre
CUBE_HALF = 0.02
TABLE_H = 0.30
CUBE_Z = TABLE_H + CUBE_HALF  # cube centre height

CUBE_XY = (0.50, -0.15)      # must match the MJCF
TARGET_XY = (0.50, 0.20)

GRASP_Z = CUBE_Z + TCP_OFFSET + 0.005   # link8 height when gripping
PLACE_Z = CUBE_Z + TCP_OFFSET + 0.015   # slightly higher, so the cube is released just above the table
HOVER = 0.12                             # extra height for approach and retreat


def make_pose(x, y, z, yaw=0.0):
    """Tool pointing straight down, fingers opening along world y when yaw=0.

    The hand is mounted -45 deg about z relative to link8, so link8 needs
    a yaw of -pi/4 to compensate. Orientation = Rz(yaw) * Rx(pi).
    """
    a = yaw - math.pi / 4
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x = math.cos(a / 2)
    p.orientation.y = math.sin(a / 2)
    p.orientation.z = 0.0
    p.orientation.w = 0.0
    return p


class PickPlace(Node):
    def __init__(self):
        super().__init__("pick_place")
        self.move_ac = ActionClient(self, MoveGroup, "/move_action")
        self.exec_ac = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.grip_ac = ActionClient(self, GripperCommand, "/panda_hand_controller/gripper_cmd")
        self.cart_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.obj_pub = self.create_publisher(CollisionObject, "/collision_object", 10)

        for ac in (self.move_ac, self.exec_ac, self.grip_ac):
            ac.wait_for_server()
        self.cart_cli.wait_for_service()

    # ---------- helpers ----------
    def _send(self, client, goal):
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            raise RuntimeError("goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        return rf.result().result

    def remove_cube_from_scene(self):
        """The planner must not treat the cube as an obstacle while grasping it."""
        o = CollisionObject()
        o.id = "cube"
        o.header.frame_id = FRAME
        o.operation = CollisionObject.REMOVE
        time.sleep(1.0)  # let the publisher connect
        self.obj_pub.publish(o)
        time.sleep(0.5)

    # ---------- motion ----------
    def gripper(self, width, effort=40.0):
        """width is per finger in metres: 0.04 open, 0.0 closed."""
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(effort)
        self._send(self.grip_ac, g)

    def move_to(self, pose, vel=0.3, acc=0.3):
        pc = PositionConstraint()
        pc.header.frame_id = FRAME
        pc.link_name = TIP
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.001])
        pc.constraint_region = BoundingVolume(primitives=[sphere], primitive_poses=[pose])
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = FRAME
        oc.link_name = TIP
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.02
        oc.absolute_y_axis_tolerance = 0.02
        oc.absolute_z_axis_tolerance = 0.02
        oc.weight = 1.0

        req = MotionPlanRequest()
        req.group_name = GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = vel
        req.max_acceleration_scaling_factor = acc
        req.goal_constraints = [Constraints(position_constraints=[pc],
                                            orientation_constraints=[oc])]

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions(plan_only=False)
        res = self._send(self.move_ac, goal)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"move_to failed, error code {res.error_code.val}")
        time.sleep(0.5)  # let joint_states settle

    def line_to(self, pose, slow=3.0):
        """Straight-line Cartesian move. `slow` stretches the trajectory timing."""
        req = GetCartesianPath.Request()
        req.header.frame_id = FRAME
        req.group_name = GROUP
        req.link_name = TIP
        req.waypoints = [pose]
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = False   # fingers end up close to the table by design
        fut = self.cart_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        res = fut.result()
        if res.fraction < 0.99:
            raise RuntimeError(f"cartesian path only {res.fraction:.0%} complete")

        traj = res.solution
        for p in traj.joint_trajectory.points:
            t = (p.time_from_start.sec + p.time_from_start.nanosec * 1e-9) * slow
            p.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            p.velocities = []
            p.accelerations = []

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = traj
        res = self._send(self.exec_ac, goal)
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"execute failed, error code {res.error_code.val}")
        time.sleep(0.5)

    # ---------- task ----------
    def run(self, cube_xy, target_xy, cube_yaw=0.0):
        cx, cy = cube_xy
        tx, ty = target_xy

        self.remove_cube_from_scene()
        self.gripper(0.04)

        self.get_logger().info("approach cube")
        self.move_to(make_pose(cx, cy, GRASP_Z + HOVER, cube_yaw))
        self.line_to(make_pose(cx, cy, GRASP_Z, cube_yaw))

        self.get_logger().info("grasp")
        self.gripper(0.0)

        self.get_logger().info("lift and transport")
        self.line_to(make_pose(cx, cy, GRASP_Z + HOVER, cube_yaw))
        self.move_to(make_pose(tx, ty, PLACE_Z + HOVER, 0.0))

        self.get_logger().info("place")
        self.line_to(make_pose(tx, ty, PLACE_Z, 0.0))
        self.gripper(0.04)
        self.line_to(make_pose(tx, ty, PLACE_Z + HOVER, 0.0))

        self.get_logger().info("done")


def main():
    rclpy.init()
    node = PickPlace()
    try:
        node.run(CUBE_XY, TARGET_XY)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()