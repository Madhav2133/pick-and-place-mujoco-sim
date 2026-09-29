# mujoco_bridge.py
import threading, time
import numpy as np
import mujoco, mujoco.viewer
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory, GripperCommand
from world import build_scene, HOME

MJ_ARM = [f"joint{i}" for i in range(1, 8)]
ROS_ARM = [f"panda_joint{i}" for i in range(1, 8)]
MJ_FING = ["finger_joint1", "finger_joint2"]
ROS_FING = ["panda_finger_joint1", "panda_finger_joint2"]


class MujocoBridge(Node):
    def __init__(self, model, data):
        super().__init__("mujoco_bridge")
        self.m, self.d = model, data
        self.lock = threading.Lock()
        cbg = ReentrantCallbackGroup()

        self.arm_qadr = [model.joint(n).qposadr[0] for n in MJ_ARM]
        self.fing_qadr = [model.joint(n).qposadr[0] for n in MJ_FING]
        self.arm_act = [model.actuator(f"actuator{i}").id for i in range(1, 8)]
        self.grip_act = model.actuator("actuator8").id

        # start at home, and hold it
        for a, q in zip(self.arm_qadr, HOME):
            data.qpos[a] = q
        self.arm_target = np.array(HOME, dtype=float)
        self.grip_target = 255.0            # 255 = open, 0 = closed
        self.traj = None                    # (t0, times, positions)
        mujoco.mj_forward(model, data)

        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(0.01, self.publish_state)

        ActionServer(self, FollowJointTrajectory,
                     "/panda_arm_controller/follow_joint_trajectory",
                     self.exec_arm, callback_group=cbg)
        ActionServer(self, GripperCommand,
                     "/panda_hand_controller/gripper_cmd",
                     self.exec_gripper, callback_group=cbg)

    # ---- ROS side ----
    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            q = [self.d.qpos[a] for a in self.arm_qadr + self.fing_qadr]
        msg.name = ROS_ARM + ROS_FING
        msg.position = [float(x) for x in q]
        self.pub.publish(msg)

    def exec_arm(self, gh):
        traj = gh.request.trajectory
        names = list(traj.joint_names)
        order = [names.index(n) for n in ROS_ARM]
        times = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
                          for p in traj.points])
        pos = np.array([[p.positions[i] for i in order] for p in traj.points])
        with self.lock:
            self.traj = (self.d.time, times, pos)
        while True:
            with self.lock:
                if self.traj is None:
                    break
            time.sleep(0.01)
        gh.succeed()
        return FollowJointTrajectory.Result()

    def exec_gripper(self, gh):
        width = float(gh.request.command.position)      # per finger, 0..0.04 m
        with self.lock:
            self.grip_target = float(np.clip(width / 0.04, 0, 1) * 255)
        time.sleep(1.0)                                  # let the fingers move
        res = GripperCommand.Result()
        res.position, res.reached_goal = width, True
        gh.succeed()
        return res

    # ---- sim side ----
    def apply_ctrl(self):
        if self.traj is not None:
            t0, times, pos = self.traj
            t = self.d.time - t0
            self.arm_target = np.array(
                [np.interp(t, times, pos[:, j]) for j in range(7)])
            if t >= times[-1]:
                self.traj = None                          # hold last point
        for a, q in zip(self.arm_act, self.arm_target):
            self.d.ctrl[a] = q
        self.d.ctrl[self.grip_act] = self.grip_target

    def sim_loop(self):
        with mujoco.viewer.launch_passive(self.m, self.d) as v:
            wall0, sim0 = time.time(), self.d.time
            while v.is_running() and rclpy.ok():
                target = sim0 + (time.time() - wall0)
                if target - self.d.time > 0.2:        # fell far behind, don't spiral
                    wall0, sim0 = time.time(), self.d.time
                    target = sim0
                with self.lock:
                    while self.d.time < target:
                        self.apply_ctrl()
                        mujoco.mj_step(self.m, self.d)
                v.sync()
                time.sleep(0.005)


def main():
    rclpy.init()
    model, data = build_scene()
    node = MujocoBridge(model, data)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    node.sim_loop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()