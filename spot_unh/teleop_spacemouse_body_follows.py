import argparse
import logging
from typing import Optional
import time

from tf_transformations import quaternion_multiply

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.tf_listener_wrapper import TFListenerWrapper
from bdai_ros2_wrappers.utilities import fqn, namespace_with
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME

from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import geometry_msgs.msg


from spot_msgs.action import RobotCommand  # type: ignore

from .simple_spot_commander import SimpleSpotCommander
from devices.spacemouse import SpaceMouse

import numpy as np
from bdai_ros2_wrappers.tf_listener_wrapper import TFListenerWrapper
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, HAND_FRAME_NAME
from bosdyn.api import geometry_pb2
from bosdyn.client import math_helpers



# /status/feedback
# standing: true
# sitting: false
# moving: false

class Teleop:
    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None) -> None:
        self._logger = logging.getLogger(fqn(self.__class__))
        self.node = node or ros_scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
        self._robot_name = robot_name

        self.odom_frame_name = namespace_with(robot_name, ODOM_FRAME_NAME)
        self.hand_frame_name = namespace_with(robot_name, HAND_FRAME_NAME)
        self.grav_aligned_body_frame_name = namespace_with(robot_name, GRAV_ALIGNED_BODY_FRAME_NAME)
        self._tf_listener = TFListenerWrapper(node)
        self._tf_listener.wait_for_a_tform_b(self.odom_frame_name, self.grav_aligned_body_frame_name)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self.node)


        odom_T_flat_body = self._tf_listener.lookup_a_tform_b(self.odom_frame_name, self.grav_aligned_body_frame_name)
        self.odom_T_flat_body_se3 = math_helpers.SE3Pose(
            odom_T_flat_body.transform.translation.x,
            odom_T_flat_body.transform.translation.y,
            odom_T_flat_body.transform.translation.z,
            math_helpers.Quat(
                odom_T_flat_body.transform.rotation.w,
                odom_T_flat_body.transform.rotation.x,
                odom_T_flat_body.transform.rotation.y,
                odom_T_flat_body.transform.rotation.z,
            ),
        )


        self._robot = SimpleSpotCommander(self._robot_name, node)
        self._robot_command_client = ActionClientWrapper(
            RobotCommand, namespace_with(self._robot_name, "robot_command"), node
        )

        self.space_mouse = SpaceMouse()
        ## run to start
        self.space_mouse.start_control()

        self.timer = node.create_timer(0.50, self.timer_callback)
        self.gripper_state = "closed"  # "opened"
        self.stow_state = "stowed"  # "unstowed"

    def is_zero_array(self, arr, tolerance=1e-3):
        return np.all(np.abs(arr) < tolerance)

    def publish_new_pose(self, new_position, new_quaternion,child_frame_id):
        """
        Publish the new hand pose in TF after applying the given displacement.
        Parameters:
            displacement_position: [x, y, z] displacement in meters.
            displacement_rotation: [roll, pitch, yaw] rotation displacement in radians.
        """
        try:
            # Create a new transform message
            new_transform = geometry_msgs.msg.TransformStamped()
            new_transform.header.stamp = self.node.get_clock().now().to_msg()
            new_transform.header.frame_id = self.odom_frame_name
            new_transform.child_frame_id = child_frame_id

            new_transform.transform.translation.x = float(new_position[0])
            new_transform.transform.translation.y = float(new_position[1])
            new_transform.transform.translation.z = float(new_position[2])
            new_transform.transform.rotation.x = float(new_quaternion[0])
            new_transform.transform.rotation.y = float(new_quaternion[1])
            new_transform.transform.rotation.z = float(new_quaternion[2])
            new_transform.transform.rotation.w = float(new_quaternion[3])


            # Publish the new transform
            self.tf_static_broadcaster.sendTransform(new_transform)
            print("Published new pose in TF.")

        except Exception as e:
            print(f"Error publishing new pose: {e}")


    def get_armcommand(self, position, quat):
        quaternion = quat
        flat_body_T_hand = self._tf_listener.lookup_a_tform_b(self.grav_aligned_body_frame_name, self.hand_frame_name )

        x = flat_body_T_hand.transform.translation.x + position[0]
        y = flat_body_T_hand.transform.translation.y + position[1]
        z = flat_body_T_hand.transform.translation.z + position[2]

        hand_ewrt_flat_body = geometry_pb2.Vec3(x=x, y=y, z=z)

        # Rotation as a quaternion
        # qw = flat_body_T_hand.transform.rotation.w + quaternion[0]
        # qx = flat_body_T_hand.transform.rotation.x + quaternion[1]
        # qy = flat_body_T_hand.transform.rotation.y + quaternion[2]
        # qz = flat_body_T_hand.transform.rotation.z + quaternion[3]

        qw, qx, qy, qz = self.quaternion_multiply([flat_body_T_hand.transform.rotation.w, flat_body_T_hand.transform.rotation.x, flat_body_T_hand.transform.rotation.y, flat_body_T_hand.transform.rotation.z],[quaternion[0], quaternion[1], quaternion[2],quaternion[3]])
        # qw, qx, qy, qz = [1,0,0,0]
        flat_body_Q_hand = geometry_pb2.Quaternion(w=qw, x=qx, y=qy, z=qz)

        flat_body_T_hand = geometry_pb2.SE3Pose(position=hand_ewrt_flat_body, rotation=flat_body_Q_hand)

        odom_T_hand = self.odom_T_flat_body_se3 * math_helpers.SE3Pose.from_obj(flat_body_T_hand)
        # self.publish_new_pose( [odom_T_hand.x, odom_T_hand.y, odom_T_hand.z], quat, "hand_odom_tf")

        print("position", position[0], position[1], position[2])
        self.publish_new_pose( [odom_T_hand.x, odom_T_hand.y, odom_T_hand.z], [qw, qx, qy, qz], "new_hand_odom_trial")

        # duration in seconds
        seconds = 1

        arm_command = RobotCommandBuilder.arm_pose_command(
            odom_T_hand.x,
            odom_T_hand.y,
            odom_T_hand.z,
            odom_T_hand.rot.w,
            odom_T_hand.rot.x,
            odom_T_hand.rot.y,
            odom_T_hand.rot.z,
            ODOM_FRAME_NAME,
            seconds,
        )
        return arm_command

    def normalize_quaternion(self, q):
        """Normalize a quaternion to ensure it represents a valid rotation."""
        norm = np.linalg.norm(q)
        return q / norm

    def quaternion_multiply(self, q1, q2):
        """
        Multiply two quaternions q1 and q2.
        Parameters:
            q1, q2: [w, x, y, z] format quaternions.
        Returns:
            Resulting quaternion after multiplication, in [w, x, y, z] format.
        """
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w_new = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x_new = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y_new = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z_new = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return self.normalize_quaternion([w_new, x_new, y_new, z_new])

    def timer_callback(self):
        # This function is called every 5 seconds
        self._logger.info("Timer callback executed.")

        dpos, drotation = self.space_mouse.input2action()

        # dpos = [0 ,0,0]
        # print("drotation", np.size(drotation))
        # print("drotation", drotation)
        quat = math_helpers.Quat.from_matrix(drotation)
        # print("quat", quat)

        # print("dpos", dpos)
        quat= [quat.w, quat.x, quat.y, quat.z]
        # print("quat", quat)

        if not self.is_zero_array(dpos) and not self.is_zero_array(quat):
            print("dpos", dpos)
            print("quat", quat)
            arm_command = self.get_armcommand(dpos, quat)
            # Tell the robot's body to follow the arm
            follow_arm_command = RobotCommandBuilder.follow_arm_command()

            # Combine the arm and mobility commands into one synchronized command.
            command = RobotCommandBuilder.build_synchro_command(follow_arm_command,arm_command)

            action_goal = RobotCommand.Goal()
            convert(command, action_goal.command)
            # Send the request and wait until the arm arrives at the goal
            # self._logger.info("Moving arm to position 1.")
            self._robot_command_client.send_goal_and_wait("arm_move_one", action_goal)

            # @staticmethod
    # def follow_arm_command():
    #     """Command robot's body to follow the arm around.
    #
    #     Args:
    #         params(spot.MobilityParams): Spot specific parameters for mobility commands.
    #     Returns:
    #         RobotCommand, which can be issued to the robot command service.
    #     """
    #     mobility_command = mobility_command_pb2.MobilityCommand.Request(
    #         follow_arm_request=basic_command_pb2.FollowArmCommand.Request())
    #     synchronized_command = synchronized_command_pb2.SynchronizedCommand.Request(
    #         mobility_command=mobility_command)
    #     command = robot_command_pb2.RobotCommand(synchronized_command=synchronized_command)
    #     return command
    #


        if self.space_mouse.open_gripper and self.gripper_state == "closed":
            gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)
            self.gripper_state = "opened"
            # Convert to a ROS message
            action_goal = RobotCommand.Goal()
            convert(gripper_command, action_goal.command)
            # Send the request and wait until the arm arrives at the goal
            self._logger.info("Moving arm to position 1.")
            self._robot_command_client.send_goal_and_wait("arm_move_one", action_goal)

        elif not self.space_mouse.open_gripper and self.gripper_state == "opened":
            gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(0.0)
            self.gripper_state = "closed"
            # Convert to a ROS message
            action_goal = RobotCommand.Goal()
            convert(gripper_command, action_goal.command)
            # Send the request and wait until the arm arrives at the goal
            self._logger.info("Moving arm to position 1.")
            self._robot_command_client.send_goal_and_wait("arm_move_one", action_goal)


        if self.space_mouse.unstow_arm and self.stow_state == "stowed":
            self.stow_state = "unstowed"
            self._logger.info("unstowing robot arm")
            result = self._robot.command("arm_unstow")
            if not result.success:
                self._logger.error("Unable to unstow robot " + result.message)
        elif not self.space_mouse.unstow_arm and self.stow_state == "unstowed":
            self.stow_state = "stowed"
            self._logger.info("stowing robot arm")
            result = self._robot.command("arm_stow")
            if not result.success:
                self._logger.error("Unable to stow robot " + result.message)

    def initialize_robot(self) -> bool:
        """
        Claims the robot and undocks it.

        """
        self._logger.info(f"Robot name: {self._robot_name}")
        self._logger.info("Claiming robot")
        result = self._robot.command("claim")
        if not result.success:
            self._logger.error("Unable to claim robot message was " + result.message)
            return False
        self._logger.info("Claimed robot")

        # Powering up.
        self._logger.info("Powering robot on")
        result = self._robot.command("power_on")
        if not result.success:
            self._logger.error("Unable to power on robot message was " + result.message)
            return False
        self._logger.info("Standing robot up")

        # Undocking.
        result = self._robot.command("undock")
        if not result.success:
            self._logger.error("Robot did not stand message was " + result.message)
            return False
        self._logger.info("Successfully stood up.")

        # should be always stowed at beginning

        self._logger.info("stowing robot arm")
        result = self._robot.command("arm_stow")
        if not result.success:
            self._logger.error("Unable to stow robot " + result.message)

        return True


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser


# @ros_process.main(cli())
# def main(args: argparse.Namespace) -> int:
#     # main passes to  ROSAwareProcess along wth cli in ros_process.main
#     teleop = Teleop(args.robot, main.node)
#     teleop.initialize_robot()
#     return 0

@ros_process.main(cli())
def main(args: argparse.Namespace) -> int:
    # main passes to  ROSAwareProcess along wth cli in ros_process.main
    teleop = Teleop(args.robot, main.node)
    teleop.initialize_robot()
    # Keep the program running until interrupted
    try:
        while True:
            pass
    except KeyboardInterrupt:
        teleop.space_mouse.stop()
        time.sleep(0.02)
        print("Received keyboard interrupt. Shutting down Teleop.")

    return 0  # Return successful exit code


if __name__ == "__main__":
    exit(main())
