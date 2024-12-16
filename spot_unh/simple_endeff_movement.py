import argparse
from typing import Optional

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.tf_listener_wrapper import TFListenerWrapper
from bdai_ros2_wrappers.utilities import namespace_with
from bosdyn.api import geometry_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, HAND_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert

from spot_msgs.action import RobotCommand  # type: ignore

from .simple_spot_commander import SimpleSpotCommander


def hello_arm(robot_name: Optional[str] = None) -> bool:
    # Set up basic ROS2 utilities for communicating with the driver
    node = ros_scope.node()
    if node is None:
        raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
    logger = node.get_logger()

    odom_frame_name = namespace_with(robot_name, ODOM_FRAME_NAME)
    grav_aligned_body_frame_name = namespace_with(robot_name, GRAV_ALIGNED_BODY_FRAME_NAME)
    tf_listener = TFListenerWrapper(node)
    tf_listener.wait_for_a_tform_b(odom_frame_name, grav_aligned_body_frame_name)

    robot = SimpleSpotCommander(robot_name, node)
    robot_command_client = ActionClientWrapper(RobotCommand, namespace_with(robot_name, "robot_command"), node)

    # Claim robot
    logger.info("Claiming robot")
    result = robot.command("claim")
    if not result.success:
        node.get_logger().info("Unable to claim robot message was " + result.message)
        # return False
    logger.info("Claimed robot")

    # Stand the robot up.
    logger.info("Powering robot on")
    result = robot.command("power_on")
    if not result.success:
        logger.info("Unable to power on robot message was " + result.message)
        # return False
    logger.info("Standing robot up")
    result = robot.command("stand")
    if not result.success:
        logger.info("Robot did not stand message was " + result.message)
        # return False
    logger.info("Successfully stood up.")


    # # Fetch the current transform from the flat body frame to the hand frame
    hand_pose_in_body = tf_listener.lookup_a_tform_b(GRAV_ALIGNED_BODY_FRAME_NAME, HAND_FRAME_NAME)

    # Add displacement to move the hand to the left
    displacement = geometry_pb2.Vec3(x=0.0, y=0.1, z=-0.05)  # 10 cm to the left
    displaced_position = math_helpers.Vec3(
        hand_pose_in_body.transform.translation.x + displacement.x,
        hand_pose_in_body.transform.translation.y + displacement.y,
        hand_pose_in_body.transform.translation.z + displacement.z
    )
    displaced_orientation = math_helpers.Quat(
        w=hand_pose_in_body.transform.rotation.w,
        x=hand_pose_in_body.transform.rotation.x,
        y=hand_pose_in_body.transform.rotation.y,
        z=hand_pose_in_body.transform.rotation.z,
    )

    target_hand_pose_in_body = math_helpers.SE3Pose(
        x=displaced_position.x,
        y=displaced_position.y,
        z=displaced_position.z,
        rot=displaced_orientation
    )

    # duration in seconds
    seconds = 2

    arm_command = RobotCommandBuilder.arm_pose_command(
        target_hand_pose_in_body.x,
        target_hand_pose_in_body.y,
        target_hand_pose_in_body.z,
        target_hand_pose_in_body.rot.w,
        target_hand_pose_in_body.rot.x,
        target_hand_pose_in_body.rot.y,
        target_hand_pose_in_body.rot.z,
        GRAV_ALIGNED_BODY_FRAME_NAME,  # Frame of reference
        seconds  # Duration to reach the position
    )


    # Make the open gripper RobotCommand

    # Combine the arm and gripper commands into one RobotCommand
    command = RobotCommandBuilder.build_synchro_command(arm_command)

    # Convert to a ROS message
    action_goal = RobotCommand.Goal()
    convert(command, action_goal.command)
    # Send the request and wait until the arm arrives at the goal
    logger.info("Moving arm to position 1.")
    robot_command_client.send_goal_and_wait("arm_move_one", action_goal)

    return True


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> None:
    hello_arm(args.robot)


if __name__ == "__main__":
    main()
