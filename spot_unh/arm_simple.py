import argparse
from typing import Optional

import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.tf_listener_wrapper import TFListenerWrapper
from bdai_ros2_wrappers.utilities import namespace_with
from bosdyn.api import geometry_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert

from spot_msgs.action import RobotCommand  # type: ignore

from .simple_spot_commander import SimpleSpotCommander


def hello_arm(robot_name: Optional[str] = None) -> bool:



    # Make the open gripper RobotCommand
    gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)

    # Combine the arm and gripper commands into one RobotCommand
    command = RobotCommandBuilder.build_synchro_command(gripper_command, arm_command)

    # Convert to a ROS message
    action_goal = RobotCommand.Goal()
    convert(command, action_goal.command)
    # Send the request and wait until the arm arrives at the goal
    logger.info("Moving arm to position 1.")
    robot_command_client.send_goal_and_wait("arm_move_one", action_goal)

    # Move the arm to a different position
    hand_ewrt_flat_body.z = 0

    flat_body_Q_hand.w = 0.707
    flat_body_Q_hand.x = 0.707
    flat_body_Q_hand.y = 0
    flat_body_Q_hand.z = 0

    flat_body_T_hand2 = geometry_pb2.SE3Pose(position=hand_ewrt_flat_body, rotation=flat_body_Q_hand)
    odom_T_hand = odom_T_flat_body_se3 * math_helpers.SE3Pose.from_obj(flat_body_T_hand2)

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

    # Close the gripper
    gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(0.0)

    # Build the proto
    command = RobotCommandBuilder.build_synchro_command(gripper_command, arm_command)

    # Convert to a ROS message
    action_goal = RobotCommand.Goal()
    convert(command, action_goal.command)
    # Send the request and wait until the arm arrives at the goal
    logger.info("Moving arm to position 2.")
    robot_command_client.send_goal_and_wait("arm_move_two", action_goal)

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
