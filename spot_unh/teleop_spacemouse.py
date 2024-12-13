import argparse
import logging
from typing import Optional
import time
import bdai_ros2_wrappers.process as ros_process
import bdai_ros2_wrappers.scope as ros_scope
from bdai_ros2_wrappers.action_client import ActionClientWrapper
from bdai_ros2_wrappers.tf_listener_wrapper import TFListenerWrapper
from bdai_ros2_wrappers.utilities import fqn, namespace_with
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME
from bosdyn.client.math_helpers import Quat, SE2Pose, SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from rclpy.node import Node

from spot_msgs.action import RobotCommand  # type: ignore

from .simple_spot_commander import SimpleSpotCommander
from devices.spacemouse import SpaceMouse


###
# TODO: CHANGE LOGIC ON PRESS THINGS SHOULD CHANGE
## RN WHEN left or right are pressed and held the thing holds
class Teleop:
    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None) -> None:
        self._logger = logging.getLogger(fqn(self.__class__))
        node = node or ros_scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use bdai_ros2_wrapper.process.main?)")
        self._robot_name = robot_name

        self._body_frame_name = namespace_with(self._robot_name, BODY_FRAME_NAME)
        self._vision_frame_name = namespace_with(self._robot_name, VISION_FRAME_NAME)

        self._tf_listener = TFListenerWrapper(node)
        self._tf_listener.wait_for_a_tform_b(self._body_frame_name, self._vision_frame_name)
        self._robot = SimpleSpotCommander(self._robot_name, node)

        self._robot_command_client = ActionClientWrapper(
            RobotCommand, namespace_with(self._robot_name, "robot_command"), node
        )

        self.space_mouse = SpaceMouse()
        ## run to start
        self.space_mouse.start_control()

        self.timer = node.create_timer(0.50, self.timer_callback)
        self.prev_arm_stow = True
        ## to do only chnage gripper when changed
        #TODO: figure how do you want to signal open and close
        self.previous_single_click_and_hold = False

    def timer_callback(self):
        # This function is called every 5 seconds
        self._logger.info("Timer callback executed.")

        if (self.space_mouse.single_click_and_hold):
            gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(0.0)
        else:
            gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)

        # Convert to a ROS message
        action_goal = RobotCommand.Goal()
        convert(gripper_command, action_goal.command)
        # Send the request and wait until the arm arrives at the goal
        self._logger.info("Moving arm to position 1.")
        self._robot_command_client.send_goal_and_wait("arm_move_one", action_goal)

        if (self.space_mouse.stow != self.prev_arm_stow):
            ## we need to change from stow to unstow or vice verse
            if (self.space_mouse.stow):
                self._logger.info("stowing robot arm")
                result = self._robot.command("arm_stow")
                if not result.success:
                    self._logger.error("Unable to stow robot " + result.message)
            else:
                self._logger.info("unstowing robot arm")
                result = self._robot.command("arm_unstow")
                if not result.success:
                    self._logger.error("Unable to unstow robot " + result.message)
            self.prev_arm_stow = not self.prev_arm_stow




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

        self.move_arm()
        return True

    def move_arm(self):
        gripper_command = RobotCommandBuilder.claw_gripper_open_fraction_command(1.0)
        # Convert to a ROS message
        action_goal = RobotCommand.Goal()
        convert(gripper_command, action_goal.command)
        # Send the request and wait until the arm arrives at the goal
        self._logger.info("Moving arm to position 1.")
        self._robot_command_client.send_goal_and_wait("arm_move_one", action_goal)


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