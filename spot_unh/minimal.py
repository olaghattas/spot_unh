import argparse
import logging
from typing import Optional

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

class Minimal:
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

    def initialize_robot(self) -> bool:
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
        return True


def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> int:
    # main passes to  ROSAwareProcess along wth cli in ros_process.main
    minimal = Minimal(args.robot, main.node)
    minimal.initialize_robot()
    return 0


if __name__ == "__main__":
    exit(main())