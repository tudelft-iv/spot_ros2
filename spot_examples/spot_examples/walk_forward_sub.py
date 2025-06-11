import argparse
import logging
from typing import Optional
import rclpy
import time
import synchros2.process as ros_process
import synchros2.scope as ros_scope
from bosdyn.client.frame_helpers import BODY_FRAME_NAME, VISION_FRAME_NAME
from bosdyn.client.math_helpers import Quat, SE2Pose, SE3Pose
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn_msgs.conversions import convert
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

from synchros2.action_client import ActionClientWrapper
from synchros2.tf_listener_wrapper import TFListenerWrapper
from synchros2.utilities import fqn, namespace_with

from spot_msgs.action import RobotCommand  # type: ignore

from .simple_spot_commander import SimpleSpotCommander

ROBOT_T_GOAL = SE2Pose(5.0, 0.0, 0.0)
VELOCITY_CMD_DURATION  = 10.0
VELOCITY_X = 0.6
VELOCITY_Y = 0.0
VELOCITY_ROT = 0.0


class WalkForward:
    def __init__(self, robot_name: Optional[str] = None, node: Optional[Node] = None) -> None:
        self._logger = logging.getLogger(fqn(self.__class__))
        node = node or ros_scope.node()
        if node is None:
            raise ValueError("no ROS 2 node available (did you use synchros2.process.main?)")
        self._robot_name = robot_name
        self._node = node

        self._body_frame_name = namespace_with(self._robot_name, BODY_FRAME_NAME)
        self._vision_frame_name = namespace_with(self._robot_name, VISION_FRAME_NAME)
        self._tf_listener = TFListenerWrapper(node)
        self._tf_listener.wait_for_a_tform_b(self._body_frame_name, self._vision_frame_name)
        self._robot = SimpleSpotCommander(self._robot_name, node)
        self._robot_command_client = ActionClientWrapper(
            RobotCommand, namespace_with(self._robot_name, "robot_command"), node
        )

        # Subscribe to the /spot_mv topic
        qos_profile = QoSProfile(depth=10)
        self._subscription = self._node.create_subscription(
            Bool, "/spot_mv", self.on_spot_mv, qos_profile
        )
        self.pub_cmd_vel = self._node.create_publisher(Twist, namespace_with(self._robot_name, "cmd_vel"), 1)


    def initialize_robot(self) -> bool:
        self._logger.info(f"Robot name: {self._robot_name}")
        self._logger.info("Claiming robot")
        result = self._robot.command("claim")
        if not result.success:
            self._logger.error("Unable to claim robot message was " + result.message)
            return False
        self._logger.info("Claimed robot")

        self._logger.info("Powering robot on")
        result = self._robot.command("power_on")
        if not result.success:
            self._logger.error("Unable to power on robot message was " + result.message)
            return False
        self._logger.info("Standing robot up")
        result = self._robot.command("stand")
        if not result.success:
            self._logger.error("Robot did not stand message was " + result.message)
            return False
        self._logger.info("Successfully stood up.")
        return True

    def walk_forward_with_world_frame_goal(self) -> None:
        self._logger.info("Walking forward")
        world_t_robot = self._tf_listener.lookup_a_tform_b(self._vision_frame_name, self._body_frame_name)
        world_t_robot_se2 = SE3Pose(
            world_t_robot.transform.translation.x,
            world_t_robot.transform.translation.y,
            world_t_robot.transform.translation.z,
            Quat(
                world_t_robot.transform.rotation.w,
                world_t_robot.transform.rotation.x,
                world_t_robot.transform.rotation.y,
                world_t_robot.transform.rotation.z,
            ),
        ).get_closest_se2_transform()
        world_t_goal = world_t_robot_se2 * ROBOT_T_GOAL
        proto_goal = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=world_t_goal.x,
            goal_y=world_t_goal.y,
            goal_heading=world_t_goal.angle,
            frame_name=VISION_FRAME_NAME,
        )

        action_goal = RobotCommand.Goal()
        convert(proto_goal, action_goal.command)
        self._robot_command_client.send_goal_and_wait("walk_forward", action_goal)
        self._logger.info("Successfully walked forward")

    def on_spot_mv(self, msg: Bool) -> None:
        if msg.data:
            self._logger.info("Received move command on /spot_mv")
            self.move_forward_x_seconds()

    def move_forward_x_seconds(self) -> None:
        self._logger.info(f"Moving forward for {VELOCITY_CMD_DURATION} seconds")
        # Implement the logic to move the robot forward for the specified duration
        twist = Twist()
        twist.linear.x = VELOCITY_X
        twist.linear.y = VELOCITY_Y
        twist.angular.z = VELOCITY_ROT
        start_time = time.time()
        while time.time() - start_time < VELOCITY_CMD_DURATION:
            self.pub_cmd_vel.publish(twist)
            time.sleep(0.05)
        self.pub_cmd_vel.publish(Twist())



def cli() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=str, default=None)
    return parser


@ros_process.main(cli())
def main(args: argparse.Namespace) -> int:
    walker = WalkForward(args.robot, main.node)
    walker.initialize_robot()
    rclpy.spin(main.node)
    main.node.destroy_node()
    rclpy.shutdown()
    main.logger.info("Shutting down")
    return 0


if __name__ == "__main__":
    exit(main())
