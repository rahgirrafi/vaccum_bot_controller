

from enum import IntEnum
import math
from threading import Thread


from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default


class VaccumBotDirections(IntEnum):
    NORTH = 0
    NORTH_WEST = 45
    WEST = 90
    SOUTH_WEST = 135
    SOUTH = 180
    SOUTH_EAST = 225
    EAST = 270
    NORTH_EAST = 315


class VaccumBotNavigator(BasicNavigator):
    creating_path = False

    def __init__(self, namespace=''):
        super().__init__(namespace=namespace)

        self.create_subscription(PoseWithCovarianceStamped,
                                 'initialpose',
                                 self._poseEstimateCallback,
                                 qos_profile_system_default)

    def getPoseStamped(self, position, rotation):
        """
        Fill and return a PoseStamped message.

        :param position: A list consisting of the x and y positions for the Pose. e.g [0.5, 1.2]
        :param rotation: Rotation of the pose about the Z axis in degrees.
        :return: PoseStamped message
        """
        pose = PoseStamped()

        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]

        # Convert Z rotation to quaternion
        pose.pose.orientation.z = math.sin(math.radians(rotation) / 2)
        pose.pose.orientation.w = math.cos(math.radians(rotation) / 2)

        return pose

    def stampPose(self, pose):
        """
        Stamp a Pose message and return a PoseStamped message.

        :param pose: Pose message
        :return: PoseStamped message
        """
        poseStamped = PoseStamped()

        poseStamped.header.frame_id = 'map'
        poseStamped.header.stamp = self.get_clock().now().to_msg()

        poseStamped.pose = pose

        return poseStamped

    def createPath(self):
        """
        Create a path using the '2D Pose Estimate' tool in Rviz.

        :return: List of PoseStamped poses
        """
        poses = []
        self.new_pose = None
        self.creating_path = True

        self.info('Creating a path. Press Enter to finish.')
        self.info('Use the "2D Pose Estimate" tool in Rviz to add a pose to the path.')

        def wait_for_key():
            input()

        input_thread = Thread(target=wait_for_key, daemon=True)
        input_thread.start()

        while self.creating_path:
            while self.new_pose is None:
                if input_thread.is_alive():
                    rclpy.spin_once(self, timeout_sec=0.1)
                else:
                    self.creating_path = False
                    break
            if self.new_pose:
                self.info('Pose added.')
                poses.append(self.stampPose(self.new_pose))
                self.new_pose = None
                self.clearAllCostmaps()
        if len(poses) > 0:
            self.info('Path created.')
            for i, p in enumerate(poses):
                self.info('Pose {0} [x,y]=[{1:.3f},{2:.3f}]'.format(
                    i, p.pose.position.x, p.pose.position.y) +
                    '[x,y,z,w]=[{0:.3f},{1:.3f},{2:.3f},{3:.3f}]'.format(
                    p.pose.orientation.x, p.pose.orientation.y,
                    p.pose.orientation.z, p.pose.orientation.w))
        return poses

    # 2D Pose Estimate callback
    def _poseEstimateCallback(self, msg: PoseWithCovarianceStamped):
        if self.creating_path:
            self.new_pose = msg.pose.pose


    def startToPose(self, pose: PoseStamped):
        """
        Perform goToPose action and print feedback.

        :param pose: Goal pose.
        """
        i = 0
        self.goToPose(pose)

        while not self.isTaskComplete():
            feedback = self.getFeedback()
            i = i + 1
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + '{0: <20}'.format('seconds.'), end='\r')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.cancelTask()

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.info('Goal failed!')
        else:
            self.info('Goal has an invalid return status!')

    def startThroughPoses(self, poses):
        """
        Perform goThroughPoses action and print feedback.

        :param poses: List of goal poses.
        """
        i = 0
        self.goThroughPoses(poses)

        while not self.isTaskComplete():
            feedback = self.getFeedback()
            i = i + 1
            if feedback and i % 5 == 0:
                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                    + '{0: <20}'.format(' seconds.'), end='\r')

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.cancelTask()

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.info('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.info('Goal failed!')
        else:
            self.info('Goal has an invalid return status!')

    def startFollowWaypoints(self, poses):
        """
        Perform followWaypoint action and print feedback.

        :param poses: List of goal poses.
        """
        i = 0
        self.followWaypoints(poses)

        while not self.isTaskComplete():
            i = i + 1
            feedback = self.getFeedback()
            if feedback and i % 5 == 0:
                print('Executing current waypoint: {0}/{1: <5}'.format(
                    str(feedback.current_waypoint + 1), str(len(poses))), end='\r')
