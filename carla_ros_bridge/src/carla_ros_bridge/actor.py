#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Base Classes to handle Actor objects
"""

from carla_ros_bridge.pseudo_actor import PseudoActor
import carla_common.transforms as trans

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import TransformStamped # pylint: disable=import-error
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, AccelWithCovariance
from visualization_msgs.msg import Marker
from shape_msgs.msg import SolidPrimitive
from autoware_auto_msgs.msg import TrackedDynamicObject, TrackedDynamicObjectArray, ObjectTrackedState

class Actor(PseudoActor):

    """
    Generic base class for all carla actors
    """

    def __init__(self, uid, name, parent, node, carla_actor):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        """
        super(Actor, self).__init__(uid=uid,
                                    name=name,
                                    parent=parent,
                                    node=node)
        self.carla_actor = carla_actor
        self.carla_actor_id = carla_actor.id

    def destroy(self):
        """
        Function (override) to destroy this object.
        Remove the reference to the carla.Actor object.
        :return:
        """
        self.carla_actor = None
        super(Actor, self).destroy()

    def get_current_ros_pose(self):
        """
        Function to provide the current ROS pose

        :return: the ROS pose of this actor
        :rtype: geometry_msgs.msg.Pose
        """
        return trans.carla_transform_to_ros_pose(
            self.carla_actor.get_transform())

    def get_current_ros_transform(self):
        """
        Function to provide the current ROS pose

        :return: the ROS pose of this actor
        :rtype: geometry_msgs.msg.Pose
        """
        return trans.carla_transform_to_ros_transform(
            self.carla_actor.get_transform())

    def get_current_ros_twist_rotated(self):
        """
        Function to provide the current ROS twist rotated

        :return: the ROS twist of this actor
        :rtype: geometry_msgs.msg.Twist
        """
        return trans.carla_velocity_to_ros_twist(
            self.carla_actor.get_velocity(),
            self.carla_actor.get_angular_velocity(),
            self.carla_actor.get_transform().rotation)

    def get_current_ros_twist(self):
        """
        Function to provide the current ROS twist

        :return: the ROS twist of this actor
        :rtype: geometry_msgs.msg.Twist
        """
        return trans.carla_velocity_to_ros_twist(
            self.carla_actor.get_velocity(),
            self.carla_actor.get_angular_velocity())

    def get_current_ros_accel(self):
        """
        Function to provide the current ROS accel

        :return: the ROS twist of this actor
        :rtype: geometry_msgs.msg.Twist
        """
        return trans.carla_acceleration_to_ros_accel(
            self.carla_actor.get_acceleration())

    def get_id(self):
        """
        Getter for the carla_id of this.
        :return: unique carla_id of this object
        :rtype: int64
        """
        return self.carla_actor_id

    def get_marker_color(self):  # pylint: disable=no-self-use
        """
        Function (override) to return the color for marker messages.

        :return: default color used by traffic participants
        :rtpye : std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 255.
        color.g = 255.
        color.b = 255.
        return color

    def get_marker_pose(self):
        """
        Function to return the pose for traffic participants.

        :return: the pose of the traffic participant.
        :rtype: geometry_msgs.msg.Pose
        """
        return trans.carla_transform_to_ros_pose(self.carla_actor.get_transform())

    def get_marker(self):
        """
        Helper function to create a ROS visualization_msgs.msg.Marker for the actor

        :return:
        visualization_msgs.msg.Marker
        """
        marker = Marker(header=self.get_msg_header(frame_id="map"))
        marker.color = self.get_marker_color()
        marker.color.a = 0.3
        marker.id = self.get_id()
        marker.type = Marker.CUBE

        marker.pose = self.get_marker_pose()
        marker.scale.x = self.carla_actor.bounding_box.extent.x * 2.0
        marker.scale.y = self.carla_actor.bounding_box.extent.y * 2.0
        marker.scale.z = self.carla_actor.bounding_box.extent.z * 2.0
        return marker

    # def get_classification(self):  # pylint: disable=no-self-use
    #     """
    #     Function to get object classification (overridden in subclasses)
    #     """
    #     return TrackedDynamicObject.CLASSIFICATION_UNKNOWN

    # def get_status(self):  # pylint: disable=no-self-use
    #     """
    #     Function to get object classification (overridden in subclasses)
    #     """
    #     return TrackedDynamicObject.STATUS_UNKNOWN

    def get_object_info(self):
        """
        Function to send object messages of this traffic participant.
        A derived_object_msgs.msg.Object is prepared to be published via '/carla/objects'
        :return:
        """
        obj = TrackedDynamicObject(header=self.get_msg_header("map"))
        obj.id = self.get_id()
        try:
            obj.role_name = str(self.carla_actor.attributes.get('role_name'))
        except ValueError:
            pass

        # Classification
            #todo
        # Tracked State
        tracked_state = ObjectTrackedState()

            # Pose
        pose = PoseWithCovariance()
        pose.pose = self.get_current_ros_pose()
        tracked_state.pose = pose
            # Twist
                # for Actor class twist = 0
            # Acceleration
                # for Actor class accel = 0
        obj.tracked_state = tracked_state

        # Shape
        obj.shape.type = SolidPrimitive.BOX
        obj.shape.dimensions.extend([
            self.carla_actor.bounding_box.extent.x * 2.0,
            self.carla_actor.bounding_box.extent.y * 2.0,
            self.carla_actor.bounding_box.extent.z * 2.0])
        # obj.classification = self.get_classification()
        # obj.status = self.get_status()
        return obj
