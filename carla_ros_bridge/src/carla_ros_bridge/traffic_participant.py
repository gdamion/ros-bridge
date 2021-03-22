#!/usr/bin/env python

#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla traffic participants
"""

from std_msgs.msg import ColorRGBA
from carla_ros_bridge.actor import Actor
import carla_common.transforms as trans

from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, AccelWithCovariance
from autoware_auto_msgs.msg import TrackedDynamicObject, TrackedDynamicObjectArray, ObjectTrackedState

class TrafficParticipant(Actor):

    """
    actor implementation details for traffic participant
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
        self.classification_age = 0
        super(TrafficParticipant, self).__init__(uid=uid,
                                                 name=name,
                                                 parent=parent,
                                                 node=node,
                                                 carla_actor=carla_actor)

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.

        On update vehicles send:
        - tf global frame
        - object message
        - marker message

        :return:
        """
        self.classification_age += 1
        super(TrafficParticipant, self).update(frame, timestamp)

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
        twist = TwistWithCovariance()
        twist.twist = self.get_current_ros_twist()
        tracked_state.twist = twist

            # Acceleration
        accel = AccelWithCovariance()
        accel.accel = self.get_current_ros_accel()
        tracked_state.acceleration = accel

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
