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

from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from carla_ros_bridge.actor import Actor
import carla_common.transforms as trans
from kamaz_msgs.msg import CarlaObject
from shape_msgs.msg import SolidPrimitive

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
        A CarlaObject is prepared to be published via '/carla/objects'
        :return:
        """
        obj = CarlaObject(header=self.get_msg_header("map"))
        obj.id = self.get_id()

        try:
            obj.rolename = str(self.carla_actor.attributes.get('role_name'))
        except ValueError:
            pass

        obj.type = self.carla_actor.type_id
        obj.pose = self.get_current_ros_pose()
        obj.vel = self.get_current_ros_twist()
        obj.acc = self.get_current_ros_accel()
        obj.shape.type = SolidPrimitive.BOX
        obj.shape.dimensions.extend([
            self.carla_actor.bounding_box.extent.x * 2.0,
            self.carla_actor.bounding_box.extent.y * 2.0,
            self.carla_actor.bounding_box.extent.z * 2.0])
        obj.classification = self.get_classification()
        obj.status = self.get_status()
        return obj
