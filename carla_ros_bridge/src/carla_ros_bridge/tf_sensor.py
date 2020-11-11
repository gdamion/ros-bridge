#!/usr/bin/env python
#
# Copyright (c) 2020 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
handle a tf sensor
"""

import rospy

from carla_ros_bridge.pseudo_actor import PseudoActor

from tf2_msgs.msg import TFMessage


class TFSensor(PseudoActor):

    """
    Pseudo tf sensor
    """

    def __init__(self, name, parent, node):
        """
        Constructor
        :param name: name identiying the sensor
        :type name: string
        :param carla_world: carla world object
        :type carla_world: carla.World
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param node: node-handle
        :type node: carla_ros_bridge.CarlaRosBridge
        """

        super(TFSensor, self).__init__(parent=parent, node=node, prefix=None)

        self.tf_publisher = rospy.Publisher('tf', TFMessage, queue_size=100)

    def update(self, frame, timestamp):
        """
        Function (override) to update this object.
        """
        tf_to_publish = [self.parent.get_ros_transform()]
        tf_msg = TFMessage(tf_to_publish)
        self.tf_publisher.publish(tf_msg)
