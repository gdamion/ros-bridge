#!/usr/bin/env python

#
# Copyright (c) 2018-2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla traffic objects
"""

from carla_ros_bridge.actor import Actor
import carla_common.transforms as trans
from std_msgs.msg import ColorRGBA
from carla_msgs.msg import CarlaTrafficLightStatus, CarlaTrafficLightInfo, CarlaObjectKamazInfo
from carla import TrafficLightState
from shape_msgs.msg import SolidPrimitive


class Traffic(Actor):

    """
    Actor implementation details for traffic objects
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

        super(Traffic, self).__init__(uid=uid,
                                      name=name,
                                      parent=parent,
                                      node=node,
                                      carla_actor=carla_actor)

    def get_marker_color(self):  # pylint: disable=no-self-use
        """
        Function (override) to return the color for marker messages.

        :return: default color used by traffic participants
        :rtpye : std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 255.
        color.g = 230.
        color.b = 0.
        return color

    def get_classification(self):  # pylint: disable=no-self-use
        """
        Function to get object classification (overridden in subclasses)
        """
        return CarlaObjectKamazInfo.CLASSIFICATION_SIGN

    def get_status(self):
        type = self.carla_actor.type_id
        status = type.rpartition('.')[-1]
        if status == "90":
            return CarlaObjectKamazInfo.STATUS_LIMIT90
        elif status == "60":
            return CarlaObjectKamazInfo.STATUS_LIMIT60
        elif status == "40":
            return CarlaObjectKamazInfo.STATUS_LIMIT40
        elif status == "stop":
            return CarlaObjectKamazInfo.STATUS_STOP
        else:
            return CarlaObjectKamazInfo.STATUS_UNKNOWN

class TrafficLight(Actor):

    """
    Traffic implementation details for traffic lights
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
        :type carla_actor: carla.TrafficLight
        """
        super(TrafficLight, self).__init__(uid=uid,
                                           name=name,
                                           parent=parent,
                                           node=node,
                                           carla_actor=carla_actor)

    def get_info(self):
        """
        Get the info of the traffic light
        """
        info = CarlaTrafficLightInfo()
        info.id = self.get_id()
        info.transform = self.get_current_ros_pose()
        info.trigger_volume.center = trans.carla_location_to_ros_vector3(
            self.carla_actor.trigger_volume.location)
        info.trigger_volume.size.x = self.carla_actor.trigger_volume.extent.x * 2.0
        info.trigger_volume.size.y = self.carla_actor.trigger_volume.extent.y * 2.0
        info.trigger_volume.size.z = self.carla_actor.trigger_volume.extent.z * 2.0
        return info

    def get_marker_color(self):  # pylint: disable=no-self-use
        """
        Function (override) to return the color for marker messages.
        :return: default color used by traffic participants
        :rtpye : std_msgs.msg.ColorRGBA
        """
        color = ColorRGBA()
        color.r = 255.
        color.g = 0.
        color.b = 0.
        return color

    def get_classification(self):  # pylint: disable=no-self-use
        """
        Function to get object classification (overridden in subclasses)
        """
        return CarlaObjectKamazInfo.CLASSIFICATION_LIGHT

    def get_status(self):
        """
        Get the current state of the traffic light
        """
        status = CarlaTrafficLightStatus()
        status.id = self.get_id()
        carla_state = self.carla_actor.get_state()
        if carla_state == TrafficLightState.Red:
            status.state = CarlaTrafficLightStatus.RED
        elif carla_state == TrafficLightState.Yellow:
            status.state = CarlaTrafficLightStatus.YELLOW
        elif carla_state == TrafficLightState.Green:
            status.state = CarlaTrafficLightStatus.GREEN
        elif carla_state == TrafficLightState.Off:
            status.state = CarlaTrafficLightStatus.OFF
        else:
            status.state = CarlaTrafficLightStatus.UNKNOWN
        return status

    def get_object_info(self):
        """
        Function to send object messages of this traffic participant.
        A derived_object_msgs.msg.Object is prepared to be published via '/carla/objects'
        :return:
        """
        obj = CarlaObjectKamazInfo(header=self.get_msg_header("map"))
        obj.id = self.get_id()

        try:
            obj.rolename = str(self.carla_actor.attributes.get('role_name'))
        except ValueError:
            pass

        obj.type = self.carla_actor.type_id
        obj.pose = self.get_current_ros_pose()
        obj.shape.type = SolidPrimitive.BOX
        obj.shape.dimensions.extend([
            self.carla_actor.bounding_box.extent.x * 2.0,
            self.carla_actor.bounding_box.extent.y * 2.0,
            self.carla_actor.bounding_box.extent.z * 2.0])
        obj.classification = self.get_classification()

        status = self.get_status()
        if status.state == CarlaTrafficLightStatus.RED:
            obj.status = CarlaObjectKamazInfo.STATUS_RED_LIGHT
        elif status.state == CarlaTrafficLightStatus.YELLOW:
            obj.status = CarlaObjectKamazInfo.STATUS_YELLOW_LIGHT
        elif status.state == CarlaTrafficLightStatus.GREEN:
            obj.status = CarlaObjectKamazInfo.STATUS_GREEN_LIGHT
        elif status.state == CarlaTrafficLightStatus.OFF:
            obj.status = CarlaObjectKamazInfo.STATUS_LIGHT_OFF
        else:
            obj.status = CarlaObjectKamazInfo.STATUS_UNKNOWN

        return obj
