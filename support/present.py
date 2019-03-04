#!/usr/bin/env python

# Copyright (c) 2018 Christoph Pilz
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Classes to handle Agent object (player and non-player)
Inspired by carla_ros_bridge Carla 0.8.4
"""

import math
import numpy
import prctl
import rospy
import tf

from math import sin, cos, sqrt, pow

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from visualization_msgs.msg import MarkerArray, Marker

from dbw_mkz_msgs.msg import SteeringReport, WheelSpeedReport
from dbw_mkz_msgs.msg import BrakeReport, GearReport, SteeringReport, ThrottleReport

from common_msgs.msg import DetectedObject
from common_msgs.msg import ObjectList

from timed_event_handler import TimedEventHandler
from .control import InputController
from .singleton import Singleton


class ClockHandler(metaclass=Singleton):
    def __init__(self):
        self.pub_clock = rospy.Publisher('/clock', Clock, queue_size=1)

    def process(self):
        simtime = TimedEventHandler().getCurrentSimTime()
        cur_time = rospy.Time()
        cur_time.set(int(simtime), int(1000000000 * (simtime - int(simtime))))
        clock = Clock(cur_time)
        self.pub_clock.publish(clock)


class SemanticCarlaTags(metaclass=Singleton):
    """
    Conforms to Carla 0.9.1
    https://github.com/carla-simulator/carla/blob/0.9.1/Docs/cameras_and_sensors.md
    """

    def __init__(self):
        self.__semanticCarlaTags = {
            0: None,
            1: "Buildings",
            2: "Fences",
            3: "Other",
            4: "Pedestrians",
            5: "Poles",
            6: "RoadLines",
            7: "Roads",
            8: "Sidewalks",
            9: "Vegetation",
            10: "Vehicles",
            11: "Walls",
            12: "TrafficSigns",

            "None": 0,
            "Buildings": 1,
            "Fences": 2,
            "Other": 3,
            "Pedestrians": 4,
            "Poles": 5,
            "RoadLines": 6,
            "Roads": 7,
            "Sidewalks": 8,
            "Vegetation": 9,
            "Vehicles": 10,
            "Walls": 11,
            "TrafficSigns": 12
        }

    def get(self, elem):
        return self.__semanticCarlaTags[elem]


class MondeoPlayerAgentHandler(metaclass=Singleton):
    """
    Convert player agent into ros message (as marker)
    Convert player agent into vehicle/utm_odom msg
    """

    def __init__(self):
        self.acker_track = rospy.get_param('~track', 1.5824)
        self.acker_wheelbase = rospy.get_param('~wheel_base', 2.8498)
        self.center_axle_offset = rospy.get_param('~center_axle_offset', -1.361)
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.2413)

        # +/- 8.2 SteeringCmd
        # ford mondeo steering ratio 14.8
        self.steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        self.max_steer_angle = rospy.get_param('~max_steer_angle', 8.2)

        self.pub_odom = rospy.Publisher('vehicle/utm_odom', Odometry, queue_size=10)
        self.pub_tf = rospy.Publisher('tf', TFMessage, queue_size=10)
        self.pub_js = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.pub_wsr = rospy.Publisher('/vehicle/wheel_speed_report', WheelSpeedReport, queue_size=10)
        self.pub_br = rospy.Publisher('/vehicle/brake_report', BrakeReport, queue_size=10)
        self.pub_gr = rospy.Publisher('/vehicle/gear_report', GearReport, queue_size=10)
        self.pub_tr = rospy.Publisher('/vehicle/throttle_report', ThrottleReport, queue_size=10)
        self.pub_sr = rospy.Publisher('/vehicle/steering_report', SteeringReport, queue_size=10)

        self.pub_ol = rospy.Publisher('/carla/object_list', ObjectList, queue_size=10)

    def process(self, carla_actor):
        """
        first odom, then tf, then Reports
        """
        # --- --- Get Data --- ---
        old_control = InputController().get_old_control()
        ego_pose = carla_actor.get_transform()
        velocity = carla_actor.get_velocity()
        forward_speed = sqrt(pow(velocity.x, 2.0) + pow(velocity.y, 2.0) + pow(velocity.z, 2.0))
        simtime = TimedEventHandler().getCurrentSimTime()
        cur_time = rospy.Time()
        cur_time.set(int(simtime), int(1000000000 * (simtime - int(simtime))))

        # carla to ros corrections
        # ego_pose.location.x = ego_pose.location.x
        ego_pose.location.y = -ego_pose.location.y
        # ego_pose.location.z = ego_pose.location.z
        ego_pose.rotation.roll = -math.radians(ego_pose.rotation.roll)
        ego_pose.rotation.pitch = math.radians(ego_pose.rotation.pitch)
        ego_pose.rotation.yaw = -math.radians(ego_pose.rotation.yaw)
        # velocity.x = velocity.x
        velocity.y = -velocity.y
        # velocity.z = velocity.z

        # --- --- Odometry --- ---
        o = Odometry()
        o.header.stamp = cur_time
        o.header.frame_id = "utm"
        o.child_frame_id = "gps"

        o.pose.pose.position.x = ego_pose.location.x
        o.pose.pose.position.y = ego_pose.location.y
        o.pose.pose.position.z = ego_pose.location.z
        q = tf.transformations.quaternion_from_euler(
            ego_pose.rotation.roll, ego_pose.rotation.pitch, ego_pose.rotation.yaw)  # RPY
        o.pose.pose.orientation.x = q[0]
        o.pose.pose.orientation.y = q[1]
        o.pose.pose.orientation.z = q[2]
        o.pose.pose.orientation.w = q[3]
        # o.pose.covariance == empty

        # calculate the center-axle offset
        A = numpy.matrix([[cos(-ego_pose.rotation.yaw), sin(-ego_pose.rotation.yaw), 0],
                          [-sin(-ego_pose.rotation.yaw), cos(-ego_pose.rotation.yaw), 0],
                          [0, 0, 1]])
        d = numpy.matrix([[self.center_axle_offset],
                          [0],
                          [1]])
        tmp = A.getI() * d
        o.pose.pose.position.x += tmp.item(0, 0)
        o.pose.pose.position.y += -tmp.item(1, 0)

        # o.twist.twist.angular == not available
        o.twist.twist.linear.x = velocity.x
        o.twist.twist.linear.y = velocity.y
        o.twist.twist.linear.z = velocity.z
        # o.twist.covariance == empty
        self.pub_odom.publish(o)

        # --- --- TF --- ---
        # --- base_footprint ---
        t = TransformStamped()
        t.header.stamp = cur_time
        t.header.frame_id = "map"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = o.pose.pose.position.x
        t.transform.translation.y = o.pose.pose.position.y
        t.transform.translation.z = o.pose.pose.position.z
        t.transform.rotation.x = o.pose.pose.orientation.x
        t.transform.rotation.y = o.pose.pose.orientation.y
        t.transform.rotation.z = o.pose.pose.orientation.z
        t.transform.rotation.w = o.pose.pose.orientation.w
        # # NOTE center-axle offset already calculated for Odometry
        tf_msg = []
        tf_msg.append(t)
        self.pub_tf.publish(TFMessage(tf_msg))

        # --- --- Joint States --- ---
        # TODO calculate joints
        jsv = forward_speed / self.wheel_radius
        js = JointState()
        js.header.frame_id = ""
        js.header.stamp = cur_time
        js.name = ['wheel_fl', 'wheel_fr', 'wheel_rl',
                   'wheel_rr', 'steer_fl', 'steer_fr']
        js.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        js.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        js.velocity = [jsv, jsv, jsv, jsv, 0.0, 0.0]
        self.pub_js.publish(js)

        # --- --- Reports --- ---
        # --- WheelSpeedReport ---
        w = WheelSpeedReport()
        w.header.frame_id = ""
        w.header.stamp = cur_time
        w.front_left = forward_speed / self.wheel_radius
        w.front_right = forward_speed / self.wheel_radius
        w.rear_left = forward_speed / self.wheel_radius
        w.rear_right = forward_speed / self.wheel_radius
        self.pub_wsr.publish(w)

        # --- BrakeReport ---
        br = BrakeReport()
        br.header.frame_id = ""
        br.header.stamp = cur_time
        br.override = old_control['brake'] > 0.0
        br.pedal_input = old_control['brake']
        br.pedal_output = old_control['brake']
        self.pub_br.publish(br)

        # --- GearReport ---
        gr = GearReport()
        gr.header.frame_id = ""
        gr.header.stamp = cur_time
        gr.cmd.gear = old_control['gear']
        gr.state.gear = old_control['gear']
        self.pub_gr.publish(gr)

        # --- ThrottleReport ---
        tr = ThrottleReport()
        tr.header.frame_id = ""
        tr.header.stamp = cur_time
        tr.override = old_control['throttle'] > 0.0
        tr.pedal_input = old_control['throttle']
        tr.pedal_output = old_control['throttle']
        self.pub_tr.publish(tr)

        # --- SteeringReport ---
        sr = SteeringReport()
        sr.header.frame_id = ""
        sr.header.stamp = cur_time
        sr.speed = forward_speed
        steer = - old_control['steer']
        sr.steering_wheel_angle = steer * self.max_steer_angle
        sr.steering_wheel_angle_cmd = steer * self.max_steer_angle
        self.pub_sr.publish(sr)

    def processGodSensor(self, carla_actor):
        """

        """
        # --- --- Get Data --- ---
        simtime = TimedEventHandler().getCurrentSimTime()
        cur_time = rospy.Time()
        cur_time.set(int(simtime), int(1000000000 * (simtime - int(simtime))))
        header = Header(stamp=cur_time, frame_id='map')
        source = String(data="sim_carla")

        # --- --- Filter Data Semantically --- ---
        vehicles = [(actor.id, actor)
                    for actor in carla_actor.get_world().get_actors() if SemanticCarlaTags().get("Vehicles") in actor.semantic_tags and actor.id != carla_actor.id]
        pedestrians = [(actor.id, actor)
                       for actor in carla_actor.get_world().get_actors() if SemanticCarlaTags().get("Pedestrians") in actor.semantic_tags]
        traffic_lights = [(actor.id, actor)
                          for actor in carla_actor.get_world().get_actors() if SemanticCarlaTags().get("TrafficSigns") in actor.semantic_tags and actor.type_id == "traffic.traffic_light"]
        speed_limit_signs = [(actor.id, actor)
                             for actor in carla_actor.get_world().get_actors() if SemanticCarlaTags().get("TrafficSigns") in actor.semantic_tags and actor.type_id.startswith("traffic.speed_limit.")]

        # --- --- Filter Data with Detection Logic --- ---
        detected_vehicles = [
            getDetectedVehicle(vehicle, vehicle_id)
            for vehicle_id, vehicle in vehicles
        ]
        detected_pedestrians = [
            get_detected_pedestrian(pedestrian, pedestrian_id)
            for pedestrian_id, pedestrian in pedestrians
        ]

        # --- --- Publish Data --- ---
        detected_objects = detected_vehicles + detected_pedestrians  # + detected_traffic_lights + detected_speed_limit_signs
        object_list = ObjectList(header=header, source=source, objects=detected_objects)
        self.pub_ol.publish(object_list)


def get_detected_pedestrian(actor, actor_id):
    """
    Return a DetectedObject msg

    :param object: carla agent object (pb2 object (vehicle, pedestrian or traffic light))
    :param object_id: id of the detected object (int32)
    :return: a ros DetectedObject msg
    """
    # --- --- Get Data from Carla --- ---
    actor_pose = actor.get_transform()
    velocity = actor.get_velocity()
    forward_speed = sqrt(pow(velocity.x, 2.0) + pow(velocity.y, 2.0) + pow(velocity.z, 2.0))
    bounding_box = actor.bounding_box

    # carla to ros corrections
    # actor_pose.location.x = actor_pose.location.x
    actor_pose.location.y = -actor_pose.location.y
    # actor_pose.location.z = actor_pose.location.z
    actor_pose.rotation.roll = -math.radians(actor_pose.rotation.roll)
    actor_pose.rotation.pitch = math.radians(actor_pose.rotation.pitch)
    actor_pose.rotation.yaw = -math.radians(actor_pose.rotation.yaw)
    # velocity.x = velocity.x
    velocity.y = -velocity.y
    # velocity.z = velocity.z

    # --- --- Fill Data Into DetectedObject --- ---
    # --- Fill Default ---
    detected_pedestrian = DetectedObject()
    detected_pedestrian.id = actor_id  # unique id of the simulation

    # --- Fill Likelihood ---
    # we know its there
    detected_pedestrian.existence_probability = 1.0
    detected_pedestrian.class_probability.prob_bicycle = 0
    detected_pedestrian.class_probability.prob_car = 0
    detected_pedestrian.class_probability.prob_motorbike = 0
    detected_pedestrian.class_probability.prob_truck = 0
    detected_pedestrian.class_probability.prob_pedestrian = 1.0
    detected_pedestrian.class_probability.prob_stationary = 0

    # --- Fill Pose ---
    detected_pedestrian.reference_point = 1  # center

    detected_pedestrian.object.pose.pose.position.x = actor_pose.location.x
    detected_pedestrian.object.pose.pose.position.y = actor_pose.location.y
    detected_pedestrian.object.pose.pose.position.z = actor_pose.location.z
    q = tf.transformations.quaternion_from_euler(
        actor_pose.rotation.roll, actor_pose.rotation.pitch, actor_pose.rotation.yaw)  # RPY
    detected_pedestrian.object.pose.pose.orientation.x = q[0]
    detected_pedestrian.object.pose.pose.orientation.y = q[1]
    detected_pedestrian.object.pose.pose.orientation.z = q[2]
    detected_pedestrian.object.pose.pose.orientation.w = q[3]
    detected_pedestrian.object.pose.pose.position.z += bounding_box.location.z
    # detected_pedestrian.object.pose.covariance == empty
    detected_pedestrian.object.valid_pose = True
    detected_pedestrian.object.valid_orientation = True

    # --- Fill Orientation (redundant) ---
    detected_pedestrian.object.orientation_angle.x = actor_pose.rotation.pitch
    detected_pedestrian.object.orientation_angle.y = actor_pose.rotation.roll
    detected_pedestrian.object.orientation_angle.z = actor_pose.rotation.yaw
    detected_pedestrian.object.valid_orientation_angle = True

    # --- Fill Bounding Box ---
    detected_pedestrian.object.dimension.x = bounding_box.extent.x * 2.0
    detected_pedestrian.object.dimension.y = bounding_box.extent.y * 2.0
    detected_pedestrian.object.dimension.z = bounding_box.extent.z * 2.0
    detected_pedestrian.object.valid_dimension = True

    # --- Fill Twist ---
    detected_pedestrian.object.twist.twist.linear.x = velocity.x
    detected_pedestrian.object.twist.twist.linear.y = velocity.y
    detected_pedestrian.object.twist.twist.linear.z = velocity.z
    # detected_pedestrian.object.twist.twist.angular -> not available
    # detected_pedestrian.object.twist.covariance -> empty
    detected_pedestrian.object.valid_twist = True

    # detected_pedestrian.object.accel # available with get_acceleration()
    detected_vehicle.object.valid_acceleration = False

    detected_pedestrian.age = 0  # TODO its new?
    detected_pedestrian.prediction_age = 0  # TODO its new?

    return detected_pedestrian


def getDetectedVehicle(actor, actor_id):
    """
    Return a DetectedObject msg

    :param actor: carla actor (vehicle, pedestrian, traffic light, ...)
    :param actor_id: id of the detected object
    :return: a ros DetectedObject msg
    """
    # --- --- Get Data from Carla --- ---
    actor_pose = actor.get_transform()
    velocity = actor.get_velocity()
    forward_speed = sqrt(pow(velocity.x, 2.0) + pow(velocity.y, 2.0) + pow(velocity.z, 2.0))
    bounding_box = actor.bounding_box

    # carla to ros corrections
    # actor_pose.location.x = actor_pose.location.x
    actor_pose.location.y = -actor_pose.location.y
    # actor_pose.location.z = actor_pose.location.z
    actor_pose.rotation.roll = -math.radians(actor_pose.rotation.roll)
    actor_pose.rotation.pitch = math.radians(actor_pose.rotation.pitch)
    actor_pose.rotation.yaw = -math.radians(actor_pose.rotation.yaw)
    # velocity.x = velocity.x
    velocity.y = -velocity.y
    # velocity.z = velocity.z

    # --- --- Fill Data Into DetectedObject --- ---
    # --- Fill Default ---
    detected_vehicle = DetectedObject()
    detected_vehicle.id = actor_id  # unique id of the simulation

    # --- Fill Likelihood ---
    # we know its there, but we cannot say which type of vehicle it is
    # OPTIMIZE maybe calculate better likelihood via bounding box or blueprint name
    detected_vehicle.existence_probability = 1.0
    detected_vehicle.class_probability.prob_bicycle = 0.25
    detected_vehicle.class_probability.prob_car = 0.25
    detected_vehicle.class_probability.prob_motorbike = 0.25
    detected_vehicle.class_probability.prob_truck = 0.25
    detected_vehicle.class_probability.prob_pedestrian = 0
    detected_vehicle.class_probability.prob_stationary = 0

    # --- Fill Pose ---
    detected_vehicle.reference_point = 1  # center

    detected_vehicle.object.pose.pose.position.x = actor_pose.location.x
    detected_vehicle.object.pose.pose.position.y = actor_pose.location.y
    detected_vehicle.object.pose.pose.position.z = actor_pose.location.z
    q = tf.transformations.quaternion_from_euler(
        actor_pose.rotation.roll, actor_pose.rotation.pitch, actor_pose.rotation.yaw)  # RPY
    detected_vehicle.object.pose.pose.orientation.x = q[0]
    detected_vehicle.object.pose.pose.orientation.y = q[1]
    detected_vehicle.object.pose.pose.orientation.z = q[2]
    detected_vehicle.object.pose.pose.orientation.w = q[3]
    detected_vehicle.object.pose.pose.position.z += bounding_box.extent.z
    # detected_vehicle.object.pose.covariance == empty
    detected_vehicle.object.valid_pose = True
    detected_vehicle.object.valid_orientation = True

    # --- Fill Orientation (redundant) ---
    detected_vehicle.object.orientation_angle.x = actor_pose.rotation.pitch
    detected_vehicle.object.orientation_angle.y = actor_pose.rotation.roll
    detected_vehicle.object.orientation_angle.z = actor_pose.rotation.yaw
    detected_vehicle.object.valid_orientation_angle = True

    # --- Fill Bounding Box ---
    detected_vehicle.object.dimension.x = bounding_box.extent.x * 2.0
    detected_vehicle.object.dimension.y = bounding_box.extent.y * 2.0
    detected_vehicle.object.dimension.z = bounding_box.extent.z * 2.0
    detected_vehicle.object.valid_dimension = True

    # --- Fill Twist ---
    detected_vehicle.object.twist.twist.linear.x = velocity.x
    detected_vehicle.object.twist.twist.linear.y = velocity.y
    detected_vehicle.object.twist.twist.linear.z = velocity.z
    # detected_vehicle.object.twist.twist.angular -> not available
    # detected_vehicle.object.twist.covariance -> empty
    detected_vehicle.object.valid_twist = True

    # detected_vehicle.object.accel # available with get_acceleration(), but we only set Pose
    detected_vehicle.object.valid_acceleration = False

    detected_vehicle.age = 0  # TODO its new?
    detected_vehicle.prediction_age = 0  # TODO its new?

    return detected_vehicle
