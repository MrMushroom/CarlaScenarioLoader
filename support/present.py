"""
Classes to handle Agent object (player and non-player)
Inspired by carla_ros_bridge Carla 0.8.4
"""

import numpy
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
