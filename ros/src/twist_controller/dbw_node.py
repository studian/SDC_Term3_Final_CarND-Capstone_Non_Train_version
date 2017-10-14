#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
from styx_msgs.msg import Lane


from longitudinal_controller import LongController
from lateral_controller import LatController

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # below this speed no control any more
        # and we hold the vehicle if target speed is zero 
        min_speed = 0.1


        # init controller
        self.longControl = LongController(vehicle_mass,brake_deadband,decel_limit,accel_limit,wheel_radius)
        self.latControl = LatController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # init variables
        self.dbw_enabled = False
        self.velocity = None
        self.pose = None
        self.twist = None
        self.waypoints = None
        self.last_timestamp = 0.0

        # Subscribe to needed topics
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_cb, queue_size=1)
        rospy.Subscriber('/final_waypoints', Lane, self.waypoints_cb, queue_size=1)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb, queue_size=1)
	
        
        self.loop()

    def loop(self):
        rate = rospy.Rate(20) # 20Hz
        while not rospy.is_shutdown():
            self.control_step()
            rate.sleep()

    def control_step(self):
        now = rospy.get_rostime()
        timestamp = now.to_sec()
        delta_t = timestamp - self.last_timestamp
        self.last_timestamp = timestamp

        flag_dataRX = self.velocity is not None and \
                      self.pose is not None and \
                      self.twist is not None and \
                      self.waypoints is not None
                      
        throttle = 0.0
        brake = 0.0
        steer = 0.0

        #rospy.logwarn("delta_t: %f" % delta_t)

        if flag_dataRX and delta_t >0:
            current_spd = self.velocity.linear.x
            if self.dbw_enabled:
                # longitudinal control
                target_spd = self.twist.linear.x
                throttle, brake =  self.longControl.control(target_spd,current_spd,delta_t)
                
                # convert from m/s to km/h
                #rospy.logwarn("target_spd: {0:.2f}, current_spd: {1:.2f} km/h, throttle: {2:.2f} %, brake: {3:.2f} Nm".format(target_spd * 3.6, current_spd * 3.6, throttle, brake))
                #rospy.logwarn("throttle: %f" % throttle + "; brake: %f" % brake)
                
                # lateral control
                target_yawRate = self.twist.angular.z
                #rospy.logwarn("CTE: %f" % CTE)
                steer = self.latControl.control(target_spd, target_yawRate, current_spd, self.waypoints, self.pose, delta_t)
                
            else:
                self.longControl.reset(current_spd)
                self.latControl.reset()
                
        else:
            self.longControl.reset(0.0)
            self.latControl.reset()
        
            

        self.publish(throttle, brake, steer)

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)

    def dbw_cb(self, message):
        # extract dbw_enabled variable
        self.dbw_enabled = bool(message.data)

    def velocity_cb(self, message):
        # extract velocity
        self.velocity = message.twist

    def pose_cb(self, message):
        # extract position
        self.pose = message.pose

    def twist_cb(self, message):
        # extract the twist message """
        self.twist = message.twist

    def waypoints_cb(self, message):
        # extract waypoints
        self.waypoints = message.waypoints
        
                
if __name__ == '__main__':
    DBWNode()
