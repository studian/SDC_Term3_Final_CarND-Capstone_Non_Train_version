#!/usr/bin/env python

import rospy
from styx_msgs.msg import TrafficLight
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import numpy as np

import math

import sys
sys.setrecursionlimit(10000) # deep recursion (ONLY) during insertions

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
KMPH2MPS = 1000. / (60. * 60.)   # 0.277778
MPH2MPS = 0.44704
#MAX_SPD = 20.0 * KMPH2MPS
#MAX_SPD = 10.0 * MPH2MPS
#MAX_SPD = 5.0

DIST_STOP_TL = 6.0

CAR_STATE_STOP = 0
CAR_STATE_SLOWDOWN = 1
CAR_STATE_MAX_SPEED = 2
CAR_STATE_OVERDIVE = 3


class WaypointUpdater(object):
    def __init__(self):
        
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
         

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # get max velocity from Waypoint Loader's config
        #max_velocity_kmph = rospy.get_param('/waypoint_loader/velocity') 
        #self.max_velocity = max_velocity_kmph * KMPH2MPS
        #rospy.logwarn("Max velocity: {0:.2f} km/h".format(max_velocity_kmph))

        self.current_velocity = None
        # get current velocity too
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)

        # TODO: Add other member variables you need below
        self.flag_waypoints_retrieved = False # flag for retrieving waypoints only once
        self.base_waypoints = None
        self.num_waypoints = -1 # just a shortcut to avoid using len() all the time

        self.pose = None
        self.pose_stamp = None
        self.car_x = None
        self.car_y = None
        #self.car_z = None # keep always 0
        self.car_theta = None

        # The track waypoints stored as:
        # [x, y, s, d, v_s, v_d, index]
        self.track = None
        self.track_root = None # the root of the kd-tree

        # variables for traffic lights
        self.tl_waypoint = None
        self.last_tl_waypoint = None
        self.tl_state = TrafficLight.UNKNOWN
        self.last_tl_state = TrafficLight.UNKNOWN
        # Keep track of last max speed for traffic light behavior
        self.last_max_spd = 2 
        
        # define a car state as a small state machine to react on traffic lights
        self.car_state_tl = CAR_STATE_STOP
        
        # memorize next waypoint
        self.next_wp_index = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            if self.flag_waypoints_retrieved and self.pose is not None:
                # unwrapping the vehicle pose
                self.car_x = self.pose.position.x
                self.car_y = self.pose.position.y
                #car_z = self.pose.position.z # not used but hey...

                # get orientation
                s = self.pose.orientation.w # quaternion scalar
                v1 = self.pose.orientation.x # vector part 1
                v2 = self.pose.orientation.y # vector part 2
                v3 = self.pose.orientation.z # vector part 3        
                
                # now obtaining orientation of the car (assuming rotation about z: [0;0;1])
                self.car_theta = 2 * np.arccos(s)
                # Constraining the angle in [-pi, pi)
                if self.car_theta > np.pi:
                    self.car_theta = -(2 * np.pi - self.car_theta)
                # Now get the next waypoint....
                if self.flag_waypoints_retrieved:
                    (self.next_wp_index, step) = self.findNextWaypoint()
                    
                   # publish the nodes
                    self.publishWaypoints(self.next_wp_index, step)
            rate.sleep()


    # Again trying another approach in the next waypoint without considering 
    # orientation. This timewe use a cross product of position-minus-neareset wp
    # and direction vector from nearest to next.
    def findNextWaypoint(self):
        # use brute force minimum distance
        nn_index = 0
        map_x = self.base_waypoints[nn_index].pose.pose.position.x
        map_y = self.base_waypoints[nn_index].pose.pose.position.y
        mindist = (self.car_x - map_x) ** 2 + (self.car_y - map_y) ** 2
        
        for i in range(1, self.num_waypoints):
            x = self.base_waypoints[i].pose.pose.position.x
            y = self.base_waypoints[i].pose.pose.position.y
            
            dist = (self.car_x - x) ** 2 + (self.car_y - y) ** 2            
            if (dist < mindist):
                mindist = dist
                map_x = x
                map_y = y
                nn_index = i
        # now this node maybe 'behind ' or 'ahead' of the car
        # with repsect to its ****current heading*****
        # So we need to take cases 
        #nn_index = nearest_node.index
        next_wp_index = ( nn_index + 1 ) % len(self.base_waypoints)
        
        # now the difference vector of the car's position and the direction
        # vector v from the nearest waypoint to the next
        vx = self.base_waypoints[next_wp_index].pose.pose.position.x - map_x
        vy = self.base_waypoints[next_wp_index].pose.pose.position.y - map_y
        norm_v = np.sqrt( vx*vx + vy*vy )
        vx /= norm_v
        vy /= norm_v
        # now the difference : car position - nearest wp
        dx = self.car_x - map_x
        dy = self.car_y - map_y
        # Get the dot product of d and v
        dot = vx * dx + vy * dy
        if dot >= 0:
            return (next_wp_index, +1)
        else:
            return (nn_index, +1)
        
    def pose_cb(self, msg):
        self.pose = msg.pose
        # get the time stamp. might be useful to calculate latency
        self.pose_stamp = msg.header.stamp
    
    # returns a string that represents the waypoints as a matlab matrix            
    def getWaypointMatrixAtr(self):
        str_ = " [ "
        for wp in self.base_waypoints:
            str_ += str(wp.pose.pose.position.x) + " , " + str(wp.pose.pose.position.y) + " ; "                 
        str_ = str_ + " ] "
        
        return str_
    
    # This function publishes the next waypoints
    # For now it sets velocity to the same value...        
    def publishWaypoints(self, next_wp_index, step):
        
        msg = Lane()
        #msg.header.stamp = rospy.Time
        msg.waypoints = []
        index = next_wp_index
        
        current_velocity = self.current_velocity.linear.x if self.current_velocity is not None else 0.0
        if self.tl_waypoint is not None:
            dist_tl = self.distance(self.base_waypoints, index, self.tl_waypoint)
            # show more useful info
            #rospy.logwarn("Distance to Red TL: {0:.3f} m, vel {1:.2f} km/h, wp ix {2}".format(
            #                                        dist_tl, current_velocity*3.6, self.tl_waypoint))
            rospy.logwarn("Distance of Traffic light : {0:.3f} m, velocity {1:.2f} km/h".format(dist_tl, current_velocity*3.6))
        else:
            rospy.logwarn("....................................., velocity : {0:.2f} km/h".format(current_velocity*3.6))
        for i in range(LOOKAHEAD_WPS):
            # index of the trailing waypoints 
            wp = Waypoint()
            wp.pose.pose.position.x = self.base_waypoints[index].pose.pose.position.x
            wp.pose.pose.position.y = self.base_waypoints[index].pose.pose.position.y
            # Velocity
            # TODO - TODO : Fill it with sensible velocities using
            #               feedback from the traffic light node etc...
            
            # get maximum allowed speed from base waypoint
            max_spd = self.base_waypoints[index].twist.twist.linear.x
            #max_spd = 10 * KMPH2MPS # 10 km/h

            if self.car_state_tl == CAR_STATE_OVERDIVE or self.car_state_tl == CAR_STATE_MAX_SPEED:
                wp.twist.twist.linear.x = max_spd
            else:
                if self.tl_waypoint is None: # should never happen
                    wp.twist.twist.linear.x = 0.0 # safe state
                else:
                    dist_tl = self.distance(self.base_waypoints, index, self.tl_waypoint)
                    if self.car_state_tl == CAR_STATE_STOP:
                        wp.twist.twist.linear.x = min(max(0.0, 0.2*(dist_tl-DIST_STOP_TL)), max_spd)
                        # stop vehicle if vehicle is almost standing and target speed is very low for waypoint
                        if current_velocity < 0.25 and wp.twist.twist.linear.x < 0.25:
                            wp.twist.twist.linear.x = 0.0
                    elif self.car_state_tl == CAR_STATE_SLOWDOWN:
                        # slow down to max_spd/2
                        wp.twist.twist.linear.x = min(max(max_spd/2, max_spd/2+0.2*(dist_tl-DIST_STOP_TL)), max_spd)
                    else: # should never happen
                        wp.twist.twist.linear.x = 0.0 # safe state

            # add the waypoint to the list
            msg.waypoints.append(wp)
        
            # increas/decrease index
            index = (index + step) % self.num_waypoints
            
        # publish the message
        self.final_waypoints_pub.publish(msg)
        
    def waypoints_cb(self, lanemsg):
        # unwrap the message
        if not self.flag_waypoints_retrieved:
            #header = lanemsg.header
            self.base_waypoints = lanemsg.waypoints
            self.num_waypoints = len(self.base_waypoints)

            # raise the flag so that we don't have to do this again...
            self.flag_waypoints_retrieved = True            
            
            return
            
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        tl_wp = msg.data
        if tl_wp >= 0: # A traffic light is detected
            self.tl_state = (tl_wp & 0xFFFF0000) >> 16            
            self.tl_waypoint = tl_wp & 0xFFFF
            if ( self.last_tl_waypoint is None ) and self.tl_state == TrafficLight.RED:
                rospy.logwarn("Red traffic light at : %d", tl_wp)
            
        else:
            self.tl_waypoint = None
            self.tl_state = TrafficLight.UNKNOWN
            if self.last_tl_waypoint is not None:
                rospy.logwarn("Green traffic light!! Let's GO~~!")
        
        # set small statemachine to react on traffic lights
        current_velocity = self.current_velocity.linear.x if self.current_velocity is not None else 0.0

        if self.base_waypoints is None or self.next_wp_index is None:  #not all data received yet
            self.car_state_tl = CAR_STATE_STOP
        else: 
            if self.tl_waypoint is None: # no traffic light
                self.car_state_tl = CAR_STATE_MAX_SPEED
            else:
                dist_tl = self.distance(self.base_waypoints, self.next_wp_index, self.tl_waypoint)
                time2tlStop = 9999.9 # high value
                if current_velocity > 1.: # avoid devision by zero and car moving
                    time2tlStop = max(0.0,dist_tl-DIST_STOP_TL) / current_velocity
                
                if self.tl_state == TrafficLight.YELLOW and self.last_tl_state == TrafficLight.GREEN:
                    if time2tlStop < 1.0: # no time to react -> full speed
                        self.car_state_tl = CAR_STATE_OVERDIVE
                    else: # break
                        self.car_state_tl = CAR_STATE_SLOWDOWN
        
                elif self.tl_state == TrafficLight.YELLOW and self.last_tl_state == TrafficLight.YELLOW:
                    if self.car_state_tl == CAR_STATE_OVERDIVE: 
                        self.car_state_tl = CAR_STATE_OVERDIVE
                    else:
                        if time2tlStop < 1.0: # no time to react -> full speed
                            self.car_state_tl = CAR_STATE_OVERDIVE
                        else: # break
                            self.car_state_tl = CAR_STATE_STOP
                    
                elif self.tl_state == TrafficLight.YELLOW and self.last_tl_state == TrafficLight.RED:
                    if time2tlStop < 1.0: # dont cross before green
                        self.car_state_tl = CAR_STATE_SLOWDOWN # don't cross before green - not leaving red light
                    else:
                        self.car_state_tl = CAR_STATE_MAX_SPEED # Leaving a red light. Give it all you have got!

                elif self.tl_state == TrafficLight.YELLOW and self.last_tl_state == TrafficLight.UNKNOWN:
                    if self.car_state_tl == CAR_STATE_OVERDIVE:
                        self.car_state_tl = CAR_STATE_OVERDIVE
                    else:
                        if time2tlStop < 1.0: # no time to react -> full speed
                            self.car_state_tl = CAR_STATE_OVERDIVE
                        else: # watch out
                            self.car_state_tl = CAR_STATE_SLOWDOWN
                elif self.tl_state == TrafficLight.GREEN:
                    self.car_state_tl = CAR_STATE_MAX_SPEED
                elif self.tl_state == TrafficLight.RED:
                    if self.car_state_tl == CAR_STATE_OVERDIVE: 
                        self.car_state_tl = CAR_STATE_OVERDIVE
                    elif dist_tl < 0.1 and current_velocity > 1*KMPH2MPS:
                        self.car_state_tl = CAR_STATE_OVERDIVE
                    else:
                        self.car_state_tl = CAR_STATE_STOP
                else: # The |UNKNOWN" case. Always step on it!
                    self.car_state_tl = CAR_STATE_MAX_SPEED
 
        
        # keep track of previous state. Useful in deciding speed transitioning
        self.last_tl_waypoint = self.tl_waypoint
        self.last_tl_state = self.tl_state
        
        return

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def current_velocity_cb(self, msg):
        ''' Callback for /current_velocity topic
            Simply save the current velocity value
        '''
        self.current_velocity = msg.twist

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_waypoint_from_index(self, wp_index):
        ''' Return the waypoint specified by wp_index from base_waypoints '''
        return self.base_waypoints[wp_index]
        


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
