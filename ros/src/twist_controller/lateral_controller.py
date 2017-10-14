import rospy

from pid import PID
from yaw_controller import YawController
import numpy as np
import math


LAT_JERK_LIMIT = 5.0

class LatController(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle):
        self.wheel_base = wheel_base
        self.steer_ratio = steer_ratio
        self.min_speed = min_speed
        self.max_lat_accel = max_lat_accel

        self.min_angle = -max_steer_angle
        self.max_angle = max_steer_angle

        # init feed forward yaw-rate control
        self.yawControl = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        # init PID
        self.steer_PID = PID(kp=0.05, ki=0.0005, kd=0.5)
       
        self.last_steer = 0.0
        
    def control(self,target_spd, target_yawRate, current_spd, waypoints, pose, delta_t):
        
        steer = self.last_steer
        if current_spd > self.min_speed:
            # feed forward control to drive curvature of road
            steer_feedForward = self.yawControl.get_steering(target_spd, target_yawRate, current_spd)
            # limit steering angle
            steer_feedForward = max(min(steer_feedForward, self.max_angle), self.min_angle)
            
            # PID control
            CTE = self.calc_CTE(waypoints, pose)
            steer_PID = self.steer_PID.step(CTE, delta_t, mn=self.min_angle-steer_feedForward, mx=self.max_angle-steer_feedForward)
            
            # steering command
            steer = steer_feedForward + steer_PID
            self.last_steer = steer
        else:
            self.steer_PID.freeze()
                     
        return steer
        
    def reset(self):
        self.steer_PID.reset()
        
    def transfromWPcarCoord(self, waypoints, pose):
        n = len(waypoints)
         # get car's x and y position and heading angle
        car_x = pose.position.x
        car_y = pose.position.y
        # get orientation
        s = pose.orientation.w # quaternion scalar
        v1 = pose.orientation.x # vector part 1
        v2 = pose.orientation.y # vector part 2
        v3 = pose.orientation.z # vector part 3        
        
        # now obtaining orientation of the car (assuming rotation about z: [0;0;1])
        car_theta = 2 * np.arccos(s)
        # Constraining the angle in [-pi, pi)
        if car_theta > np.pi:
            car_theta = -(2 * np.pi - car_theta)
        #car_theta = pose.orientation.z
       
        # transform waypoints in vehicle coordiantes
        wp_carCoord_x = np.zeros(n)
        wp_carCoord_y = np.zeros(n)

        for i in range(n):
            wp_x = waypoints[i].pose.pose.position.x
            wp_y = waypoints[i].pose.pose.position.y            
            wp_carCoord_x[i] = (wp_y-car_y)*math.sin(car_theta)-(car_x-wp_x)*math.cos(car_theta)
            wp_carCoord_y[i] = (wp_y-car_y)*math.cos(car_theta)-(wp_x-car_x)*math.sin(car_theta)
            
        return wp_carCoord_x, wp_carCoord_y
                
    def calc_CTE(self, waypoints, pose):
        # transfrom waypoints into vehicle coordinates
        wp_carCoord_x, wp_carCoord_y = self.transfromWPcarCoord(waypoints, pose)
        
        # get waypoint which should be used for controller input
        viewRange = 10.0
        n_points_min = 5
        idxFirstWP, idxLastWP = self.findWPs(wp_carCoord_x, viewRange, n_points_min)
                
        # do some checks if enough waypoints in the front of the car are available

        if (idxFirstWP<0):
            rospy.logerr("dbw_node: No waypoint in front of car for lateral control received!")
            return 0.0
        elif (idxLastWP<0):
            rospy.logerr("dbw_node: Not enough waypoints for a view range of %f m!" % viewRange)
            return 0.0
        elif (idxLastWP+1-idxFirstWP < n_points_min):
            rospy.logerr("dbw_node: Resolution of waypoints not high enough!")
            return 0.0
        else:
            # Interpolate waypoints (already transformed to vehicle coordinates) to a polynomial of 3rd degree
            coeffs = np.polyfit(wp_carCoord_x[idxFirstWP:idxLastWP+1], wp_carCoord_y[idxFirstWP:idxLastWP+1], 3)
            p = np.poly1d(coeffs)
            # distance to track is polynomial at car's position x = 0
            CTE = p(0.0)
            return CTE
        
    def findWPs(self, wp_carCoord_x, viewRange, n_points_min):
        # this function return first waypoint in front of car
        # it is asumed that wayponts are already ordered (by waypoint_updater)
        # if return value is negative then no point in front of car is found!
        
        # transfrom waypoint in vehcile coordnates
        idxFirstWP = -1
        idxLastWP = -1
        n_pts = len(wp_carCoord_x)
        flagFirstWP = False
        for i in range(n_pts):
            if not flagFirstWP and wp_carCoord_x[i] >= 0.0:
                idxFirstWP = i
                flagFirstWP = True
            elif wp_carCoord_x[i] >= viewRange:
                idxLastWP = i
                break
        # if view range to less than check if enough points are available
        
        #rospy.logwarn("idxFirstWP: %f" % idxFirstWP + "idxLastWP: %f" % idxLastWP )
        #if idxFirstWP>=0 and idxLastWP<0 and n_pts-idxFirstWP >= n_points_min:
        #    idxLastWP = n_pts-1 # zero based indexing
        return idxFirstWP, idxLastWP

