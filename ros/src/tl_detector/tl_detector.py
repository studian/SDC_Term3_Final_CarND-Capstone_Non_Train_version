#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import numpy as np
import random
import math

STATE_COUNT_THRESHOLD = 3
# this means how many frames the image is delay
# set DELAY=1 means there is no delay.
DELAY = 1
# if a light is more then 200m away from the car
# we'll ignore that light
MAX_DISTANCE_SQR = 40000

class Point:
    def __init__(self, t):
        self.x = t[0]
        self.y = t[1]

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.tl_wps = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        rospy.logwarn("The parameter string is : %s", config_string )
        self.config = yaml.load(config_string)

        self.upcoming_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        
        # debug: publisher of cross-correlation results
        self.ccresult_pub = rospy.Publisher('/traffic_ccresult', Image, queue_size=1)
        
        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.count = 0
        self.camera_car_position = []        
        
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
       
    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            if state == TrafficLight.UNKNOWN:        
                self.upcoming_light_pub.publish(Int32(-1))
                return
            self.last_wp = light_wp
            self.upcoming_light_pub.publish(Int32( light_wp | (self.state << 16) ))
        else:
            self.upcoming_light_pub.publish(Int32(self.last_wp | (self.last_state << 16) ))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_index = -1
        closest_dis = -1

        if self.waypoints is not None:
            wps = self.waypoints.waypoints
            for i in range(len(wps)):
                dis = (wps[i].pose.pose.position.x - pose.x) ** 2 + \
                    (wps[i].pose.pose.position.y - pose.y) ** 2

                if (closest_dis == -1) or (closest_dis > dis):
                    closest_dis = dis
                    closest_index = i
        return closest_index


    # George: Here's my version of project_to_image_plane
    #     I am avoiding the TransformListener object as I am not sure about how
    #     to configure it without having doubts. The transform is easy to 
    #     work-out directly from the pose vector and gives me control over the 
    #     coordinate frame.
    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """
        # Retreving camera intronsics
        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        # image size        
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        #TODO Use tranform and rotation to calculate 2D position of light in image

        x = 0
        y = 0
        
        return (x,y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)

        #get traffic light classification
        return self.light_classifier.traffic_predict(cv_image)
        
    def track_index_diff(self,index1, index2):
        if (self.waypoints.waypoints is None):
            return -1        
        N = len(self.waypoints.waypoints)
        if index2 >= index1 :
            return index2 - index1
        else:
            return N - index1 - 1 + index2
   
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if (self.waypoints is None):
            return (-1, TrafficLight.UNKNOWN)
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        
        # now, for all stop_lines find nearest points (shoudl be done ONCE
        if (len(self.tl_wps)==0):           
            for i, stop_line in enumerate(stop_line_positions):
                tl_wp = self.get_closest_waypoint(Point(stop_line))
                self.tl_wps.append( (tl_wp+5) % len(self.waypoints.waypoints) ) # +1 to give extra margin towards the traffic light
            
        
        light = None
        
        if(self.pose):
            car_wp = self.get_closest_waypoint(self.pose.pose.position)

            
            # Now find the smallest distance to a traffic light wp from the car wp:
            minDist = self.track_index_diff(car_wp, self.tl_wps[0])
            light_index = 0
            for i in range(1, len(self.tl_wps)):
                dist = self.track_index_diff(car_wp, self.tl_wps[i])
                if dist > 150 / 0.63: #about 150 meters ON-TRACK 
                    continue
                if (dist < minDist):
                    minDist = dist                    
                    light_index = i
            
            light = self.lights[light_index]
            #print("Nearest Traffic light index  : ", light_index)
        
        if light:
            # print(light_wp, self.count)
            state = self.get_light_state(light)
            return self.tl_wps[light_index], state
        
        return -1, TrafficLight.UNKNOWN       
   

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
