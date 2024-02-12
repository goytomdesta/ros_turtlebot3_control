#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from std_srvs.srv import Empty
from sensor_msgs.msg import LaserScan
import random
import numpy as np 

class NavigationPlanner:

    # Define topic names
    INITIAL_POSE_TOPIC = "/initialpose"
    GOAL_TOPIC = "/move_base_simple/goal"
    RESULT_TOPIC = "/move_base/result"
    SCAN_TOPIC = "/scan"
    # Define maximum number of retries and retry threshold
    MAX_RETRIES = 100
    RETRY_THRESHOLD = 0.1
    OBSTACLE_THRESHOLD = 0.1  # Distance threshold for obstacle detection

    def __init__(self):
        # List of navigation points
        self.navigation_points = [(1.75, 1.0), (2.0, 0.0), (1.75, -1.0),
                                   (1.0, -1.75), (0.0, -2.0), (-1.0, -1.75),
                                   (-1.75, -1.0), (-2.0, 0.0), (-1.75, 1.0),
                                   (-1.0, 1.75), (0.0, 2.0), (1.0, 1.75),
                                   (-0.5, -0.5), (-0.5, 0.5), (0.5, 0.5),
                                   (0.5, -0.5)]
        self.goal_coords = ()
        self.prev_goal_coords = ()
        self.retry_count = 0

        # Publishers for initial pose and goal
        self.pose_init_pub = rospy.Publisher(self.INITIAL_POSE_TOPIC, PoseWithCovarianceStamped, queue_size=1)
        self.wait_for_subscriber(self.pose_init_pub)

        self.goal_pub = rospy.Publisher(self.GOAL_TOPIC, PoseStamped, queue_size=1)
        self.wait_for_subscriber(self.goal_pub)

        # Subscriber for navigation status
        self.navigation_status_sub = rospy.Subscriber(self.RESULT_TOPIC, MoveBaseActionResult, self.navigation_status_monitor)

        # Subscriber for laser scan data
        self.scan_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback)

        # Subscriber for localization information
        self.position_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.__get_localization)
        self.localization = None  # Initialize localization variable

        # Service proxy for clearing cost maps
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.clear_cost_maps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        # Publish initial pose
        self.publish_initial_pose()

    def wait_for_subscriber(self, publisher):
        # Wait for subscribers to connect
        while publisher.get_num_connections() == 0:
            rospy.loginfo("Waiting for subscribers to connect")
            rospy.sleep(1)

    def publish_initial_pose(self):
        # Publish the initial pose of the robot
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = -2.0
        msg.pose.pose.position.y = -0.5
        msg.pose.pose.orientation.w = 1.0
        self.pose_init_pub.publish(msg)
        rospy.sleep(2)

    def start_random_navigation(self):
        # Start a random navigation
        self.clear_cost_maps()
        rospy.sleep(1)

        if not self.prev_goal_coords:
            self.prev_goal_coords = (-2.0, -0.5)
        else:
            self.prev_goal_coords = self.goal_coords

        self.goal_coords = self.navigation_points[random.randrange(len(self.navigation_points))]
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self.goal_coords[0]
        msg.pose.position.y = self.goal_coords[1]
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        rospy.loginfo('[navigation_planner_demo] Navigation started towards: '+str(self.goal_coords))

    def navigation_status_monitor(self, data):
        # Monitor the navigation status and handle retries if needed
        if data.status.status == 3:
            rospy.loginfo('[navigation_planner_demo] Goal reached. Creating new navigation.')
            self.retry_count = 0
            self.start_random_navigation()
        elif data.status.status == 4:
            rospy.loginfo('[navigation_planner_demo] Goal failed. ' + str(self.prev_goal_coords) +
                          ' -> ' +str(self.goal_coords))
            if self.retry_count < self.MAX_RETRIES:
                rospy.loginfo('[navigation_planner_demo] Retrying navigation...')
                self.retry_count += 1
                self.start_random_navigation()
            else:
                rospy.logwarn('[navigation_planner_demo] Maximum retries reached. Abandoning navigation.')
                self.retry_count = 0
                self.start_retry_navigation()

    def __get_localization(self, msg):
        # Callback function to update localization information
        self.localization = msg.pose.pose.position
    
    def start_retry_navigation(self):
        # Start a retry navigation if the goal is within the retry threshold
        rospy.loginfo('[navigation_planner_demo] Fallback behavior: Retry navigation with obstacle avoidance...')
        dst = np.sqrt(((self.localization.x**2)-(self.goal_coords[0]**2))+((self.localization.y**2)-(self.goal_coords[1]**2)))

        if dst <= self.RETRY_THRESHOLD:
            rospy.loginfo("goal is within threshold")
            self.start_random_navigation()

    def scan_callback(self, msg):
        # Process the laser scan data to detect obstacles
        min_range = min(msg.ranges)  # Get the minimum range from the laser scan data
        if min_range < self.OBSTACLE_THRESHOLD:
            # Detected obstacle within threshold distance, take evasive action
            rospy.loginfo('[navigation_planner_demo] Obstacle detected. Taking evasive action...')
            self.take_evasive_action()

    def check_for_obstacles(self, msg):
        # Check if there are obstacles nearby using the laser scan data
        # For simplicity, we can check if the minimum range from the laser scan data is below a threshold
        return min(msg.ranges) < self.OBSTACLE_THRESHOLD

    def take_evasive_action(self):
        # Move the robot away from the obstacle using a Cartesian method (move backward)
        backward_distance = -0.3  
        msg = PoseStamped()
        msg.header.frame_id = 'base_link'  
        msg.pose.position.x = backward_distance
        msg.pose.orientation.w = 1.0  
        self.goal_pub.publish(msg)
        rospy.sleep(2)  

        # Attempt to start another path
        self.start_random_navigation()

        # If the attempt to start another path fails, clear the cost map and try again
        if not self.goal_coords:
            rospy.loginfo('[navigation_planner_demo] Attempt to start another path failed. Clearing cost maps and retrying...')
            self.clear_cost_maps()
    
    def clear_cost_maps(self):
        # Example of clearing cost maps
        self.clear_cost_maps()

if __name__ == '__main__':
    rospy.init_node('navigation_planner_demo', anonymous=False)
    n = NavigationPlanner()
    rospy.loginfo('[navigation_planner] Started')
    n.start_random_navigation()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[navigation_planner_demo] Stopped")







