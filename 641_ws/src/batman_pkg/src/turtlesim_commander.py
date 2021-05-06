#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from std_srvs.srv import Empty
from waypoints import get_ctrs
import glob, os
from os.path import dirname, abspath
pkg_path = dirname(dirname(abspath(__file__)))


class TurtleBot:

    def __init__(self):
        # # Creates a node with name 'turtlebot_controller' and make sure it is a
        # # unique node (using anonymous=True).
        # rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(1000)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1.5*10):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=80):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self, goal_pose, distance_tolerance):
        """Moves the turtle to the goal."""

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        if distance_tolerance == 0:
            rospy.logwarn("distance_tolerance parameter cannot be exactly 0.\n"
                          "setting distance_tolerance to 0.001")
            distance_tolerance = 0.001

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        return

        # If we press control + C, the node will stop.
        #rospy.spin()


def main():
    rospy.init_node("turtlesim_batman_commander")
    clear_srv = rospy.ServiceProxy("/clear", Empty())
    my_batman_turtle = TurtleBot()
    media_path = os.path.join(pkg_path, "media")

    print("Below are the available logos that can be drawn:\n")
    for idx, file in enumerate(os.listdir(media_path)):
        if file.endswith(".jpg") or file.endswith(".jpeg") or file.endswith(".png"):
            print("%i- %s" % (idx, file))

    desired_logo_idx = int(input("\nSelect the number of the desired logo to draw using turtlesim: "))
    desired_logo_path = os.path.join(media_path, os.listdir(media_path)[desired_logo_idx])
    # print("desired_logo_path", desired_logo_path)
    my_ctrs, my_shape = get_ctrs(desired_logo_path, False)
    aspect_ratio = my_shape[0]/my_shape[1]
    x_factor = 11 / my_shape[1]
    y_factor = 11 * aspect_ratio / my_shape[0] 
    # print("type(my_ctrs)", type(my_ctrs))
    waypoints = my_ctrs[0]
    print("number of waypoints from the contour: ", len(waypoints))

    goals = []
    for idx, wp in enumerate(waypoints):
        my_goal = Pose()
        my_goal.x = wp[0][0] * x_factor
        my_goal.y = (my_shape[0]-wp[0][1]) * y_factor
        goals.append(my_goal)

    my_batman_turtle.move2goal(goals[0], 0.001)
    clear_srv.call()
    goals.pop(0)
    print("starting movement\n")
    for idx, goal in enumerate(goals):
        print("movement %s\n" % (idx+1))
        my_batman_turtle.move2goal(goal, 0.03)
    print("done")
    


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException: pass