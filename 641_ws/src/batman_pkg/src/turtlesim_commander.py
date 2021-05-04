#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from std_srvs.srv import Empty
from waypoints import pleb


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
    
    # b_move = input("\ndo you want to go somewhere? (y/n)\t")
    # while b_move == "y":
    #     desired_pose = Pose()
    #     desired_pose.x = float(input("\nenter desired x pose: "))
    #     desired_pose.y = float(input("\nenter desired y pose: "))
    #     my_batman_turtle.move2goal(desired_pose, 0.001)
    #     b_move = input("\ndo you want to go somewhere else? (y/n)\t")

    # x_factor = 11/20
    # y_factor = 11/40

    # waypoints = [(10,0), (10,0.5), (3.5, 4), (3.5, 4.05),
    #              (1,9), (1.005,9), (7,9), (7,8.95), (8,7.1),
    #              (9.2, 7), (9.2, 7.005), (9.5,8.75), (9.5,8.74),
    #              (9.6, 7.9), (9.65, 7.9), (10.4, 7.9), (10.4, 7.91),
    #              (10.5, 8.75), (10.52,8.73), (10.8,7), (11,7), (12,7.1)]

    goals = []
    my_ctrs, my_shape = pleb()
    # x_factor = 11/768
    # y_factor = 11/410/2
    x_factor = 11/my_shape[1]
    y_factor = 11/my_shape[0]/2
    # print("type(my_ctrs)", type(my_ctrs))
    waypoints = my_ctrs[0]
    print("len(waypoints", len(waypoints))
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