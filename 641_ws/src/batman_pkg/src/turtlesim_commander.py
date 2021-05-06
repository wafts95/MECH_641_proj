#!/usr/bin/env python3

import rospy
from TurtleBot import TurtleBot
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from waypoints import get_ctrs
import glob, os
from os.path import dirname, abspath
pkg_path = dirname(dirname(abspath(__file__)))


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