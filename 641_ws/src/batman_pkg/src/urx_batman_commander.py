#!/usr/bin/env python3

import rospy
from waypoints import get_ctrs
import glob, os
from os.path import dirname, abspath
pkg_path = dirname(dirname(abspath(__file__)))
from urx.robot import Robot

def main():
    rospy.init_node("turtlesim_batman_commander")
    rob = Robot("10.247.234.131", use_rt=True)
    # my_batman_turtle = TurtleBot()
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
    x_factor = 0.2 / my_shape[1]
    y_factor = 0.2 * aspect_ratio / my_shape[0]
    # print("type(my_ctrs)", type(my_ctrs))
    waypoints = my_ctrs[0]
    print("number of waypoints from the contour: ", len(waypoints))

    goals = []
    for idx, wp in enumerate(waypoints):
        # my_goal = [-0.2834079817223762, 0.7996098718156992, 0, 0.03721792822029717, 2.163286647869941, 2.200256493983291]
        my_goal = [-0.2834079817223762, 0.7996098718156992, -0.22211594266874537, 0.03721792822029717, 2.163286647869941, 2.200256493983291]
        my_goal[0] = wp[0][0] * x_factor + my_goal[0]
        my_goal[1] = (my_shape[0]-wp[0][1]) * y_factor + my_goal[1]
        goals.append(my_goal)

    # my_batman_turtle.move_waypoints(goals, 0.05)
    # rob.movels(goals, acc=0.1, vel=0.1, radius=0.002, wait=True, threshold=0.005)
    rob.movels(goals[:650], acc=0.1, vel=0.1, radius=0.002, wait=True, threshold=0.005)
    rob.movels(goals[650:1300], acc=0.1, vel=0.1, radius=0.002, wait=True, threshold=0.005)
    rob.movels(goals[1300:], acc=0.1, vel=0.1, radius=0.002, wait=True, threshold=0.005)
    # rob.movels([[-0.1417313772341532, 0.8377118897659168, -0.11673642337098096, 0.03738974285495133, 2.1634200317553476, 2.1999816863935258]], acc=0.01, vel=0.01, radius=0.002, wait=True, threshold=0.005)
    print("done")
    rob.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException: pass