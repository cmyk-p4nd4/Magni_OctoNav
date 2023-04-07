#!/usr/bin/env python

import rospy 
import cv2

from geometry_msgs.msg import PoseStamped, Quaternion, PointStamped, Point
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseActionFeedback
from matplotlib import pyplot as plt
from scipy.interpolate import interp1d
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker

grid = OccupancyGrid()
pub = None
def load():
    img = cv2.imread('map.png', 0)

    for i in range(len(img)):
        img[i] = (img[i] / 255) * 100

    grid.data = img
    pub.publish(grid)


def main():
    rospy.init_node('ogm_corner_detect')

    


    plt.ion()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
        pass
    
    plt.ioff


if __name__ == '__main__':
    main()