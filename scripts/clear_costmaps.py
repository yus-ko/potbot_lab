#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

def call_clear_costmaps():
    rospy.wait_for_service('/robot_0/move_base/clear_costmaps')
    try:
        clear_costmaps = rospy.ServiceProxy('/robot_0/move_base/clear_costmaps', Empty)
        clear_costmaps()  
        rospy.loginfo("Successfully called /move_base/clear_costmaps")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('clear_costmap_caller')

    rate = rospy.Rate(1)  

    while not rospy.is_shutdown():
        call_clear_costmaps()  
        rate.sleep()
