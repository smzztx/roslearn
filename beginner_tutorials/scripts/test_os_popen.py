#!/usr/bin/env python

import rospy
import subprocess
import os


class amy_bat(object):
    def __init__(self):
        rospy.init_node('test_os_popen', anonymous=False)
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)
        rospy.on_shutdown(self.Shutdown)



    def run(self):
        # os.system("rosrun beginner_tutorials talker.py")
        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            rospy.loginfo("rosrun beginner_tutorials talker.py")
            # os.system("rosrun beginner_tutorials talker.py")
            ps_talker = subprocess.Popen(["rosrun","beginner_tutorials","talker.py"])
            rospy.sleep(5)
            rospy.loginfo("rosnode kill /talker")
            # ps_talker.kill()
            ps_kill_talker = subprocess.Popen(["rosnode","kill","/talker"])
            ps_talker.wait()
            ps_kill_talker.wait()
            rate.sleep()

    def Shutdown(self):
        rospy.logwarn("%s: shutting down" % self.nodename)
if __name__ == '__main__':
    amybat = amy_bat()
    try:
        amybat.run()
    except rospy.ROSInterruptException:
        rospy.logerr("amy_bat: exception thrown")
    rospy.spin()
