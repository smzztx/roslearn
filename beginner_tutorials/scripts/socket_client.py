#!/usr/bin/env python

import rospy
import socket

class socket_client(object):
    def __init__(self):
        rospy.init_node('socket_client', anonymous=False)
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)
        rospy.on_shutdown(self.Shutdown)
        
        HOST = '192.168.123.200'  
        PORT = 12580
        BUFFERSIZE = 256
        self.ADDR = (HOST,PORT)  
        self.tcpSockClt=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        try:
            self.tcpSockClt.connect(self.ADDR)
            rospy.loginfo("connected")
        except socket.error:
            rospy.logwarn("%s: connect error" % self.nodename)
    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            print 'client'
            # try:
            #     self.tcpSockClt.connect(self.ADDR)
            #     rospy.loginfo("connected")
            # except socket.error:
            #     rospy.logwarn("%s: connect error" % self.nodename)
            try:
                self.tcpSockClt.send('#BATP\n')
                print self.tcpSockClt.recv(1024)
                rospy.loginfo("send")
            except socket.error:
                rospy.logwarn("%s: send error" % self.nodename)
            else:
                self.tcpSockClt.send('exit')
                self.tcpSockClt.close()

            rate.sleep()

    def Shutdown(self):
        rospy.logwarn("%s: shutting down" % self.nodename)
if __name__ == '__main__':
    socketclient = socket_client()
    try:
        socketclient.run()
    except rospy.ROSInterruptException:
        rospy.logerr("socket_client: exception thrown")
    rospy.spin()
