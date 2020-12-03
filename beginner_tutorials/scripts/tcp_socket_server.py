#!/usr/bin/env python

import rospy
import socket
import time
import threading
from std_msgs.msg import String

class tcp_socket_server(object):
    def __init__(self):
        rospy.init_node('tcp_socket_server', anonymous=False)
        self.nodename = rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)
        rospy.on_shutdown(self.Shutdown)

        HOST = '192.168.123.200'  
        PORT = 12580
        BUFFERSIZE = 256
        self.ADDR = (HOST,PORT)  
        self.SockServer=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.SockServer.bind(self.ADDR)
        self.SockServer.listen(5)
        self.callback_data = String()
        rospy.Subscriber("chatter", String, self.callback)

    def tcplink(self,sock,addr):
        print 'Accept new connection from %s:%s...' % addr
        sock.send('Welcome!')
        while not rospy.is_shutdown():
            data = sock.recv(1024)
            time.sleep(1)
            if data == 'exit' or not data:
                break
            sock.send('Hello, %s!' % data)
        sock.close()
        print 'Connection from %s:%s closed.' % addr

    def callback(self,data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.callback_data = data

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            sock, addr = self.SockServer.accept()
            t = threading.Thread(target=self.tcplink, args=(sock, addr))
            t.start()
            
            rate.sleep()

    def Shutdown(self):
        rospy.logwarn("%s: shutting down" % self.nodename)
if __name__ == '__main__':
    tcpsocketclient = tcp_socket_server()
    try:
        tcpsocketclient.run()
    except rospy.ROSInterruptException:
        rospy.logerr("tcp_socket_server: exception thrown")
    rospy.spin()
