#!/usr/bin/env python
import traceback
import sys
import socket
import rospy
import signal
import subprocess
import thread
import time
import xml.etree.cElementTree as ET
import io
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, PoseWithCovariance, Point, Quaternion, Twist
from SocketServer import ThreadingTCPServer, BaseRequestHandler
from std_msgs.msg import String

class AmyTestHandlerr(BaseRequestHandler):
	def handle(self):
		print 'Now connect form ',self.client_address[0]
		self.senddata='waitting'
		# self.cmd_vel = rospy.Publisher('/velocity', Twist, queue_size=10) 

	#plaser = subprocess.Popen(["rosrun", "amy_comm", "laser_send.py"])
		try:
			recvdata=self.request.recv(1024)
			while not rospy.is_shutdown():
				print recvdata
				if "TSONAR1" in recvdata:
					pass

				self.request.sendall(self.senddata+'\n')
				pubsend = rospy.Publisher('/tsend_data', String, queue_size=1)
				pubsend.publish(self.senddata)
				print self.senddata
				rospy.sleep(5)
		except : 
			traceback.print_exc()  
			print "broken connect "

class get_data():
	def __init__(self):
		self.data()

	def data(self):
		while not rospy.is_shutdown():	    		    			    		    		    	
			rospy.sleep(5)
			print 'you have wast 5 seconds of your life!'
	          
if __name__ == "__main__":
	host = '192.168.123.200'      
	port = 12580     
	addr = (host, port)

	# try:
	# 	get_data()
	# except: #rospy.ROSInterruptException:
	# 	rospy.loginfo("Exception thrown")

	rospy.init_node('socket_server', anonymous=False)
	thread.start_new_thread(get_data,())
	server = ThreadingTCPServer(addr, AmyTestHandlerr)
	server.serve_forever()
	rospy.spin()


