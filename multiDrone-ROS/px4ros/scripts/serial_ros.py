#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import serial
import time

#function: record current time(s)
current_milli_time = lambda: int(time.time()*1000)

def serial_node():

	pub = rospy.Publisher('serial_data_received', String, queue_size=20)
	rospy.init_node('px4ros_serial', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	rospy.Subscriber("serial_data_send", String, callback)
	
	opm_string = ''
	to_send_string = ''
	
	while not rospy.is_shutdown():	
		#hello_str = "hello world %s" % rospy.get_time()

		opm_string = myserial.read(myserial.inWaiting())
		#if len(opm_string) >= 6:
		#	to_send_string = opm_string[5:len(opm_string)-2] 
		if len(opm_string) > 0:
			to_send_string = opm_string 
		else:
			to_send_string = ''

		#print to_send_string
		rospy.loginfo("["+"%s"+"]  "+"%s",current_milli_time(),to_send_string)
		pub.publish(to_send_string)
		rate.sleep()

'''callback function'''
def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	myserial.write(data.data)


if __name__ == '__main__':
	myserial = serial.Serial('/dev/ttyUSB0',115200,timeout=None)
	print myserial.portstr
	try:
		serial_node()
	except rospy.ROSInterruptException:
		pass
