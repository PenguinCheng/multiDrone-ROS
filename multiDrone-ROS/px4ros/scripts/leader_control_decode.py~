#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from px4ros.msg import gpsPosition

def decode_node():
	rospy.init_node('px4ros_leader_control_decode', anonymous=True)
	pub = rospy.Publisher('neighbour_position', gpsPosition, queue_size=20)
	rate = rospy.Rate(10) # 10hz

	rospy.Subscriber("serial_data_received", String, decodeInfo)
	
	position = gpsPosition()	
	while not rospy.is_shutdown():
		position.lat = neighbour_one_lat
		position.lon = neighbour_one_lon
		pub.publish(position)
		rospy.loginfo("%s",position)
		rate.sleep()

'''callback function'''
def decodeInfo(data):
	global neighbour_one_lon
	global neighbour_one_lat
	global last_neighbour_one_lon
	global last_neighbour_one_lat

	if data.data:
		latStr = ""
		lonStr = ""
		i = 3
		if data.data[0:3] == "x2o":        #for leader1, to receive the data of leader 2
			while i<len(data.data):
				if data.data[i] == "a":
					break
				lonStr += data.data[i]
				i += 1
			i += 1
			while i<len(data.data):
				if data.data[i] == "\n":
					break
				latStr += data.data[i]
				i += 1
		try:
			neighbour_one_lon = float(lonStr)
			neighbour_one_lat = float(latStr)

			last_neighbour_one_lon = neighbour_one_lon
			last_neighbour_one_lat = neighbour_one_lat
		except Exception, e:
			print "decode exception:", e
			neighbour_one_lon = last_neighbour_one_lon    #maybe 0.0
			neighbour_one_lat = last_neighbour_one_lat
	else:
		neighbour_one_lon = last_neighbour_one_lon    #maybe 0.0
		neighbour_one_lat = last_neighbour_one_lat
		print "not received neighbour information!"
	
if __name__ == '__main__':
	try:
		neighbour_one_lat = 0.0
		neighbour_one_lon = 0.0
		last_neighbour_one_lon = 0.0
		last_neighbour_one_lat = 0.0
	
		decode_node()
	except rospy.ROSInterruptException:
		pass


