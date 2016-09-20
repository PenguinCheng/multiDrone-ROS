#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from px4ros.msg import gpsPosition

def decode_node():
	rospy.init_node('px4ros_leader_control_decode', anonymous=True)
	pub = rospy.Publisher('neighbour_position', gpsPosition, queue_size=20)
	pub2 = rospy.Publisher('neighbour_position_2', gpsPosition, queue_size=20)
	rate = rospy.Rate(10) # 10hz

	rospy.Subscriber("serial_data_received", String, decodeInfo)
	
	position = gpsPosition()	
	position2 = gpsPosition()
	while not rospy.is_shutdown():
		position.lat = neighbour_one_lat
		position.lon = neighbour_one_lon
		pub.publish(position)
		rospy.loginfo("1:%s",position)

		position2.lat = neighbour_two_lat
		position2.lon = neighbour_two_lon
		pub.publish(position2)
		rospy.loginfo("2:%s",position2)

		rate.sleep()

'''callback function'''
def decodeInfo(data):
	global neighbour_one_lon
	global neighbour_one_lat
	global last_neighbour_one_lon
	global last_neighbour_one_lat
	global neighbour_two_lon
	global neighbour_two_lat
	global last_neighbour_two_lon
	global last_neighbour_two_lat

	if data.data:
		latStr = ""
		lonStr = ""
		latStr2 = ""
		lonStr2 = ""
		i = 3
		if data.data[0:3] == "x1o":        #receive the data of leader 1 and leader 2
			while i<len(data.data):
				if data.data[i] == "a":
					break
				lonStr += data.data[i]
				i += 1
			i += 1
			while i<len(data.data):
				if data.data[i] == "x":
					break
				latStr += data.data[i]
				i += 1
			#i +=1
			if data.data[i:i+3] == "x2o":
				i += 3  #skip x2o
				while i<len(data.data):
					if data.data[i] == "a":
						break
					lonStr2 += data.data[i]
					i += 1
				i += 1
				while i<len(data.data):
					if data.data[i] == "\n":
						break
					latStr2 += data.data[i]
					i += 1

			try:
				neighbour_one_lon = float(lonStr)
				neighbour_one_lat = float(latStr)

				last_neighbour_one_lon = neighbour_one_lon
				last_neighbour_one_lat = neighbour_one_lat

				neighbour_two_lon = float(lonStr2)
				neighbour_two_lat = float(latStr2)

				last_neighbour_two_lon = neighbour_two_lon
				last_neighbour_two_lat = neighbour_two_lat
			except Exception, e:
				print "decode exception:", e
				neighbour_one_lon = last_neighbour_one_lon    #maybe 0.0
				neighbour_one_lat = last_neighbour_one_lat
				neighbour_two_lon = last_neighbour_two_lon    #maybe 0.0
				neighbour_two_lat = last_neighbour_two_lat
		elif data.data[0:3] == "x2o":        #receive the data of leader 1 and leader 2
			while i<len(data.data):
				if data.data[i] == "a":
					break
				lonStr += data.data[i]
				i += 1
			i += 1
			while i<len(data.data):
				if data.data[i] == "x":
					break
				latStr += data.data[i]
				i += 1
			i +=1
			if data.data[i:i+3] == "x1o":
				i += 3  #skip x2o
				while i<len(data.data):
					if data.data[i] == "a":
						break
					lonStr2 += data.data[i]
					i += 1
				i += 1
				while i<len(data.data):
					if data.data[i] == "\n":
						break
					latStr2 += data.data[i]
					i += 1

			try:
				neighbour_one_lon = float(lonStr)
				neighbour_one_lat = float(latStr)

				last_neighbour_one_lon = neighbour_one_lon
				last_neighbour_one_lat = neighbour_one_lat

				neighbour_two_lon = float(lonStr2)
				neighbour_two_lat = float(latStr2)

				last_neighbour_two_lon = neighbour_two_lon
				last_neighbour_two_lat = neighbour_two_lat
			except Exception, e:
				print "decode exception:", e
				neighbour_one_lon = last_neighbour_one_lon    #maybe 0.0
				neighbour_one_lat = last_neighbour_one_lat
				neighbour_two_lon = last_neighbour_two_lon    #maybe 0.0
				neighbour_two_lat = last_neighbour_two_lat
	
	else:
		neighbour_one_lon = last_neighbour_one_lon    #maybe 0.0
		neighbour_one_lat = last_neighbour_one_lat
		neighbour_two_lon = last_neighbour_two_lon    #maybe 0.0
		neighbour_two_lat = last_neighbour_two_lat
		print "not received neighbour information!"
	
if __name__ == '__main__':
	try:
		neighbour_one_lat = 0.0
		neighbour_one_lon = 0.0
		last_neighbour_one_lon = 0.0
		last_neighbour_one_lat = 0.0
	
		neighbour_two_lat = 0.0
		neighbour_two_lon = 0.0
		last_neighbour_two_lon = 0.0
		last_neighbour_two_lat = 0.0
	
		decode_node()
	except rospy.ROSInterruptException:
		pass


