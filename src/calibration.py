#!/usr/bin/env python 
import rospy, math, time,tf,serial
from ar_track_alvar_msgs.msg import AlvarMarkers
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class spherical_calibration:
	def __init__(self):
		self.serial_buffer=""
		self.s1=[];self.s2=[];self.s3=[];self.s4=[];self.t1=[];self.t2=[];self.t3=[];self.timer=[] #storage arrays
		rospy.init_node("Calibration_Node")
		self.sub_once=rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.tag_callback,queue_size = 1)
		rospy.on_shutdown(self.plot)
		self.ser=serial.Serial('/dev/ttyACM0',115200)
		self.file=open('datalog.csv','w')
		self.data_count=0

	def tag_callback(self,data):
		ids=map(lambda x: x.id, data.markers) #get foudn ids
		if 3 in ids:
			position=data.markers[0].pose.pose.position
			o=data.markers[0].pose.pose.orientation
			t11=math.degrees(math.atan(position.x/0.13))
			t22=math.degrees(math.atan(position.y/0.13))
			t33=yaw=tf.transformations.euler_from_quaternion([o.x,o.y,o.z,o.w])[2] #yaw
			self.timer.append(time.time())
			self.t1.append(t11)
			self.t2.append(t22)
			self.t3.append(t33)
			self.s1.append(self.curr_s1)
			self.s2.append(self.curr_s2)
			self.s3.append(self.curr_s3)
			self.s4.append(self.curr_s4)
			self.data_count += 1
			print(self.data_count,t11,t22,t33,self.curr_s1,self.curr_s2,self.curr_s3,self.curr_s4)
			self.file.write(str(t11)+','+str(t22)+','+str(t33)+','+str(self.curr_s1)+','+str(self.curr_s2)+','+str(self.curr_s3)+','+str(self.curr_s4)+'\n')

	def plot(self):
		self.sub_once.unregister()
		fig, ax1 = plt.subplots()
		t=np.array(self.timer)
		x=np.array(self.t2)
		y1=np.array(self.s1)
		y2=np.array(self.s2)
		y3=np.array(self.s3)
		y4=np.array(self.s4)		#z=np.array(t3)
		ax1.plot(t,x,'b')
		ax2 = ax1.twinx()
		ax2.plot(t,y1,'r')
		ax2.plot(t,y2,'g')
		ax2.plot(t,y3,'c')
		ax2.plot(t,y4,'m')
		plt.show()

	def read_serial(self):
		read=self.ser.read(self.ser.inWaiting())
		if len(read) > 0:
			self.serial_buffer += read
		else:
			return
		while self.serial_buffer.find('\r\n') > 0:
			data,self.serial_buffer=self.serial_buffer.split('\r\n',1)
			data=data.split('\t')
			if len(data) == 4:
				self.curr_s1=data[0]
				self.curr_s2=data[1]
				self.curr_s3=data[2]
				self.curr_s4=data[3]

	def spinner(self):
		while not rospy.is_shutdown():
			self.read_serial()

a=spherical_calibration()
a.spinner()
