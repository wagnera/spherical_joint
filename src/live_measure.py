#!/usr/bin/env python 
import rospy, math, time,tf,serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.externals import joblib
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf.transformations import quaternion_from_euler as eul2quat

class spherical_measure:
	def __init__(self):
		self.serial_buffer=""
		rospy.init_node("Measurement_Node")
		self.ser=serial.Serial('/dev/ttyACM0',115200)
		self.model1=joblib.load('axis1_model.sav')
		self.model2=joblib.load('axis2_model.sav')
		rospy.loginfo("Models Loaded")
		self.tf_br = tf2_ros.TransformBroadcaster()

	def read_serial(self):
		read=self.ser.read(self.ser.inWaiting())
		if len(read) > 0:
			self.serial_buffer += read
			print(self.serial_buffer)
		else:
			return
		while self.serial_buffer.find('\r\n') > 0:
			data,self.serial_buffer=self.serial_buffer.split('\r\n',1)
			data=data.split('\t')
			if len(data) == 4:
				"""self.curr_s1=data[0]
				self.curr_s2=data[1]
				self.curr_s3=data[2]
				self.curr_s4=data[3]"""
				measurement=np.array([[data[0],data[1],data[2],data[3]]]).astype(float)
				theta_1=self.model1.predict(measurement)[0]
				#print(theta_1)
				theta_2=self.model2.predict(measurement)[0]
				print(theta_1,theta_2)
				t = TransformStamped()
				t.header.stamp = rospy.Time.now()
				t.header.frame_id='base_link'
				t.child_frame_id='cup'
				q=eul2quat(math.radians(-theta_1),math.radians(theta_2),0)
				t.transform.rotation.x=q[0]
				t.transform.rotation.y=q[1]
				t.transform.rotation.z=q[2]
				t.transform.rotation.w=q[3]
				self.tf_br.sendTransform(t)

	def spinner(self):
		while not rospy.is_shutdown():
			self.read_serial()

a=spherical_measure()
a.spinner()
