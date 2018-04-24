#!/usr/bin/env python 
import rospy, math, time,tf,serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.externals import joblib

class spherical_calibration:
	def __init__(self):
		self.serial_buffer=""
		rospy.init_node("Measurement_Node")
		self.ser=serial.Serial('/dev/ttyACM0',115200)
		self.model1=joblib.load('axis1_model.sav')
		self.model2=joblib.load('axis2_model.sav')
		rospy.loginfo("Models Loaded")

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
				"""self.curr_s1=data[0]
				self.curr_s2=data[1]
				self.curr_s3=data[2]
				self.curr_s4=data[3]"""
				measurement=np.array([[data[0],data[1],data[2],data[3]]]).astype(float)
				theta_1=self.model1.predict(measurement)[0]
				#print(theta_1)
				theta_2=self.model2.predict(measurement)[0]
				print(theta_1,theta_2)

	def spinner(self):
		while not rospy.is_shutdown():
			self.read_serial()

a=spherical_calibration()
a.spinner()
