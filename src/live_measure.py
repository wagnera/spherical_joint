#!/usr/bin/env python 
import rospy, math, time,tf,serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.externals import joblib
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf.transformations import quaternion_from_euler as eul2quat
from ar_track_alvar_msgs.msg import AlvarMarkers

class spherical_measure:
	def __init__(self):
		self.serial_buffer=""
		rospy.init_node("Measurement_Node")
		self.ser=serial.Serial('/dev/ttyACM0',115200)
		self.model1=joblib.load('axis1_model.sav')
		self.model2=joblib.load('axis2_model.sav')
		rospy.loginfo("Models Loaded")
		self.tf_br = tf2_ros.TransformBroadcaster()

		self.file=open('datalog.csv','w')
		self.sub_once=rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.tag_callback,queue_size = 1)
		self.t1=[];self.t2=[];self.t3=[];

	def read_serial(self):
		read=self.ser.read(self.ser.inWaiting())
		if len(read) > 0:
			self.serial_buffer += read
			#print(self.serial_buffer)
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
				self.theta1=theta_1
				self.theta2=theta_2
				#print(theta_1,theta_2)
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

	def tag_callback(self,data):
		ids=map(lambda x: x.id, data.markers) #get foudn ids
		if 3 in ids:
			position=data.markers[0].pose.pose.position
			o=data.markers[0].pose.pose.orientation
			t11=math.degrees(math.atan(position.x/0.13))
			t22=math.degrees(math.atan(position.y/0.13))
			t33=yaw=tf.transformations.euler_from_quaternion([o.x,o.y,o.z,o.w])[2] #yaw
			self.t1.append(t11)
			self.t2.append(t22)
			self.t3.append(t33)
			try:
				self.theta1
			except:
				return
			print("wrote to file")
			self.file.write(str(t11)+','+str(t22)+','+str(t33)+','+str(self.theta1)+','+str(self.theta2)+'\n')


	def spinner(self):
		while not rospy.is_shutdown():
			self.read_serial()

a=spherical_measure()
a.spinner()
