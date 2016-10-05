#!/usr/bin/python
import rospy
import numpy as np
import sensor_msgs.msg
import std_msgs.msg
import geometry_msgs.msg
import openravepy as orpy

count = 0
trigger = False
along = []

class JointInfo(object):
	def __init__(self):
		super(JointInfo, self).__init__()
		
		rospy.Subscriber("/start_pos", geometry_msgs.msg.Point, self.start_pos)
		rospy.Subscriber("denso/joint_states", sensor_msgs.msg.JointState, self.joint_state)
		self.pub = rospy.Publisher('/Extruder_yaw',std_msgs.msg.Float64MultiArray, queue_size=1)

		self.env = orpy.Environment()
		self.env.SetViewer('qtcoin')
		env_loaded = self.env.Load("/home/sc3dp/catkin_ws/src/criprint/denso_vs087_description/worlds/proam_demo.env.xml")
		self.robot = self.env.GetRobots()[0]
		self.robot_name = rospy.get_param('~robot_name', 'denso')
		self.env.AddRobot(self.robot)
		self.manip_name = rospy.get_param('~manipulator', 'mai2pictor_extruder')
		self.manip = self.robot.SetActiveManipulator(self.manip_name)
		self.start_pos = np.array([0,0,0])


	def joint_state(self, joint):
		current_joint = np.asarray(joint.position)
		self.robot.SetDOFValues(current_joint)
		Tee = self.manip.GetEndEffectorTransform()
		Translation = Tee[0:3, 3]
		Rotation = Tee[0:3, 0:3]
		Trace = np.matrix.trace(Rotation)
		qw = np.sqrt((1 + Trace)/2.0)
		qx = (Rotation[2,1] - Rotation[1,2])/(4*qw)
		qy = (Rotation[0,2] - Rotation[2,0])/(4*qw)
		qz = (Rotation[1,0] - Rotation[0,1])/(4*qw)
		yaw = np.arctan(2* (qw * qz + qx * qy)/(1 - 2*(qy*qy + qz*qz))) * 180		# yaw angle

		err = np.linalg.norm(Translation - self.start_pos)
		tolerence = 1e-3
		global trigger, along
		# print 'Translation', Translation
		if trigger or err < tolerence:
			trigger = True
			global count
			count += 1
			if count == 1:
				along = Translation
			else:
				along = np.vstack((along, Translation))
				diff = (along[1] - along[0]).tolist()
				along = np.delete(along, 0, 0)[0]
				tangent = diff[1]/(diff[0]+10e-9)		# add 10e-9 to avoid divided by 0
			if not count%10:		# joint_state publishing rate is 200Hz. Drop to 20Hz for Wenxin's nozzle
				angle = std_msgs.msg.Float64MultiArray()
				angle.data = [yaw, tangent]
				# angle.tangent = tangent
				# angle.yaw = yaw
				self.pub.publish(angle)			# angle in the form of [yaw, tangent]


	def start_pos(self, init_pos):
		self.start_pos = [init_pos.x, init_pos.y, init_pos.z]

if __name__ == '__main__':
	rospy.init_node('wx_nozzle', anonymous=True)

	try:
		wx_nozzle = JointInfo()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
