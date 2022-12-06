#!/usr/bin/env python3

import rospy, tf, rospkg, random
from gazebo_msgs.srv import DeleteModel, SpawnModel, GetModelState
from geometry_msgs.msg import Quaternion, Pose, Point
import random

class CubeSpawner():

	def __init__(self) -> None:
		self.rospack = rospkg.RosPack()
		self.path = self.rospack.get_path('fruitSorter')+"/urdf/"

		self.cubes = []
		self.cubes.append(self.path+"green_cube_apriltag.urdf")
		self.cubes.append(self.path+"red_cube_apriltag.urdf")
		self.cubes.append(self.path+"yellow_cube_apriltag.urdf")

		self.case = []
		self.case.append(self.path+"wooden_case.urdf")

		self.col = 0

		self.sm = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)
		self.dm = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
		self.ms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

		case = self.case[0]
		with open(case,"r") as f:
			case_urdf = f.read()
		
		quat = tf.transformations.quaternion_from_euler(1.57,-1.57,0)
		orient = Quaternion(quat[0],quat[1],quat[2],quat[3])
		pose = Pose(Point(x=0.3625, y=1.1, z=0.0), orient)
		self.dm("case1")
		# rospy.sleep(1)
		self.sm("case1", case_urdf, '', pose, 'world')

		quat = tf.transformations.quaternion_from_euler(1.57, -1.57, 1.57)
		orient = Quaternion(quat[0],quat[1],quat[2],quat[3])
		pose = Pose(Point(x=-0.7, y=0.15, z=0.0), orient)
		self.dm("case2")
		# rospy.sleep(1)
		self.sm("case2", case_urdf, '', pose, 'world')

		self.blockCounter = 0

	def checkModel(self):
		res = self.ms("cube", "world")
		return res.success

	def getPosition(self):
		res = self.ms("cube", "world")
		return res.pose.position.z

	def spawnModel(self):
		# print(self.col)
		radomCol = [0, 1, 2]
		randInt = random.choice(radomCol)

		cube = self.cubes[randInt]
		with open(cube,"r") as f:
			cube_urdf = f.read()
		
		quat = tf.transformations.quaternion_from_euler(0,0,0)
		orient = Quaternion(quat[0],quat[1],quat[2],quat[3])
		pose = Pose(Point(x=0.3625,y=-0.4,z=0.75), orient)
		self.sm("cube"+str(self.blockCounter), cube_urdf, '', pose, 'world')
		self.blockCounter += 1
		
		# if self.col<3:
		# 	self.col += 1
		# else:
		# 	self.col = 0

		rospy.sleep(5)

	def deleteModel(self):
		self.dm("cube")
		rospy.sleep(1)

	def shutdown_hook(self):
		self.deleteModel()
		print("Shutting down")

	def deleteAll(self):
		for i in range(self.blockCounter):
			self.dm("cube"+str(i))
		self.blockCounter = 0
		print("Deleted All")



if __name__ == "__main__":
	print("Waiting for gazebo services...")
	rospy.init_node("spawn_cubes")
	rospy.wait_for_service("/gazebo/delete_model")
	rospy.wait_for_service("/gazebo/spawn_urdf_model")
	rospy.wait_for_service("/gazebo/get_model_state")
	r = rospy.Rate(2)
	cs = CubeSpawner()
	rospy.on_shutdown(cs.shutdown_hook)
	while not rospy.is_shutdown():
		# if cs.checkModel()==False:
		cs.spawnModel()
		# elif cs.getPosition()<0.05:
			# cs.deleteModel()
		r.sleep()
		