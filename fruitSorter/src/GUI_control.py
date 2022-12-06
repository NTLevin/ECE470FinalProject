#!/usr/bin/env python3

from PyQt5 import QtWidgets
from control import Ui_MainWindow
import sys
import rospy
import rospkg
import os
import yaml
import random
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetModelState
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_conveyor.srv import ConveyorBeltControl, ConveyorBeltControlRequest
from cubeSpawner import CubeSpawner
import threading
from fruitSorter.msg import aprilTagMsg

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        rospy.init_node('ur3_gazebo_spawner', anonymous=True)
        self.cs = CubeSpawner()
        self.isSpawn = True
        
        self.getInfo()
        self.ui.pushButton_respawn.clicked.connect(self.respawn)
        self.ui.pushButton_conveyor_speed.clicked.connect(self.setConveyorSpeed)
        self.ui.pushButton_delete_all.clicked.connect(self.deleteAll)
        self.ui.pushButton_conveyor_stop.clicked.connect(self.stopConveyor)
        self.conveyorBeltControl = rospy.ServiceProxy('conveyor/control', ConveyorBeltControl)
        self.aprilTagID = 0
        self.aprilTagSub = rospy.Subscriber("apriltag", aprilTagMsg, self.aprilTagCallback)
        req = ConveyorBeltControlRequest()
        req.power = 15.0
        resp = self.conveyorBeltControl(req)
        

    def getInfo(self):
        # Initialize rospack
        self.rospack = rospkg.RosPack()
        # Get path to yaml
        lab2_path = self.rospack.get_path('lab2pkg_py')
        yamlpath = 'lab2_data.yaml'
        yamlpath = os.path.join(lab2_path, 'scripts', 'lab2_data.yaml')

        with open(yamlpath, 'r') as f:
            try:
                # Load the data as a dict
                data = yaml.load(f)
                # Load block position
                self.block_xy_pos = data['block_xy_pos']
                
            except:
                sys.exit()

        # Initialize ROS node
        # rospy.init_node('ur3_gazebo_spawner', anonymous=True)
        # Initialize ROS pack
        # Get path to block
        ur_path = self.rospack.get_path('ur_description')
        # block_path = os.path.join(ur_path, 'urdf', 'block.urdf')
        block_red_path = os.path.join(ur_path, 'urdf', 'block_red.urdf')
        block_red_big_path = os.path.join(ur_path, 'urdf', 'block_red.urdf')
        block_yellow_path = os.path.join(ur_path, 'urdf', 'block_yellow.urdf')
        # block_yellow_path = os.path.join(ur_path, 'urdf', 'block_yellow.urdf')
        block_green_path = os.path.join(ur_path, 'urdf', 'block_green.urdf')
        block_green_big_path = os.path.join(ur_path, 'urdf', 'block_green_big.urdf')
        self.block_paths = [block_red_path, block_yellow_path, block_green_path, block_green_big_path]
        # Wait for service to start
        rospy.wait_for_service('gazebo/spawn_urdf_model')
        self.spawn = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
        self.delete = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        self.getState = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

        self.cubeSpwanThread = threading.Thread(target=self.cubeSpwan)
        self.cubeSpwanThread.start()

    def respawn(self):
        # Delete previous blocks
        for height in range(3):
            block_name = 'block' + str(height + 1)
            self.delete(block_name)

        starting_location = 0

        # Spawn three blocks
        for height in range(3):
            block_name = 'block' + str(height + 1)
            pose = Pose(Point(self.block_xy_pos[starting_location][height][0], 
                            self.block_xy_pos[starting_location][height][1], 0.75), Quaternion(0, 0, 0, 0))
            self.spawn(block_name, open(self.block_paths[2-height], 'r').read(), 'block', pose, 'world')

    def setConveyorSpeed(self):
        self.isSpawn = True
        # rospy.wait_for_service('gazebo_conveyor/ConveyorBeltControl')
        conveyor_speed = float(self.ui.plainTextEdit_conveyor_speed.toPlainText())
        print(f'speed = {conveyor_speed}')

        req = ConveyorBeltControlRequest()
        req.power = conveyor_speed
        resp = self.conveyorBeltControl(req)
        rospy.loginfo(f"Set Conveyor Speed = {conveyor_speed}, Received response: {bool(resp.success)}")

    def stopConveyor(self):
        self.isSpawn = False
        req = ConveyorBeltControlRequest()
        req.power = 0
        resp = self.conveyorBeltControl(req)
        rospy.loginfo(f"Set Conveyor Speed = 0, Received response: {bool(resp.success)}")

    def cubeSpwan(self):
        r = rospy.Rate(0.5)
        rospy.on_shutdown(self.cs.shutdown_hook)
        while not rospy.is_shutdown():
            # if self.cs.checkModel() == False:
            if self.isSpawn:
                self.cs.spawnModel()
            # print("HI")
            if self.cs.getPosition() < 0.02:
                self.cs.deleteModel()
            r.sleep()

    def deleteAll(self):
        self.cs.deleteAll()

    def aprilTagCallback(self, msg):
        self.aprilTagID = msg.id
        if self.aprilTagID != 0:
            # print(f'aprilTagID = {self.aprilTagID}')
            self.ui.label_8.setText(str(self.aprilTagID))

if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())