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

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        rospy.init_node('ur3_gazebo_spawner', anonymous=True)

        self.getInfo()
        self.ui.pushButton_respawn.clicked.connect(self.respawn)
        self.ui.pushButton_conveyor_speed.clicked.connect(self.setConveyorSpeed)
        self.conveyorBeltControl = rospy.ServiceProxy('conveyor/control', ConveyorBeltControl)
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
        block_path = os.path.join(ur_path, 'urdf', 'block.urdf')
        block1_path = os.path.join(ur_path, 'urdf', 'block_red.urdf')
        block2_path = os.path.join(ur_path, 'urdf', 'block_yellow.urdf')
        block3_path = os.path.join(ur_path, 'urdf', 'block_green.urdf')
        self.block_paths = [block1_path, block2_path, block3_path]
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
                            self.block_xy_pos[starting_location][height][1], 0), Quaternion(0, 0, 0, 0))
            self.spawn(block_name, open(self.block_paths[2-height], 'r').read(), 'block', pose, 'world')

    def setConveyorSpeed(self):
        # rospy.wait_for_service('gazebo_conveyor/ConveyorBeltControl')
        conveyor_speed = float(self.ui.plainTextEdit_conveyor_speed.toPlainText())
        print(f'speed = {conveyor_speed}')

        req = ConveyorBeltControlRequest()
        req.power = conveyor_speed
        resp = self.conveyorBeltControl(req)
        rospy.loginfo(f"Set Conveyor Speed = {conveyor_speed}, Received response: {bool(resp.success)}")

    def cubeSpwan(self):
        r = rospy.Rate(15)
        cs = CubeSpawner()
        rospy.on_shutdown(cs.shutdown_hook)
        while not rospy.is_shutdown():
            if cs.checkModel() == False:
                cs.spawnModel()
            elif cs.getPosition() < 0.05:
                cs.deleteModel()
            r.sleep()

if __name__ == '__main__':
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())