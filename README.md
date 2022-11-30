# Fall22 UIUC ECE470 Project : Fruit Classifying Robot Arm

### Team Members : 
* Avram Fouad ***afouad2***
* Yu-An Su ***yuanas2***
* Miguel Belda Cambra ***miguelb4***
* Nathan Tyler Levin ***ntlevin2***

### Proposal :
The robot arm will classify fruits, based on their color and/or shape, from a conveyor belt and then places fruits in boxes according to their type. We will use camera to capture the real-time image of the conveyor, and then use OpenCV to detect the fruit’s color and/or shape. Finally, we plan to use the UR3 robot arm in Gazebo to pick up the fruit and place it respectively.

### Requirements :
1. There should be a specific spot on the conveyor belt where each fruit gets examined and classified by our camera.
2. The program should be able to correctly classify each fruit based on its color, or perhaps combination of color and shape .
3. There should be at least one box for each type of fruit.
4. The robot arm should grab and place each fruit in its corresponding box.

### Project Update 1 22/10/13 :
We created the virtual environment in Gazebo, simulated the UR3 robot arm and subscribed to the camera topic within ROS to display the camera sensor input. This was done by subscribing to the topic “/cv_camera_node/image_raw” found in the camera.urdf file. As seen in the video, the arm movement in the Gazebo environment is shown on the left and the camera input is shown on the right, evident with a slight delay. Later in the project this camera will be used to distinguish fruits based on their color and then the arm will move that specific fruit to its designated location. We plan to look into if pose estimation for the fruits is feasible for this to inform the robot gripper positioning, but at this point in the project implementation for pose estimation in Gazebo was not extensively explored.
 
Video of camera input & robot moving in Gazebo : <https://www.youtube.com/watch?v=XPslIs2DJBs&feature=youtu.be>

Having established a functioning Gazebo simulation environment, successfully accessed sensor inputs, and moved the robot, we feel confident our project is feasible. Should the implementation described above prove too simple in practice, we could add difficulty to the project by implementing pose estimation with R-CNN. 

source code is here : https://drive.google.com/file/d/1DdJ8mFw_B7jsP5NnZhcOFd3K9GMblF8v/view?usp=share_link

### Tools we're using in this project :
* Python
* ROS
* Gazebo
* OpenCV
* Tensorflow / PyTorch (if we need to increase difficulty)
