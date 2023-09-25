# Through the door - Open Door Using Toyota HSR

### &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; Push open the door &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Pull open the door
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;![CBFFB0ED-234E-45A6-BD4D-B59667E8C141](https://github.com/HBRS-SDP/ss23-door_open/assets/47587089/a55c0534-9eb5-4d66-b825-07582c359031)
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;![1295C3D0-0FCA-4D1E-AE50-43908924A982](https://github.com/HBRS-SDP/ss23-door_open/assets/47587089/896b1d15-31a9-4963-bd02-3fdf33db53a4)

ERL Smart City Competition [(ERL Smart City)](https://eu-robotics.net/eurobotics/activities/european-robotics-league/) includes an episode in which the robot must open a hinged door with different configurations.​

## Aim:  To open the doors using a robotic arm from the inside and outside of the C069 lab.​

## Approach:​

We present an overview of our approach, illustrated in the diagram below:

<img width="582" alt="overall_flow" src="https://github.com/HBRS-SDP/ss23-door_open/assets/47587089/512cbb90-8f47-4f9a-aad0-11a883e213cf">





### 

## Project Description

text 

## File Description 

text

## Requirements

Before using this code, ensure that the following prerequisites are met:

* ROS (Robot Operating System) is installed and configured.
* Python is installed (compatible with ROS).
* ROS packages such as std_msgs, geometry_msgs, sensor_msgs, tmc_control_msgs, tmc_manipulation_msgs, and tmc_msgs are installed.
* Python packages from `requirements.txt` file

## Usage

To utilize this code for robot door-opening project, follow these steps:

#### Initial Setup:

Ensure all prerequisites are met as mentioned in the "Requirements" section.
### Run get_force.py:

Execute the following command to run the get_force.py script:

    python3 get_force.py
This script will initialize the force sensor capture and continuously monitor force and torque sensor data.

### For Unlatching the Door:

To unlatch the door, run the door_open_with_feedback.py script. This script provides the necessary commands or actions to unlatch the door.

    python3 door_open_with_feedback.py

### For Push/Pull Action:

For push/pull action, execute the new_move_with_cmd.py script. This script performs the push or pull action.

    python3 new_move_with_cmd.py


## About Object Detection

YOLOv5 Handle Detection Approach
  <img width="582" alt="overall_flow" src="https://github.com/HBRS-SDP/ss23-door_open/assets/104180642/abe1901a-c5e1-403b-8fd5-219ab5ff7893">

#### 2D Detection
- **Initial approach:**
  - Trained using the `MiguelARD/DoorDetect-Dataset`.
  - Faced challenges when detecting using Lucy's camera.
- **What worked:**
  - Trained on 500+ images of door handles taken from Lucy's head RGBD camera.
  - Result: Obtained bounding box coordinates for the door handle.



#### 3D Detection
- **Approach:**
  - Depth point cloud superimposed on the 2D image.
  - Used depth value for the centroid of the handle based on the coordinates of the bounding box from the previous step.
- **Result:**
  - Obtained bounding box coordinates for 3D detection.

## Logic 
#### Grasp the door handle
* After obtaining the 3D coordinates of the door handle using a detection algorithm

* Utilize motion planning to move the arm to the specified coordinates with fixed orientation

* But we encountered issues with motion planning did not work as expected, so we manually set and hardcode the joint angles to guide the arm to the door handle

* Once the robotic arm reaches the door handle, close the gripper to firmly grasp the handle

#### Unlatch door

<img width="423" alt="cw_acw_flow" src="https://github.com/HBRS-SDP/ss23-door_open/assets/47587089/45514bf1-d996-4632-88b0-7d487627c9a3">


* Once the door handle is grasped, continuously monitor force feedback

* If the torque in the x-component exceeds 0.5 Nm, initiate a clockwise rotation of the wrist

* If the torque does not exceed 0.5 Nm, go anti-clockwise instead

* After determining the rotation direction, begin rotating the wrist in the selected direction

* While rotating, gradually pull down on the door handle to completely unlatch it

#### Push/Pull

<img width="423" alt="cw_acw_flow" src="https://github.com/HBRS-SDP/ss23-door_open/assets/47587089/149e4b81-32a9-48fb-a42b-f1be17e7055e">



* After unlatching the door handle, the default action is to pull the door

* Calculate the directional force component resulting from the average components of y and z

* We verify if the force exceeds 45N, and if it does, we activate the door pull mechanism

* For pulling
    * If the directional force does not exceed 45N, continue pulling
    
    * Move backward with linear velocity until the directional force is less than –30degree
    
    * Once the force reaches -30degree, initiate lateral movement (left or right) based on the logic established during the door unlatching process
    
    * Continue moving left/right until the directional force reaches +15degree
    
    * After reaching +15degree, go back again until the force reaches -30degree, and repeat this loop
    
    * When the odom in the y-direction reaches a distance of 0.5 meters from the starting point, stop the movement and release the door handle

* For pushing

  * Begin by pushing the door slightly forward while keeping hold of the door handle

  * After pushing the door, release the door handle

  * Move the robot arm back to its home position

  * Initiate lateral movement (left or right) to create some distance from the wall

  * Finally, use the robot's body to apply force and push the door fully open, allowing the robot to pass through the doorway.





## References

* [ERL Smart City Rules](https://drive.google.com/drive/folders/1JdzsFmyMRUSpbQqrBfvmKB-dQl6YlOJ6)

