[//]: # (Image References)

[image1]: ./misc_images/forward_kinematics_1.png
[image2]: ./misc_images/dh_parameter_analysis.png

[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/robotics)
# Robotic arm - Pick & Place project Solution

For detailed steps for setting up the environment, please [see](https://github.com/udacity/RoboND-Kinematics-Project/blob/master/README.md)
### Kinematic Analysis
#### 1. Deriving DH parameters using the forward_kinematics demo and kr210.urdf.xacro file
Here is the image of link reference frames for kr210 in URDF convention:
![alt text][image1]

Here is the simplified diagram of arm joints, using DH convention:
![alt text][image2]

The unknown DH parameters from above image can be found by analyzing the URDF joint origins, from kr210.urdf.xacro file and then verified against the output of tf_echo.
##### DH parameter a1
DH parameter a1 is the distance between joint_1 and joint_2 along world x-axis i.e. 0.35.
```
robond@udacity:~/catkin_ws/src/RoboND-Kinematics-Project$ rosrun tf tf_echo link_1 link_2
At time 1543184980.155
- Translation: [0.350, 0.000, 0.420]
```
##### DH parameter d1
DH parameter d1 is the distance between base_link origin and joint_2 along world z-axis i.e. 0.75.
```
robond@udacity:~/catkin_ws/src/RoboND-Kinematics-Project$ rosrun tf tf_echo base_link link_2
At time 1543185367.054
- Translation: [0.350, 0.000, 0.750]
```
##### DH parameter a2
DH parameter a2 is the distance between joint_2 and joint_3 along world z-axis i.e. 1.25.
```
robond@udacity:~/catkin_ws/src/RoboND-Kinematics-Project$ rosrun tf tf_echo link_2 link_3
At time 1543185661.854
- Translation: [0.000, 0.000, 1.250]
```
##### DH parameter a3
DH parameter a3 is the distance between joint_3 and joint_5 (wrist center) along world z-axis i.e. -0.054.
```
robond@udacity:~/catkin_ws/src/RoboND-Kinematics-Project$ rosrun tf tf_echo link_3 link_5
At time 1543186229.655
- Translation: [1.500, 0.000, -0.054]
```
##### DH parameter d4
DH parameter a3 is the distance between joint_3 and joint_5 (wrist center) along world x-axis i.e. 1.5.
```
robond@udacity:~/catkin_ws/src/RoboND-Kinematics-Project$ rosrun tf tf_echo link_3 link_5
At time 1543186229.655
- Translation: [1.500, 0.000, -0.054]
```
##### DH parameter dg/d7
DH parameter dg or d7 is the distance between joint_5 (wrist center) and gripper_joint along world x-axis i.e. 0.303
```
robond@udacity:~/catkin_ws/src/RoboND-Kinematics-Project$ rosrun tf tf_echo link_5 gripper_link
At time 1543186439.355
- Translation: [0.303, 0.000, 0.000]
```
Here's the DH table defined in code:
```python
 # DH Parameters
        DH = { alpha0:      0,  a0:      0,  d1:   0.75,  q1:  q1,
               alpha1:  -pi/2,  a1:   0.35,  d2:      0,  q2:  q2-pi/2,
               alpha2:      0,  a2:   1.25,  d3:      0,  q3:  q3,
               alpha3:  -pi/2,  a3: -0.054,  d4:    1.5,  q4:  q4,
               alpha4:   pi/2,  a4:      0,  d5:      0,  q5:  q5,
               alpha5:  -pi/2,  a5:      0,  d6:      0,  q6:  q6,
               alpha6:      0,  a6:      0,  d7:  0.303,  q7:  0}
```
