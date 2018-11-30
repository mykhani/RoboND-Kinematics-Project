[//]: # (Image References)

[image1]: ./misc_images/forward_kinematics_1.png
[image2]: ./misc_images/dh_parameter_analysis.png
[image3]: ./misc_images/dh_transform_matrix.png
[image4]: ./misc_images/dh_vs_urdf_orientation.png
[image5]: ./misc_images/theta_calculation1.png
[image6]: ./misc_images/theta_calculation2.png
[image7]: ./misc_images/wrist_center_calculation.png
[image8]: ./misc_images/orientation_gripper.png
[image9]: ./misc_images/final_result.png

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
#### 2. Generation of Transform matrices
##### Generating individual transform matrices from DH parameters
The general form of the DH parameter transform matrix is
![alt text][image3]

This can be implemented in python code as given below:
```python
def homogeneous_transform(q, d, a, alpha):
    Transform = Matrix([[            cos(q),           -sin(q),           0,             a],
                        [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                        [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                        [                 0,                 0,           0,             1]])
    return Transform
```
whereas q, d, a and alpha are sympy symbols. Individual transform matrices can be generated as below, substituting values of DH parameters:
```python
T0_1 = homogeneous_transform(q1, d1, a0, alpha0).subs(DH)
T1_2 = homogeneous_transform(q2, d2, a1, alpha1).subs(DH)
T2_3 = homogeneous_transform(q3, d3, a2, alpha2).subs(DH)
T3_4 = homogeneous_transform(q4, d4, a3, alpha3).subs(DH)
T4_5 = homogeneous_transform(q5, d5, a4, alpha4).subs(DH)
T5_6 = homogeneous_transform(q6, d6, a5, alpha5).subs(DH)
T6_G = homogeneous_transform(q7, d7, a6, alpha6).subs(DH)
```
##### Generating Homogeneous Transform from base_link to gripper_link using gripper pose
The important point to consider here is that the gripper pose is given in URDF convention while base_link to gripper_link homogeneous transform is based on DH convention. The two convention differ in their orientation as visible from the figure below:
![alt text][image4]
As mentioned in the lectures, the URDF frame can be converted to DH convention based frame, by first rotating the gripper (intrinsic rotation) orientation by 180 degrees around z-axis, followed by -90 degrees rotation around y-axis. Gripper pose consists of x,y,z positions and roll, pitch, yaw angles. The rotation matrix for the orientation of gripper can be calculated following a sequence of extrinsic rotations i.e.
```python
R_EE = Rot(z, yaw) * Rot(y, pitch) * Rot(x, roll) # Rotation matrix for end effector orientation
# The correction matrix to align URDF and DH orientation
R_corr = Rot(z, pi) * Rot(y, -pi/2)
R0_G = R_EE * R_corr # Rotation matrix from base_link to gripper_link
# The general form of Homogeneous Transform from base_link to gripper_link is
T0_G = Matrix([[r11, r12, r13, x]
               [r21, r22, r23, y]
               [r31, r32, r33, z]
               [  0,   0,   0, 1]])
# where r11, r12 etc are the elements of R0_G Rotation matrix and x, y, and z are gripper pose locations.
```
#### 3. Inverse kinematics
The problem of finding the joint angles can be simplified by breaking down the desired gripper pose in 2 steps: 
1. The location of wrist center is determined from gripper pose. The wrist center position is used to rotate joints 1, 2 and 3.
2. Once the desired wrist center location is achieved, orientation is adjusted to match the desired gripper orientation. Orientation thus is controlled by joints 4, 5 and 6, with joint 5 acting as the wrist center.
##### Determining the location of wrist center and angles for joints 1, 2 and 3
###### Calculating WC
Below is the equation for calculating the wrist center position.   
![alt text][image7]
The point to note is that the gripper reference frame and joint6 frame, both have the same orientation in DH coordinate system. The origin of gripper lies at distance "d", along the z-axis of joint6 reference frame in DH coordinates system. If we subtract "d" from gripper position along the z-axis of gripper frame, we get the wrist center position. Note that we need to account for the difference in orientation of gripper frame in DH and URDF convention and hence apply the required correction before calculating the wrist center position.
###### Calculating theta1
![alt text][image5]
###### Calculating theta2 and theta3
![alt text][image6]
##### Determining the required rotation for joints 4, 5 and 6
Below is the equation for determining the rotation matrix for the orientation of joint 4,5 and 6.
![alt text][image8]
The values of theta 1,2 and 3 as calculated earlier as used to compute transform from frame 0 to 3. The rotation matrix from frame 0 to 6 is the orientation of gripper frame after correction. After the rotation matrix from frame 3 to 6 is calculated, the euler angles are then extracted from the rotation matrix using formulas.
#### 3. Final result
![alt text][image9]
