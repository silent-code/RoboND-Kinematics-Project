## Project: Kinematics Pick & Place
The Kinematics Pick and Place project introduced python0-based programming within the Robotic Operating System (ROS) environment and the use of the Kuka KR210 serial manipulator simulator to demonstrate basic forward and inverse kinmatic concepts. The next image is a screen capture of the simulation during the actual testing stage of the project, showing the robotic manipulator successfully placing the object into the target bin.

[//]: # (Image References)

[image1]: ./misc_images/robond-picknplace-gazebo-screencap.png
[image2]: ./misc_images/kuka_arm_annotated.png
[image3]: ./misc_images/rse_proj2_eqtns1.png
[image4]: ./misc_images/rse_proj2_eqtns2.png
The following is a screen cap of the arm placing an object in the basket.
![alt text][image1]

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.


![alt text][image2]

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | .75 | q1
1->2 | -90 | .35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -90 | -.054 | 1.5 | q4
4->5 | 90 | 0 | 0 | q5
5->6 | -90 | 0 | 0 | q6
6->EE | 0 | 0 | .303 | 0

#### 2. Using the DH parameter above, we can create individual transformation matrices about each joint. The individual joint transforms with the DH parameter substitutions are as follows:

T0_1 = [[cos(q1) -sin(q1) 0 0] <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[sin(q1) cos(q1) 0 0] <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[0 0 1 0.75] <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[0 0 0 1]]
 
T1_2 = [[cos(q2 - 0.5*pi) -sin(q2 - 0.5*pi) 0 0.35] <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;       [0 0 1 0] <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;       [-sin(q2 - 0.5*pi) -cos(q2 - 0.5*pi) 0 0] <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;       [0 0 0 1]]

T2_3 = [[cos(q3) -sin(q3) 0 1.25] <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;       [sin(q3) cos(q3) 0 0] <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;       [0 0 1 0] <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;       [0 0 0 1]]
 
T3_4 = [[cos(q4) -sin(q4) 0 -0.054] <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;       [0 0 1 1.5] <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;       [-sin(q4) -cos(q4) 0 0] <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;       [0 0 0 1]]
 
T4_5 = [[cos(q5) -sin(q5) 0 0] <br>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;      [0 0 -1 0] <br>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;      [sin(q5) cos(q5) 0 0] <br>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;      [0 0 0 1]]
 
T5_6 = [[cos(q6) -sin(q6) 0 0] <br>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;      [-sin(q6) -cos(q6) 0 0] <br>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;      [0 0 -1 0] <br>
 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;      [0 0 0 1]]
 
T6_Grip = [[1 0 0 0] <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;          [0 1 0 0] <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;          [0 0 1 0.303] <br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;          [0 0 0 1]]
 
#### 3. Next we were required to decouple the Inverse Kinematics problem into Inverse Position Kinematics and Inverse Orientation Kinematics and by doing so obtain the equations to calculate all individual joint angles.

First we needed to obtain the location of the wrist center position [WCx, WCy, WCz]. Since the Kuka has a spherical wrist, the wrist position and orientation with respect to the robot base are independent. The following diagram shows the wrist center position derivation.  

![alt text][image4]

Once we have the wrist center position it is fairly straight forward to derive the first three joint angles. Once those are obtained, one can use 
![alt text][image3]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  





