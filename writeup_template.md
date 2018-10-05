## Project: Kinematics Pick & Place
This projected demonstrated that we could do math from the 1980s.

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | L1 | qi
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 0 | 0
3->4 |  0 | 0 | 0 | 0
4->5 | 0 | 0 | 0 | 0
5->6 | 0 | 0 | 0 | 0
6->EE | 0 | 0 | 0 | 0

T0_1 = [[cos(q1) -sin(q1) 0 0]
 [sin(q1) cos(q1) 0 0]
 [0 0 1 0.750000000000000]
 [0 0 0 1]]
 
T1_2 = [[cos(q2 - 0.5*pi) -sin(q2 - 0.5*pi) 0 0.350000000000000]
 [0 0 1 0]
 [-sin(q2 - 0.5*pi) -cos(q2 - 0.5*pi) 0 0]
 [0 0 0 1]]

T2_3 = [[cos(q3) -sin(q3) 0 1.25000000000000]
 [sin(q3) cos(q3) 0 0]
 [0 0 1 0]
 [0 0 0 1]]
 
T3_4 = [[cos(q4) -sin(q4) 0 -0.0540000000000000]
 [0 0 1 1.50000000000000]
 [-sin(q4) -cos(q4) 0 0]
 [0 0 0 1]]
 
[[cos(q5) -sin(q5) 0 0]
 [0 0 -1 0]
 [sin(q5) cos(q5) 0 0]
 [0 0 0 1]]
 
[[cos(q6) -sin(q6) 0 0]
 [-sin(q6) -cos(q6) 0 0]
 [0 0 -1 0]
 [0 0 0 1]]
 
[[1 0 0 0]
 [0 1 0 0]
 [0 0 1 0.303000000000000]
 [0 0 0 1]]
 
[[ -(-((-sin(q3)*sin(q2 - 0.5*pi)*cos(q1) + cos(q1)*cos(q3)*cos(q2 - 0.5*pi))*cos(q4) + sin(q1)*sin(q4))*sin(q5) + (-sin(q3)*cos(q1)*cos(q2 - 0.5*pi) - sin(q2 - 0.5*pi)*cos(q1)*cos(q3))*cos(q5))*sin(q6) + (((-sin(q3)*sin(q2 - 0.5*pi)*cos(q1) + cos(q1)*cos(q3)*cos(q2 - 0.5*pi))*cos(q4) + sin(q1)*sin(q4))*cos(q5) + (-sin(q3)*cos(q1)*cos(q2 - 0.5*pi) - sin(q2 - 0.5*pi)*cos(q1)*cos(q3))*sin(q5))*cos(q6)
  -(-((-sin(q3)*sin(q2 - 0.5*pi)*cos(q1) + cos(q1)*cos(q3)*cos(q2 - 0.5*pi))*cos(q4) + sin(q1)*sin(q4))*sin(q5) + (-sin(q3)*cos(q1)*cos(q2 - 0.5*pi) - sin(q2 - 0.5*pi)*cos(q1)*cos(q3))*cos(q5))*cos(q6) - (((-sin(q3)*sin(q2 - 0.5*pi)*cos(q1) + cos(q1)*cos(q3)*cos(q2 - 0.5*pi))*cos(q4) + sin(q1)*sin(q4))*cos(q5) + (-sin(q3)*cos(q1)*cos(q2 - 0.5*pi) - sin(q2 - 0.5*pi)*cos(q1)*cos(q3))*sin(q5))*sin(q6)
  -(-sin(q3)*sin(q2 - 0.5*pi)*cos(q1) + cos(q1)*cos(q3)*cos(q2 - 0.5*pi))*sin(q4) + sin(q1)*cos(q4)
  -0.303*(-sin(q3)*sin(q2 - 0.5*pi)*cos(q1) + cos(q1)*cos(q3)*cos(q2 - 0.5*pi))*sin(q4) + 0.303*sin(q1)*cos(q4) + 0.054*sin(q3)*sin(q2 - 0.5*pi)*cos(q1) - 1.5*sin(q3)*cos(q1)*cos(q2 - 0.5*pi) - 1.5*sin(q2 - 0.5*pi)*cos(q1)*cos(q3) - 0.054*cos(q1)*cos(q3)*cos(q2 - 0.5*pi) + 1.25*cos(q1)*cos(q2 - 0.5*pi) + 0.35*cos(q1)]
 [ -(-((-sin(q1)*sin(q3)*sin(q2 - 0.5*pi) + sin(q1)*cos(q3)*cos(q2 - 0.5*pi))*cos(q4) - sin(q4)*cos(q1))*sin(q5) + (-sin(q1)*sin(q3)*cos(q2 - 0.5*pi) - sin(q1)*sin(q2 - 0.5*pi)*cos(q3))*cos(q5))*sin(q6) + (((-sin(q1)*sin(q3)*sin(q2 - 0.5*pi) + sin(q1)*cos(q3)*cos(q2 - 0.5*pi))*cos(q4) - sin(q4)*cos(q1))*cos(q5) + (-sin(q1)*sin(q3)*cos(q2 - 0.5*pi) - sin(q1)*sin(q2 - 0.5*pi)*cos(q3))*sin(q5))*cos(q6)
  -(-((-sin(q1)*sin(q3)*sin(q2 - 0.5*pi) + sin(q1)*cos(q3)*cos(q2 - 0.5*pi))*cos(q4) - sin(q4)*cos(q1))*sin(q5) + (-sin(q1)*sin(q3)*cos(q2 - 0.5*pi) - sin(q1)*sin(q2 - 0.5*pi)*cos(q3))*cos(q5))*cos(q6) - (((-sin(q1)*sin(q3)*sin(q2 - 0.5*pi) + sin(q1)*cos(q3)*cos(q2 - 0.5*pi))*cos(q4) - sin(q4)*cos(q1))*cos(q5) + (-sin(q1)*sin(q3)*cos(q2 - 0.5*pi) - sin(q1)*sin(q2 - 0.5*pi)*cos(q3))*sin(q5))*sin(q6)
  -(-sin(q1)*sin(q3)*sin(q2 - 0.5*pi) + sin(q1)*cos(q3)*cos(q2 - 0.5*pi))*sin(q4) - cos(q1)*cos(q4)
  -0.303*(-sin(q1)*sin(q3)*sin(q2 - 0.5*pi) + sin(q1)*cos(q3)*cos(q2 - 0.5*pi))*sin(q4) + 0.054*sin(q1)*sin(q3)*sin(q2 - 0.5*pi) - 1.5*sin(q1)*sin(q3)*cos(q2 - 0.5*pi) - 1.5*sin(q1)*sin(q2 - 0.5*pi)*cos(q3) - 0.054*sin(q1)*cos(q3)*cos(q2 - 0.5*pi) + 1.25*sin(q1)*cos(q2 - 0.5*pi) + 0.35*sin(q1) - 0.303*cos(q1)*cos(q4)]
 [ ((sin(q3)*sin(q2 - 0.5*pi) - cos(q3)*cos(q2 - 0.5*pi))*sin(q5) + (-sin(q3)*cos(q2 - 0.5*pi) - sin(q2 - 0.5*pi)*cos(q3))*cos(q4)*cos(q5))*cos(q6) - ((sin(q3)*sin(q2 - 0.5*pi) - cos(q3)*cos(q2 - 0.5*pi))*cos(q5) - (-sin(q3)*cos(q2 - 0.5*pi) - sin(q2 - 0.5*pi)*cos(q3))*sin(q5)*cos(q4))*sin(q6)
  -((sin(q3)*sin(q2 - 0.5*pi) - cos(q3)*cos(q2 - 0.5*pi))*sin(q5) + (-sin(q3)*cos(q2 - 0.5*pi) - sin(q2 - 0.5*pi)*cos(q3))*cos(q4)*cos(q5))*sin(q6) - ((sin(q3)*sin(q2 - 0.5*pi) - cos(q3)*cos(q2 - 0.5*pi))*cos(q5) - (-sin(q3)*cos(q2 - 0.5*pi) - sin(q2 - 0.5*pi)*cos(q3))*sin(q5)*cos(q4))*cos(q6)
  -(-sin(q3)*cos(q2 - 0.5*pi) - sin(q2 - 0.5*pi)*cos(q3))*sin(q4)
  -0.303*(-sin(q3)*cos(q2 - 0.5*pi) - sin(q2 - 0.5*pi)*cos(q3))*sin(q4) + 1.5*sin(q3)*sin(q2 - 0.5*pi) + 0.054*sin(q3)*cos(q2 - 0.5*pi) + 0.054*sin(q2 - 0.5*pi)*cos(q3) - 1.25*sin(q2 - 0.5*pi) - 1.5*cos(q3)*cos(q2 - 0.5*pi) + 0.75]
 [0 0 0 1]]


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


