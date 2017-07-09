[//]: # (Image References)

[image1]: ./misc_images/inverse11.png





---


# 2. Kuka Robotic Arm - Pick & Place : (Kinematic Analysis) Report

---
##### The main approach and steps are capture below, for further details please refer the code.
###
### 2.1 Building the Transform matrices 
* The first step is to Calculate the transformation matrix based on the Denavit–Hartenberg parameters and create a DH parameters table,this will help us in creating the matrices to calculate the individual transforms between the links. 
* The DH parameters table is shown below. We run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Joint | alpha (αi−1) | a (ai−1) | di | theta (θi)
--- | --- | --- | --- | ---
1 | 0 | 0 | 0.75 | 0
2 | -pi/2 | 0.35 | 0 | q2-pi/2
3 | 0 | 1.25 | 0 | 0
4 | -pi/2 | -0.054 | 1.50 | 0
5 | pi/2 | 0 | 0 | 0
6 | -pi/2 | 0 | 0 | 0
Gripper | 0 | 0 | 0.303| 0

![alt text][image1]

### Step1: Defining the DH params for Kuka KR210
    	    DH = {A0:     0, a0:      0, d1:  0.75,
          		  A1: -pi/2, a1:   0.35, d2:     0, q2: theta2-pi/2,
        		  A2:     0, a2:   1.25, d3:     0,
        		  A3: -pi/2, a3: -0.054, d4:  1.50,
        		  A4:  pi/2, a4:      0, d5:     0,
        		  A5: -pi/2, a5:      0, d6:     0,
        		  A6:     0, a6:      0, d7: 0.303, q7:           0
    		 }

Using this we need to construct the individual transform matrices as below.

### Step2: Creating the individual transformation matrices
###

##### Calculating the Rotation matrix from Homogeneous Transformation matrix, here we are using a generic function:
            def mat_transform(A,a,d,q):
                out = Matrix([[        cos(q),       -sin(q),       0,         a],
                              [ sin(q)*cos(A), cos(q)*cos(A), -sin(A), -sin(A)*d],
                              [ sin(q)*sin(A), cos(q)*sin(A),  cos(A),  cos(A)*d],
                              [             0,             0,       0,         1]])
                return out
       
##### then convert from Homogeneous transform matrix to rotation matrix and obtain system of equation and solve for q4,q5,q6 and further populate response for the IK request and replace theta1,theta2...,theta6 by your joint angle variables
    	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
    	    joint_trajectory_list.append(joint_trajectory_point)

---

## Step 3: Calculating the joint angles
* Since this is inverse kinematic problem we have two aspects: 
	- the first part is determining the postion of the end-effector this is done by simply calculating the angles for every joint **before** the wrist, 
	- and the orientation problem which is solved by calculating the angles for every joint **after** the wrist
* The wrist was determined to be in link 3 as joints 4, 5 and 6 are what give the enf-effector its orientation.

### Step4: Extracting end-effector position and orientation from request
	        # Extract end-effector position and orientation from request
    	    # px,py,pz = end-effector position
    	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])                   


---
### Step 5: Calculating Wrist Centre (WC) based on target roll, pitch and yaw angles 

           ## Calculate Rrpy Rotation Matrix given roll pitch and yaw
    	    R_x = Matrix([[ 1,            0,          0],
                		  [ 0,    cos(roll), -sin(roll)],
                		  [ 0,    sin(roll),  cos(roll)]])

    	    R_y = Matrix([[ cos(pitch),     0,  sin(pitch)],
                		  [          0,     1,           0],
                		  [-sin(pitch),     0,  cos(pitch)]])

    	    R_z = Matrix([[ cos(yaw), -sin(yaw),        0],
                		  [ sin(yaw),  cos(yaw),        0],
                		  [        0,         0,        1]])
---



### Project Implementation:
• Updated IK server.py file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. 

---