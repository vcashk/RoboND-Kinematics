[//]: # (Image References)

[image1]: ./misc_images/inverse11.png
[kuka1]: ./misc_images/kuka1.png
[fig1]: ./misc_images/fig1.png
[DH2]: ./misc_images/DH2.png
[DH3]: ./misc_images/DH3.png
[kukaaxis]: ./misc_images/kukaAxisDH.JPG
[kuka2]: ./misc_images/kuka2.png
[kuka3]: ./misc_images/kuka3.png
[kuka4]: ./misc_images/kuka4.png
[kuka5]: ./misc_images/kuka5.png
[kuka6]: ./misc_images/kuka6.png
[kuka7]: ./misc_images/kuka7.png
[kuka8]: ./misc_images/kuka8.png
[kuka9]: ./misc_images/kuka9.png
[kuka10]: ./misc_images/kuka10.png
[kuka11]: ./misc_images/kuka11.png
[DHTable]: ./misc_images/DHTable.jpg
[urdf]: ./misc_images/urdf.jpg
[wrist]: ./misc_images/wrist.jpg
[wrist2]: ./misc_images/wrist2.jpg
[kuka_workspace]: ./misc_images/kuka_workspace.png






---


# 2. Kuka Robotic Arm - Pick & Place : (Kinematic Analysis) Report

---
The robot in consideration is the Kuka KR120. Here are the details from Kuka Robot from www.kuka.com

![alt text][kuka1]

The objective of this assignment is to perform Kinematic Analysis both Direct and Inverse Robot Kinematics.
###
In general a 3D robot arm has n joints and n + 1 links. Numbering of links starts with 0 for the fixed base link to n for the end-effector link. Numbering of the joints start with 1 for the joint connecting the first movable link to the base link, and increases sequentially up to n. Therefor the link i is connected between the lower link i − 1 by joint i and the next link i+1 by joint i+1, as shown in Fig. 1 below:

![alt text][fig1]

the above Fig.1 also shows the links i−1, i and i+1 of a serial robot, along with joints i−1, i and i + 1. Every joint has an axis, which may be translational or rotational. To relate the kinematic information of the robot components, we attach a local coordinate frame Fi to each link i at joint i + 1 based on the Denavit-Hartenberg (DH) method:

![alt text][DH2]

#####

![alt text][DH3]

#####

With reference to figure above:

1. The zi-axis is aligned with the i + 1 joint axis.2. The xi-axis is defined along the common normal between the zi−1 and zi axes,pointing from the zi−1 to the zi-axis.3. The yi-axis is determined by the right-hand rule.In the DH method the origin oi of the frame Fi(oi, xi, yi, zi) attached to link i is placed at the intersection of the joint axis i + 1 with the common normal between the zi−1 and zi axes.####
Four parameters (ai, αi, di, θi), called the DH parameters, are used to define the geometry of a link i:

####
1. Link length ai: the distance along the xi-axis between the zi−1 and zi axes.2. Link twist αi: the rotation angle about the xi-axis between the zi−1 and zi axes.
3. Joint distance di: the distance along the zi−1-axis between the xi−1 and xi axes. 
4. Joint angle θi: the rotation angle about the zi−1-axis between the xi−1 and xi axes

Assumptions:

1. A manipulator is made up of links and joints (kinematic chain). The links are rigid and the joints are 1 dof, regardless of type (revolute or prismatic). We can always accomodate multi-dof (universal-type) joints by thinking that a link of length 0 sits between degrees of freedom (d = 0 in H).
2. There are n + 1 links, numbered from 0 to n, 0 being the base (assumed fixed in this course), and n being the end effector.
3. There are n joints, numbered from 1 to n. Joint i connects link i − 1 to link i.
4. When joint i is actuated, link i moves. Consistently with the numbering,nothing moves link 0.
5. To each joint there corresponds a single joint variable, qi, which is either the angle of rotation (revolute joint) or the displacement (prismatic joint).

Denavit-Hartenberg Convention:

1. Number the joints from 1 to n starting with base and ending with end-effector.
2. Establish the base Co-ordinate system
3. Establish joint axis
4. Establish origin of the ith co-ordinate system
5. Establish Xi and Yi axis by the by the right-hand rule.
6. Find the links and joints.


###
### 2.1 Building the Transform matrices 
* The first step is to Calculate the transformation matrix based on the Denavit–Hartenberg parameters and create a DH parameters table,this will help us in creating the matrices to calculate the individual transforms between the links. 
* The DH parameters table is shown below. We run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

### Kuka KR 120 URDF analysis:
Based on the information provided we have the the "Join Reference Frame" diagram below:

![alt text][kukaaxis]

In the above diagram the joint reference frame origins are indicated by O1...On in orientations as they appear in the URDF file. 

In the URDF file  each joint is defined relative to its parent joint. Hence inorder to get from joint 2, we have to 'translate' .35 meters in the 'X' direction and .42 meters in the Z direction.

Each row of the DH parameter table represents the homogeneous transform between frame {i-1} and frame {i}.Therefore we should incrementally check that the position of our frame origins is consistent with the cumulative displacements defined in the URDF file.

Based on the URDF file defination we map the following:
![alt text][urdf]

Next based on DH convention and method we :

1. Label Joints from 1 to nJoints labels were defined in URDF file. KUKA KR210 has 6DOFs, so n = 6
2. Define Joint axes: joint axes were defined in URDF file.
3. Label Links from 0 to n. Links were labeled and defined in URDF file. Here Link 0 is always a fixed base link.
4. Define Z axes directions: Z axes directions were obtained by running forward kinematics demo and determine positive Z axes directions based on Right-hand rule. (Images shown below DH parameter table)
5. Define X axes that are common normal to both Z(i-1) and Zi
Obtain DH parameters alpha, a, d, and theta, alpha: the angle between Z(i-1) and Z about X(i-1) using Right-hand rule.
here 'a': the offset from Z(i-1) to Zi along X(i-1)
and 'd': the offset from X(i-1) to Xi along Zi
theta: the angle between X(i-1) to Xi about Zi using Right-hand rule.

Following schematic digrams capture the axis of the various parts of the Kuka 210. This is to show how we are approaching the identification of the axis.

![alt text][kuka2]
![alt text][kuka3]
![alt text][kuka4]
![alt text][kuka5]
![alt text][kuka6]
![alt text][kuka7]
![alt text][kuka8]
![alt text][kuka9]
![alt text][kuka10]
![alt text][kuka11]

###

Since the Robot arm has 6 revolute joints, only the θ terms are time variable. The only complicated one is joint2. θ i is the angle between xi-1 and xi measured about the zi axis in a right-hand sense. The Robot arm is shown where all the joint angles are assumed to be 0.Therefore, x1 is not parallel to x2 when θ2 equals 0.There's a constant offset of -90 degrees.

To compare the total homogeneous transform between the base link and the gripper link, one needs to account for the difference in orientation of the gripper link frame. To do this is to apply a sequence of fix body that is intrinsic rotations to the gripper frame in the python code. To do so is to align the two frames, first rotate about the z axis by 180 degrees and then the y-axis by -90 degrees.

Following diagram provides the DH parameters and DH Table:

![alt text][DHTable]

#### Step1: Using the DH parameter table derived we create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between baselink and gripperlink using only end-effector(gripper) pose:

The transformation matrix is the combination of 2 rotations and 2 translations such as:

1. Rotation along the X axis by α
2. Translation along the X axis by a
3. Rotation along the Z axis by θ
4, Translation along Z axis by d

We can create a dictionary of our known DH parameter quantities, where

- α is the twist angles
- a is the link length
- d is the link offsets
- q is the joint variables, the θs



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
We can create the Homogeneous transforms between each neighbouring links. For example between link 0 and 1, 1 and 2, 2 and 3, all the way between link 6 in the Gripper G. Next, we can begin to incrementally build the total homogenous transform between the base link and the gripper frame , by incrementally post-multiplying individual homogenous transforms.

##### Calculating the Rotation matrix from Homogeneous Transformation matrix, here we are using a generic function:
            def dh_transformation(theta_x, d_dz, theta_z, d_dx):
    return Matrix([[cos(theta_z), -sin(theta_z), 0, d_dz],
                   [sin(theta_z) * cos(theta_x), cos(theta_z) * cos(theta_x), -sin(theta_x), -sin(theta_x) * d_dx],
                   [sin(theta_z) * sin(theta_x), cos(theta_z) * sin(theta_x), cos(theta_x), cos(theta_x) * d_dx],
                   [0, 0, 0, 1]])       

---

### Using this we create individual transformation matrices
        #between link i-1 and i
        t0_1 = dh_transformation(alpha0, a0, q1, d1).subs(s)
        t1_2 = dh_transformation(alpha1, a1, q2, d2).subs(s)
        t2_3 = dh_transformation(alpha2, a2, q3, d3).subs(s)
        t3_4 = dh_transformation(alpha3, a3, q4, d4).subs(s)
        t4_5 = dh_transformation(alpha4, a4, q5, d5).subs(s)
        t5_6 = dh_transformation(alpha5, a5, q6, d6).subs(s)
        t6_7 = dh_transformation(alpha6, a6, q7, d7).subs(s)
        t0_7 = simplify(t0_1 * t1_2 * t2_3 * t3_4 * t4_5 * t5_6 * t6_7)
----
#### Next is the orientation difference between definition of gripper link in URDF and the DH Convention.


#### Correction need to account of Orientation Difference
       def correction_matrix():
    r_z = Matrix([[cos(pi), -sin(pi), 0, 0],
                  [sin(pi), cos(pi), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    r_y = Matrix([[cos(-pi / 2), 0, sin(-pi / 2), 0],
                  [0, 1, 0, 0],
                  [-sin(-pi / 2), 0, cos(-pi / 2), 0],
                  [0, 0, 0, 1]])
    return simplify(r_z * r_y)
  
This is to account for the difference in orientation of the gripper link as it is defined in the URDF and the DH convention, we aplpy a body fixed rotation about the z axis and then the y axis. Therefore the R_correction is the composition of rorations of this z axis and y axis rotation

### 
From this correction, we get the Forward kinematics of the Robot arm and this maybe verified.

####  Step 3: Inverse Kinematic

The Inverse Kinematics is the opposite of Forwards kinematics. Meaning that the position and orientation of the end-effector is known and the goal is to calculate the joint angles of the Arm Robot. However for a Robot Arm with 6 joints, the overall transformation between the base and end effector involves 6 multiples of Equation which can result in a complicated non-linear equation.

####There are 2 ways to solve the Inverse Kinematic.
###

The first consist of purely numerical approach(For example Raphson Algorithm) which is to guess and iterate until the error is sufficiently small or the robot is such an idiot that you give up. 

###

The second approach is known as the closed-form solution. Closed_form solutions are specific algebraic equation or equations that do not require iteration to solve. 

###


It has two main advantages:

###

1. Quick to solve than numerical approaches
2. Easier to develop rules 
We need the following conditions:
	1.	Three neighboring joint axes intersect at a single point
	2.	Three neighboring joint axes are parallel
Our 6 Joints Robot Arm satisfy one of the above conditions.The last 3 joints 4, 5, 6 of our Robot Arm satisfy the first condition and this type of design is called Spherical Wrist and the point of intersection is called the Wrist Center.

For the Spherical Wrist and Wrist Center.
The location of the Wrist Center and End Effector relative to the base frame 0 is given by 0rwc/0 and 0rEE/0 respectively and the location of the End Effector to the Wrist Center is given by 0ree/wc as shown below.

![alt text][wrist2]

![alt text][wrist]

Hence as demostrated above we obtain:

wx = px - (d6 + l) * nx

wy = py - (d6 + l) * ny

wz = pz - (d6 + l) * nz


By providing the angle value for Joint 1(θ1), Joint 2(θ2) and Joint 3(θ3) into the transformation matrix we can get the orientation of the Robot Arm when Joint 4, joint 5 and joint 6 angles(θ4,θ5,θ6) is 0.





### Project Implementation:
• Updated IK server.py file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. 

### Further Improvement Possibilities:
The inverse kinematics calculation performed uses acos() and atan2() functions whose outputs are limited within 0 to pi and -pi to pi, respectively. 

However,workspace range for KUKA KR210 arm is more than what is defined by these numerical limitations asa shown below.

![alt text][kuka_workspace]

This can be further improved. We may apply various techniques to further reduce the error.

In the simulator the error may be different compared to the real world, the mechanical design specifications, the material used, the velocity, dynamics etc. may also have different outcomes and need to be considered.

We may also create a DeepLearning model to undrstand the same and evolve the model.


---