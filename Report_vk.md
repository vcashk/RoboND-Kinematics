[//]: # (Image References)

[image1]: ./misc_images/inverse11.png





---


# 2. Robotic arm - Pick & Place : Reports

---

## 2.1 Building the Transform matrices 
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

###Defining the modified DH Transformation matrix

        s = {alpha0: 0,     a0:   0,    d1: 0.75,
             alpha1: -pi/2, a1: 0.35,   d2: 0,       q2: q2 - pi/2,
             alpha2: 0,     a2: 1.25,   d3: 0,
             alpha3: -pi/2, a3: -0.054, d4: 1.5,
             alpha4: pi/2,  a4: 0,      d5: 0,
             alpha5: -pi/2, a5: 0,      d6: 0,
             alpha6: 0,     a6: 0,      d7: 0.303,   q7: 0}

* Using this we need to construct the individual transform matrices, which are shown below:

### Creating the individual transformation matrices
        T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
           [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
           [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
           [                   0,                   0,            0,               1]])

        T0_1 = T0_1.subs(s)

        T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
           [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
           [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
           [                   0,                   0,            0,               1]])

        T1_2 = T1_2.subs(s)

        T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
           [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
           [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
           [                   0,                   0,            0,               1]])

        T2_3 = T2_3.subs(s)

        T3_4 = Matrix([[             cos(q4),            -sin(q4),            0,              a3],
           [ sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
           [ sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
           [                   0,                   0,            0,               1]])

        T3_4 = T3_4.subs(s)

        T4_5 = Matrix([[             cos(q5),            -sin(q5),            0,              a4],
           [ sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
           [ sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
           [                   0,                   0,            0,               1]])

        T4_5 = T4_5.subs(s)

        T5_6 = Matrix([[             cos(q6),            -sin(q6),            0,              a5],
           [ sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
           [ sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
           [                   0,                   0,            0,               1]])

        T5_6 = T5_6.subs(s)

        T6_7 = Matrix([[             cos(q7),            -sin(q7),            0,              a6],
           [ sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
           [ sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
           [                   0,                   0,            0,               1]])

        T6_7 = T6_7.subs(s)

       
###Next Step is to transforming from the base link to end effector
        T0_7 = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_7)

---

## 2.2 Calculating the joint angles
* Since this is inverse kinematic problem we have two aspects: 
	- the first part is determining the postion of the end-effector this is done by simply calculating the angles for every joint **before** the wrist, 
	- and the orientation problem which is solved by calculating the angles for every joint **after** the wrist
* The wrist was determined to be in link 3 as joints 4, 5 and 6 are what give the enf-effector its orientation.

### Extracting end-effector position and orientation from request
	        # px,py,pz = end-effector position
	        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
                   


---
###Calculating Wrist Centre (WC) the construct R0_6 based on target roll, pitch and yaw angles of end-effector

            R_roll = Matrix([[ 1,              0,        0],
                          [ 0,        cos(roll), -sin(roll)],
                          [ 0,        sin(roll),  cos(roll)]])

            R_pitch = Matrix([[ cos(pitch),        0,  sin(pitch)],
                          [       0,        1,        0],
                          [-sin(pitch),        0,  cos(pitch)]])

            R_yaw = Matrix([[ cos(yaw), -sin(yaw),        0],
                          [ sin(yaw),  cos(yaw),        0],
                          [ 0,              0,        1]])

            R0_6 = simplify(R_roll * R_pitch * R_yaw)
---

###Calculating wrist centre (WC) based on translation along z-axis from EE location

            P_EE = Matrix([[px],[py],[pz]])

            P_WC = simplify(P_EE - 0.303 * R0_6 * Matrix([[1],[0],[0]]))

            J5 = P_WC
---

#### Then calculating theta1,2,3; the EE orientation  and Calculating R0_3
####
Since the end effector of the robot arm is rigidly attached to the robots wrist center hence the orientation of the end effector is the same as the one of the wrist center further as per what we learned in Lesson 2.18 we can extract the Euler angles of the wrist center.

            theta1 = atan2(J5[1], J5[0])
            #print("theta1",theta1)

            J2__0 = [0.35, 0, 0.75]
            J3__0 = [0.35, 0, 2]
            J5__0 = [1.85, 0, 1.946]

            J2 = [J2__0[0] * cos(theta1), J2__0[0] * sin(theta1), J2__0[2]]

            L2_5_X = J5[0] - J2[0]
            L2_5_Y = J5[1] - J2[1]
            L2_5_Z = J5[2] - J2[2]
            L2_5 = sqrt(L2_5_X**2 + L2_5_Y**2 + L2_5_Z**2)

            L2_3__0 = 1.25

            L3_5_X__0 = J5__0[0] - J3__0[0]
            L3_5_Z__0 = J5__0[2] - J3__0[2]
            L3_5__0 = sqrt(L3_5_X__0**2 + L3_5_Z__0**2)

            #Calculating theta3

            #D = cos(theta)
            D = (L2_5**2 - L2_3__0**2 - L3_5__0**2) / -(2 * L2_3__0 * L3_5__0)

            theta3_internal = atan2(sqrt(1-D**2), D)
            theta3 = pi / 2 - (atan2(sqrt(1-D**2), D) - atan2(L3_5_Z__0, L3_5_X__0))
            theta3_2 = pi / 2 - (atan2(-sqrt(1-D**2), D) - atan2(L3_5_Z__0, L3_5_X__0))
            #q3_1 = atan2(sqrt(1-D**2), D)
            #q3_2 = atan2(-sqrt(1-D**2), D)
            #print("theta3", theta3.evalf())
            #print("q3_2", q3_2.evalf())

            # Calculating theta 2

            #q2 = atan2(L2_5_Z, L2_5_X) - atan2(L3_5__0 * sin(pi - q3_internal), L2_3__0 + L3_5__0 * cos(pi - q3_internal))
            m1 = L3_5__0 * sin(theta3_internal)
            m2 = L2_3__0 - L3_5__0 * cos(theta3_internal)
            b2 = atan2(m1,m2)
            b1 = atan2(J5[2]-J2[2], sqrt((J5[0]-J2[0])**2 + (J5[1]-J2[1])**2))
            theta2 = pi / 2 - b2 - b1

            ##Calculating the EE orientation  and Calculating R0_3

            # Evaluatting with calculated q1, q2 & q3
            R0_3_num = R0_3.evalf(subs={q1:theta1, q2:theta2, q3:theta3})
            #rospy.loginfo("R0_3_num", R0_3_num)

            #calculating inverse of R0_3
            R0_3_num_inv = R0_3_num ** -1
            R3_6 = R0_3_num_inv * R0_6
            theta6 = atan2(R3_6[1,0],R3_6[0,0]) # rotation about Z-axis
            theta5 = atan2(-R3_6[2,0], sqrt(R3_6[0,0]*R3_6[0,0]+R3_6[1,0]*R3_6[1,0])) # rotation about Y-axis
            theta4 = atan2(R3_6[2,1],R3_6[2,2]) # rotation about X-axis

            # Populating response for the IK request
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)
---

###Project Implementation:
• Updated IK server.py file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. 

---