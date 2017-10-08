## Project: Kinematics Pick & Place

[//]: # (Image References)

[image1]: ./misc_images/img_1.png
[image2]: ./misc_images/img_2.png
[image3]: ./misc_images/img_3.png
[image4]: ./misc_images/img_4.png
[image5]: ./misc_images/img_5.png
[image6]: ./misc_images/img_6.png
[image7]: ./misc_images/img_7.png
[image8]: ./misc_images/img_8.png
[image9]: ./misc_images/img_9.png

---
### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

![alt text][image1]  
![alt text][image2]  

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0
  

alpha(i-1) | Twist angle | Angle from z(i-1) axis to z(i) axis measured along the x(i-1) axis
a(i-1) | Link length | Distance from z(i-1) axis to z(i) axis measured along the x(i-1) axis
d(i) | Link offset | Distance from x(i-1) axis to x(i) axis measured along the z(i) axis
theta(i) | Joint variable | Angle from x(i-1) axis to x(i) axis measured along the z(i) axis
  
> In joint 2, it has a constant offset -90 degree between x(1) and x(2).  

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

```python
## Create Modified DH parameters
dh_table = {alpha0:     0, a0:      0, d1:  0.75, q1:        q1,
			alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2 - pi/2,
			alpha2:     0, a2:   1.25, d3:     0, q3:        q3,
			alpha3: -pi/2, a3: -0.054, d4:   1.5, q4:        q4,
			alpha4:  pi/2, a4:      0, d5:     0, q5:        q5,
			alpha5: -pi/2, a5:      0, d6:     0, q6:        q6,
			alpha6:     0, a6:      0, d7: 0.303, q7:         0} 

## Create individual transform matrices
def create_TM(alpha, a, d, q):
	T = Matrix([[            cos(q),           -sin(q),           0,             a],   
                [ cos(alpha)*sin(q), cos(alpha)*cos(q), -sin(alpha), -sin(alpha)*d],
                [ sin(alpha)*sin(q), sin(alpha)*cos(q),  cos(alpha),  cos(alpha)*d],
                [                 0,                 0,           0,             1]])
	return T

T0_1 = create_TM(alpha0, a0, d1, q1).subs(dh_table)
T1_2 = create_TM(alpha1, a1, d2, q2).subs(dh_table)
T2_3 = create_TM(alpha2, a2, d3, q3).subs(dh_table)
T3_4 = create_TM(alpha3, a3, d4, q4).subs(dh_table)
T4_5 = create_TM(alpha4, a4, d5, q5).subs(dh_table)
T5_6 = create_TM(alpha5, a5, d6, q6).subs(dh_table)
T6_G = create_TM(alpha6, a6, d7, q7).subs(dh_table)


T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

```

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Step1. Complete the DH table.  
**Done.**  
  
Step2. Find the location of the WC relative to the base frame.  
![alt text][image3]  

Step3. Find the join angles(q1, q2,	q3).
![alt text][image4]  
![alt text][image5]  

Step4. Calculate (03)R via homogeneous transform.  

```python
T0_3 = T0_1 * T1_2 * T2_3 
R0_3 = T0_3[:3,:3]
```

Step5. Find euler angles(q4, q5, q6).  
![alt text][image6]  
![alt text][image7]  
![alt text][image8]  

Find q4, q5, q6.  

```python 
q4 = atan2(r33, -r13)
q5 = atan2(sqrt(r21**2 + r22**2), r23)
q6 = atan2(-r22, r21)

```

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

Here is my code to calculating Inverse Kinematics.  

```python
def rotation_matrix():
    r, p, y = symbols('r p y')

    R_x = Matrix([[   1,      0,       0],
                  [   0, cos(r), -sin(r)],
                  [   0, sin(r),  cos(r)]])

    R_y = Matrix([[ cos(p),  0, sin(p)],
                  [      0,  1,      0],
                  [-sin(p),  0, cos(p)]])

    R_z = Matrix([[ cos(y), -sin(y),  0],
                  [ sin(y),  cos(y),  0],
                  [      0,       0,  1]])

    R_G = R_z * R_y * R_x
    R_corr = R_z.subs(y, pi) * R_y.subs(p, -pi/2)
    R_G = R_G * R_corr
    
    return R_G

```

```python
def find_first_joints_angle(WC):
    a1, a2, a3 = 0.35, 1.25, -0.054
    d1, d4 = 0.75, 1.5

    theta1 = atan2(WC[1], WC[0])

    line_ab = a2 
    line_bc = sqrt(d4**2 + a3**2) 
    line_ca = sqrt((sqrt(WC[0]**2 + WC[1]**2) - a1)**2 + (WC[2] - d1)**2)

    angle_A = acos((line_ca**2 + line_ab**2 - line_bc**2) / (2 * line_ca * line_ab))
    angle_B = acos((line_bc**2 + line_ab**2 - line_ca**2) / (2 * line_bc * line_ab))

    gamma = atan2(WC[2] - d1, sqrt(WC[0]**2 + WC[1]**2) - a1)
    beta = atan2(d4, -a3)

    theta2 = pi/2 - angle_A - gamma
    theta3 = -(angle_B - beta)

    return theta1, theta2, theta3

def find_last_joints_angle(R):
    theta5 = atan2(sqrt(R[0,2]**2 + R[2,2]**2), R[1,2])

    if sin(theta5) < 0:
        theta4 = atan2(-R[2,2], R[0,2])
        theta6 = atan2(R[1,1], -R[1,0])
    else:
        theta4 = atan2(R[2,2], -R[0,2])
        theta6 = atan2(-R[1,1], R[1,0])

    return theta4, theta5, theta6

```

```python
R_G = rotation_matrix()

px = req.poses[x].position.x
py = req.poses[x].position.y
pz = req.poses[x].position.z

(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
		[req.poses[x].orientation.x, req.poses[x].orientation.y,
		req.poses[x].orientation.z, req.poses[x].orientation.w])

R_G = R_G.subs({'r': roll, 'p': pitch, 'y': yaw}) 
EE = Matrix([[px], [py], [pz]])
WC = EE - 0.303 * R_G[:,2]

theta1, theta2, theta3 = find_first_joints_angle(WC)
R0_3 = T0_3[:3,:3]
R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
R3_6 = R0_3.inv('LU') * R_G

theta4, theta5, theta6 = find_last_joints_angle(R3_6)

```

![alt text][image9]


