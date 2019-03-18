

## Kuka Pick and Place Project

[//]: # (Image References)
[DH]: ./misc_images/DH_kuka.png

Objectives: Utilize a simulated Kuka KR210 manipulator to pick up randomly generatedcylinders located on a shelf 
and place them in a bucket

### Denavit-Hartenberg Diagram
![Denavit-Hartenberg Diagram Kuka KR210][DH]
 * reference: Udacity Lesson 11 KR210 Forward Kinematics

### Denavit-Hartenberg Table

| n |  theta |   d   |    a   | alpha |
|:-:|:------:|:-----:|:------:|:-----:|
| 0 |   -    |   -   |    0   |   0   |
| 1 | theta1 |  0.75 |  0.35  | -pi/2 |
| 2 | theta2 |   0   |  1.25  |   0   |
| 3 | theta3 |   0   | -0.054 | -pi/2 |
| 4 | theta4 |  1.5  |    0   |  pi/2 |
| 5 | theta5 |   0   |    0   | -pi/2 |
| 6 | theta6 | 0.303 |    0   |   0   |

* theta parameters may change depending on arm orientation
* a and alpha are specific to the KR210
* because the joints are revolute d parameters will not change

## Joint Transformation Matrices
TF_Matrix function calculates each joints Transformation matrix relative to prior joint positions.

To use, feed the DH parameters for each joint int othe TF_Matrix function

```
def TF_Matrix(alpha, a, d, q):
    TF = Matrix([[cos(q), -sin(q), 0, a],
                 [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
                 [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
                 [0, 0, 0, 1]])
    return TF
```
The code listed below implements the TF_Matrix and creates individual transformation matrices for each joint
```
    T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
    T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
    T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
    T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
    T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
    T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
    T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)
    
    T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

 ```

### Inverse Kinematics

See link for inverse kinematics writeup [here](https://github.com/ejbkdb/RoboND_Kinematics_Project/blob/master/misc_images/inverse_kinematics.txt)
   
   Below is the code used to caluclate the inverse kinematics:
   
   ```
     # Calculate Wrest Center
  WC = EE - (0.303) * ROT_EE[:, 2]

  # Calculate joint angles using Geometric IK method
  # Calculate theat1
  theta1 = atan2(WC[1], WC[0])

  # find the 3rd side of the triangle
  B = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))

  # Cosine Laws SSS to find all inner angles of the triangle
  a = acos((-0.6875 + B * B) / (2.5 * B))
  b = acos((3.8125 - B * B) / (3.75))
  c = acos((0.6875 + B * B) / (3.0 * B))

  # Find theta2 and theta3
  theta2 = pi / 2 - a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
  theta3 = pi / 2 - (b + 0.036)  # 0.036 accounts for sag in link4 of -0.054m
  ```
 See link for inverse kinematics rotation sequenc [here](https://github.com/ejbkdb/RoboND_Kinematics_Project/blob/master/misc_images/rotation_sequenc.txt)

Below is the code used to caluclate the inverse kinematics rotation sequence:
```
  R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
  R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

  # Get rotation matrix R3_6 from (transpose of R0_3 * R_EE)
  R3_6 = R0_3.transpose() * ROT_EE

  # Euler angles from rotation matrix
  theta5 = atan2(sqrt(R3_6[0, 2] * R3_6[0, 2] + R3_6[2, 2] * R3_6[2, 2]), R3_6[1, 2])

  # select best solution based on theta5
  if (theta5 > pi):
      theta4 = atan2(-R3_6[2, 2], R3_6[0, 2])
      theta6 = atan2(R3_6[1, 1], -R3_6[1, 0])
  else:
      theta4 = atan2(R3_6[2, 2], -R3_6[0, 2])


      theta6 = atan2(-R3_6[1, 1], R3_6[1, 0])
 ```
 

 



