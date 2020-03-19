#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
#from kinematics_header import *

from gt_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""
def Get_MS():
    # =================== Your code starts here ====================#
    # Fill in the correct values for a1~6 and q1~6, as well as the M matrix
    M = np.matrix([
    [0,-1,0,0.54],
    [0,0,-1,0.252],
    [1,0,0,0.202],
    [0,0,0,1]
    ])
    #S = np.zeros((6,6))
    w1 = np.matrix([0, 0, 1])
    q1 = np.matrix([0, 0, 0.152])
    w2 = np.matrix([0, 1, 0])
    q2 = np.matrix([0, 0.120, 0.152])
    w3 = np.matrix([0, 1, 0])
    q3 = np.matrix([0.244, 0.120, 0.152])
    w4 = np.matrix([0, 1, 0])
    q4 = np.matrix([0.457, 0.027, 0.152])
    w5 = np.matrix([1, 0, 0])
    q5 = np.matrix([0.457, 0.110, 0.152])
    w6 = np.matrix([0, 1, 0])
    q6 = np.matrix([0.540, 0.110, 0.152])
    v1 = -np.cross(w1, q1)
    v2 = -np.cross(w2, q2)
    v3 = -np.cross(w3, q3)
    v4 = -np.cross(w4, q4)
    v5 = -np.cross(w5, q5)
    v6 = -np.cross(w6, q6)

    s1 = np.append(w1, v1, axis=1)
    s2 = np.append(w2, v2, axis=1)
    s3 = np.append(w3, v3, axis=1)
    s4 = np.append(w4, v4, axis=1)
    s5 = np.append(w5, v5, axis=1)
    s6 = np.append(w6, v6, axis=1)

    S = np.append(s1, s2, axis=0)
    S = np.append(S, s3, axis=0)
    S = np.append(S, s4, axis=0)
    S = np.append(S, s5, axis=0)
    S = np.append(S, s6, axis=0)

    # ==============================================================#
    return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

    # Initialize the return_value
    return_value = [None, None, None, None, None, None]

    # =========== Implement joint angle to encoder expressions here ===========
    print("Foward kinematics calculated:\n")

    # =================== Your code starts here ====================#
    theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])

    #T = np.eye(4)

    M, S = Get_MS()


    T = np.eye(4)

    for s_idx in range(6):
        s = S[s_idx].tolist()[0]
        square_S = np.matrix([
        [0, -s[2], s[1], s[3]],
        [s[2], 0, -s[0], s[4]],
        [-s[1], s[0], 0, s[5]],
        [0, 0, 0, 0]
        ])

        T = np.matmul(T, expm(square_S * theta[s_idx]))

    T = np.matmul(T, M)

    # ==============================================================#

    print(str(T) + "\n")

    return_value[0] = theta1 + PI
    return_value[1] = theta2
    return_value[2] = theta3
    return_value[3] = theta4 - (0.5*PI)
    return_value[4] = theta5
    return_value[5] = theta6

    return return_value

"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):

   # theta1 to theta6
    thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    l01 = 0.152
    l02 = 0.120
    l03 = 0.244
    l04 = 0.093
    l05 = 0.213
    l06 = 0.083
    l07 = 0.083
    l08 = 0.082
    l09 = 0.0535
    l10 = 0.059   # thickness of aluminum plate is around 0.006

    xgrip = xWgrip + (148 / 1000.0)
    ygrip = yWgrip - (148 / 1000.0)
    zgrip = zWgrip - 0.01

    # xgrip = xWgrip
    # ygrip = yWgrip
    # zgrip = zWgrip

    yaw_Wgrip = np.radians(yaw_WgripDegree)

    xcen = xgrip - l09*np.cos(yaw_Wgrip)
    ycen = ygrip - l09*np.sin(yaw_Wgrip)
    zcen = zgrip

    # theta1
    alpha = np.arctan2(ycen, xcen)
    beta = np.arcsin((l02-l04+l06)/np.sqrt(xcen**2+ycen**2))
    thetas[0] = alpha - beta

    # theta6
    thetas[5] = PI/2 + thetas[0] - yaw_Wgrip

    v_cen_to_3end = np.matrix([-l07, -l06-(27/1000.0), l08 + l10]).transpose()

    R_theta1 = np.matrix([
                        [np.cos(thetas[0]), -np.sin(thetas[0]), 0],
                        [np.sin(thetas[0]), np.cos(thetas[0]), 0],
                        [0, 0, 1]
    ])

    rotated_v_cen_to_3end = np.matmul(R_theta1, v_cen_to_3end).transpose().tolist()[0]

    x3end = xcen + rotated_v_cen_to_3end[0]
    y3end = ycen + rotated_v_cen_to_3end[1]
    z3end = zcen + rotated_v_cen_to_3end[2]

    d = np.sqrt(x3end**2 + y3end**2 + (z3end - l01)**2)

    alpha = np.arctan2((z3end - l01), np.sqrt(x3end**2 + y3end**2))

    beta = np.arccos((l03**2 + d**2 - l05**2) / (2*l03*d))

    thetas[1]= -(alpha + beta)

    thetas[2]= PI - np.arccos((d**2 - l03**2 - l05**2)/ (-2*l03*l05))

    zeta = np.arccos((l05**2 + d**2 - l03**2)/ (2*l05*d))

    gamma = np.arctan2(np.sqrt(x3end**2 + y3end**2), (z3end - l01))

    thetas[3]= -(gamma+zeta-0.5*PI) # Default value Need to Change, need + (0.5*PI) for compensation

    thetas[4]=-PI/2

    #print("theta1 to theta6: " + str(thetas) + "\n")

    #return lab_fk(float(thetas[0] + PI), float(thetas[1]), float(thetas[2]), \
        #          float(thetas[3] - 0.5*PI), float(thetas[4]), float(thetas[5]) )
    return [float(thetas[0] + PI), float(thetas[1]), float(thetas[2]), \
                  float(thetas[3] - 0.5*PI), float(thetas[4]), float(thetas[5]) ]
