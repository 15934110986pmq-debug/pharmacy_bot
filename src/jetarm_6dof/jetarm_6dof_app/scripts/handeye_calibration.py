#!/usr/bin/env python3
# encoding: utf-8
# Date:2022/02/16
# Author:aiden
# 机械臂眼在手上标定(calibration of the robotic arm's camera on the hand)
import math
import numpy as np
import transforms3d as tfs

def rot2quat(r):
    quat = tfs.quaternions.mat2quat(r[0:3, 0:3])[1:]
    return quat

def quat2rot(q):
    p = np.dot(q.T, q)
    w = np.sqrt(np.subtract(1, p[0][0]))
    r = tfs.quaternions.quat2mat([w, q[0], q[1], q[2]])
    return r

def xyz_rpy2rot(x, y, z, roll, pitch, yaw):
    mat = tfs.euler.euler2mat(math.radians(roll), math.radians(pitch), math.radians(yaw))
    mat = tfs.affines.compose(np.squeeze(np.asarray((x, y, z))), mat, [1, 1, 1])
    
    return mat

def skew(v):
    return np.array([[ 0,    -v[2],  v[1]],
                     [ v[2],  0,    -v[0]],
                     [-v[1],  v[0],  0]])

def get_hand_camera_rot(hand, camera):
    hand_matrix = []
    camera_matrix = []
    if len(hand) == len(camera):
        for i in range(len(hand)):
            if len(hand[i]) == 6 and len(camera[i]) == 6:
                hand_matrix.append(xyz_rpy2rot(hand[i][0], 
                                                hand[i][1],
                                                hand[i][2],
                                                hand[i][3],
                                                hand[i][4],
                                                hand[i][5]))  
                camera_matrix.append(xyz_rpy2rot(camera[i][0], 
                                                camera[i][1],
                                                camera[i][2],
                                                camera[i][3],
                                                camera[i][4],
                                                camera[i][5])) 
    
    return hand_matrix, camera_matrix

def get_normalized(Hgs, Hcs):
    A, B = [], []
    Hgijs, Hcijs = [], []
    for i in range(len(Hgs)):
        for j in range(i+1, len(Hgs)):
            Hgij = np.dot(np.linalg.inv(Hgs[j]),Hgs[i])
            Hgijs.append(Hgij)
            Pgij = np.dot(2,rot2quat(Hgij))
            
            Hcij = np.dot(Hcs[j], np.linalg.inv(Hcs[i]))
            Hcijs.append(Hcij)
            Pcij = np.dot(2, rot2quat(Hcij))
            
            A.append(skew(np.add(Pgij, Pcij)))
            B.append(np.subtract(Pcij, Pgij))
    
    return Hgijs, Hcijs, A, B

def get_Rcg(A, B):
    MA = np.asarray(A).reshape(-1,3)
    MB = np.asarray(B).reshape(-1,1)

    Pcg_ = np.dot(np.linalg.pinv(MA), MB)
    pcg_norm = np.dot(np.conjugate(Pcg_).T, Pcg_)
    Pcg = np.sqrt(np.add(1, np.dot(Pcg_.T, Pcg_)))
    Pcg = np.dot(np.dot(2, Pcg_), np.linalg.inv(Pcg))
    Rcg = quat2rot(np.divide(Pcg, 2)).reshape(3, 3)

    return Rcg 

def get_Tcg(Hgs, Hgijs, Hcijs, Rcg):
    k = 0
    A = []
    B = []
    for i in range(len(Hgs)):
        for j in range(i+1,len(Hgs)):
            Hgij = Hgijs[k]
            Hcij = Hcijs[k]
            A.append(np.subtract(Hgij[0:3, 0:3], np.eye(3, 3)))
            B.append(np.subtract(np.dot(Rcg, Hcij[0:3, 3:4]), Hgij[0:3, 3:4]))
            k += 1
    MA = np.asarray(A).reshape(-1, 3)
    MB = np.asarray(B).reshape(-1, 1)
    Tcg = np.dot(np.linalg.pinv(MA), MB).reshape(3, )

    return Tcg

def get_cam2hand(hand, camera):
    Hgs, Hcs = get_hand_camera_rot(hand, camera)
    Hgijs, Hcijs, A, B = get_normalized(Hgs, Hcs)
    Rcg = get_Rcg(A, B)
    Tcg = get_Tcg(Hgs, Hgijs, Hcijs, Rcg)
    
    X = tfs.affines.compose(Tcg, np.squeeze(Rcg), [1, 1, 1])
    
    return X

if __name__ == '__main__':
    hand = [[1.1988093940033604, -0.42405585264804424, 0.18828251788562061, 151.3390418721659, -18.612399542280507, 153.05074895025035],
            [1.1684831621733476, -0.183273375514656, 0.12744868246620855, -161.57083804238462, 9.07159838346732, 89.1641128844487],
            [1.1508343174145468, -0.22694301453461405, 0.26625166858469146, 177.8815855486261, 0.8991159570568988, 77.67286224959672]]
    camera = [[-0.16249272227287292, -0.047310635447502136, 0.4077761471271515, -56.98037030812389, -6.16739631361851, -115.84333735802369],
              [0.03955405578017235, -0.013497642241418362, 0.33975949883461, -100.87129330834215, -17.192685528625265, -173.07354634882094],
              [-0.08517949283123016, 0.00957852229475975, 0.46546608209609985, -90.85270962096058, 0.9315977976503153, 175.2059707654342]]
    print(get_cam2hand(hand, camera))
