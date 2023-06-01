import numpy as np
from __future__ import division
import sympy
import math
# input: Pvehicle_W = [X_vehicle, Y_vehicle], Pco_ca = [Xco_ca, Yco_ca, Zco_ca], row

def coordinateConvector(Pvehicle,Ppp_ca,row):
    x_diff = 180.0
    X_move = 2406.015 - Pvehicle[0] - x_diff # the distance between the center of the lidar and the robotic arm
    Y_move = Pvehicle[1]-0.0
    # transformation matrix of the camera to the vehicle base 
    # I need the angle of the camera
    T5_0   = np.array([0.9848,0.0,0.1736,94.5935],[0.0,-1.0,0.0,-71.5],[0.1736,0.0,-0.9848,399.2706],[0.0,0.0,0.0,1.0])
    # transformation matrix of the vehicle base to the origin coordinate
    T0_ori = np.array([[0.0,-1.0,0.0,X_move],
                       [1.0,0.0, 0.0,Y_move],
                       [0.0,0.0, 1.0,17.2],  # we need to measure this
                       [0.0,0.0, 0.0,1.0]])  
    Tca_ori = np.dot(T0_ori,T5_0)
    # the coordinent of the camera is left hand sys
    Ppp_ca_new = np.array([[Ppp_ca[1]],[Ppp_ca[0]],[Ppp_ca[2]],[1]])
    # transform the coordiate of pingpong ball in the camera to the origin 
    Ppp_ori = np.dot(Tca_ori,Ppp_ca_new)
    # get the distance up from the board
    height  = np.round(Ppp_ori[2]/25.4,1)
    
    # get which plant has the pingpong ball
    num_plant = math.round((X_move-590.55)/50.8)
    X_plant   = num_plant*50.8+590.55
    # 1
    if(Ppp_ori[0]-X_plant<=0):
        orientation = sympy.symbols('R')
    else:
        orientation = sympy.symbols('L')
    #distance = np.abs(Ppp_ori[0]-X_plant)

    # should consider the thick of the board
    if(row == 1):
        Y_plant = 641.35
    else:
        Y_plant = 641.35+(row-1)*585.79  # 585.7875 figure out the wide of the board
    
    # 2
    #if(Ppp_ori[1]-Y_plant<=0):
    #    orientation = sympy.symbols('R')
    #else:
    #    orientation = sympy.symbols('L')
    #distance = np.abs(Ppp_ori[1]-Y_plant)
    #distance = np.around(distance/25.4,2)
    # 3
    P_plant   = np.array([X_plant],[Y_plant],[0])
    S_plant   = np.array([0],[0],[0])
    Sol_plant = np.cross(P_plant,S_plant)
    d_pp      = np.cross(S_plant,Sol_plant-np.cross(Ppp_ori,S_plant))
    distance  = np.abs(d_pp)
    distance  = np.around(distance/25.4,1)
    Mapping   = 'R'+ str(row)+'-'+ 'P'+str(num_plant)+'-'+ orientation+str(distance) +'-'+'H'+str(height)
    return Mapping