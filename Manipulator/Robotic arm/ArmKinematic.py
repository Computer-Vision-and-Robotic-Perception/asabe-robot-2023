from cmath import pi
import numpy as np
import math
import matplotlib.pyplot as plt
import time

def cos(degree):
    return math.cos(math.radians(degree))

def sin(degree):
    return math.sin(math.radians(degree))

def trans(theta,alpha,a,d):
	# get the transformation matrix
	T_alpha = np.array([[1, 0,          0,           0],
                        [0, cos(alpha), -sin(alpha), 0],
                        [0, sin(alpha), cos(alpha),  0],
                        [0, 0,          0,           1]])
	T_a     = np.array([[1, 0, 0, a],
                        [0, 1, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
	T_theta = np.array([[cos(theta), -sin(theta), 0, 0],
                        [sin(theta), cos(theta),  0, 0],
                        [0,               0,      1, 0],
                        [0,               0,      0, 1]])
	T_d     = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, d],
                        [0, 0, 0, 1]])
	T       = np.dot(np.dot(np.dot(T_alpha, T_a),T_theta),T_d)
	return T

def fkine(alpha,a,theta,d):
	# get the forward kinematics
	# angle is degree, length is mm, 
	n  = len(theta)
	An = np.eye(4)
	for i  in range(n):
		T  = trans(theta[i],alpha[i],a[i],d[i])
		An = np.dot(An,T)  # forth axis to the zero axis transformatrix
	return An

def Traj(t0,tf,angle0,anglef):
	# set the time interval and the initial angle and the final angle
	# output the step angle and average velocity for every step
	state = np.array([[angle0], [0.0], [0.0], [anglef], [0.0], [0.0]])
	T = np.array([[t0**5, t0**4, t0**3, t0**2, t0, 1.0],
				  [5*(t0**4), 4*(t0**3), 3*(t0**2), 2*t0, 1.0, 0.0],
				  [20*(t0**3), 12*(t0**2), 6*t0, 1.0, 0.0, 0.0],
				  [tf**5, tf**4, tf**3, tf**2, tf, 1.0],
				  [5*(tf**4), 4*(tf**3), 3*(tf**2), 2*tf, 1.0,0.0],
				  [20*(tf**3), 12*(tf**2), 6*tf, 1.0, 0.0, 0.0]])
	T = np.linalg.inv(T)
	A = np.dot(T,state)
	n = int((tf-t0)/0.05)+1
	path  = []
	vpath = []
	time = np.linspace(t0,tf,n)
	for t in time:
		th1  = np.dot(np.array([t**5, t**4, t**3, t**2, t, 1.0]),A)
		vth1 = np.dot(np.array([5*t**4, 4*t**3, 3*t**2, 2*t, 1.0, 0.0]),A)
		path.append(th1[0])
		vpath.append(vth1[0])
	angle  = []
	vangle = []
	for i in range(n-1):
		print(i)
		angle.append(path[i+1]-path[i])
		vangle.append((vpath[i+1]+vpath[i])/2)
	angle.append(0)
	vangle.append(0)	
	return angle,vangle
# inverse Kinematic   
def CottonArm(Pco_ca):
	# use geometric method to calculate the inverse kinematic
	# input the coordinate of the cotton in the frame of the camera
	alpha_arm = np.array([0.0,0.0,90.0,90.0]) # offset of the arm  x axis
	a_arm     = np.array([0.0,0.0,0.0,0.0])   # x axis
	d4        = 0.0
	d_arm     = np.array([244.0,172.0,113.5,d4]) # offset of the arm z axis
	theta1    = 0.0
	theta3    = 0.0
	theta_arm = np.array([theta1,0.0,theta3,0.0])

	# the angle of the camera unknown
	Pco_ca_new = np.array([[Pco_ca[1]],[Pco_ca[0]],[Pco_ca[2]],[1]]) # left hand to right hand
	alpha_cam  = np.array([0.0,0.0,0.0,90.0,90.0])
	a_cam      = np.array([0.0,0.0,0.0,87.5,0.0])
	d_cam      = np.array([244.0,172.0,23.5,71.5,40.85])
	theta4     = 10.0 # camera angle
	theta_cam  = np.array([0.0,0.0,0.0,theta4,0.0])
	Tca_0      = fkine(alpha_cam,a_cam,theta_cam,d_cam) # calculate the coordinate of the camera to the base frame
	# get the coordinate the the cotton in the frame of the 0 frame(base of the robot arm)
	Pco_0 = np.dot(Tca_0,Pco_ca_new)
	Xco_0 = Pco_0[0]
	Yco_0 = Pco_0[1]
	Zco_0 = Pco_0[2]

	# get the initial state of the robotic endeffector
	T4_0_arm_ini = np.array([[1.0,0.0,0.0,0.0],[0.0,-1.0,0.0,-113.5],[0.0,0.0,-1.0,416.0],[0.0,0.0,0.0,1.0]])
	Pend_4_ini   = np.array([[-39.6],[0.0],[214.5],[1.0]]) # need to adjust the position of the cotton
	# get the coordinate of the endeffector in the frame of the 0 frame at the beginning
	Tend_4_ini   = np.array([[1,0,0,Pend_4_ini[0]],[0,1,0,Pend_4_ini[1]],[0,0,1,Pend_4_ini[2]],[0,0,0,1]])
	Pend_0_ini   = np.dot(T4_0_arm_ini,Pend_4_ini)
	Yend_0       = Pend_0_ini[1]
	y            = Yend_0
	x            = math.sqrt(Xco_0*Xco_0+Yco_0*Yco_0-y*y)
	# use geometric method to get the first angle
	P1 = np.array([x,y,0])
	P1 = P1/np.linalg.norm(P1)
	P2 = np.array([Xco_0,Yco_0,0])
	P2 = P2/np.linalg.norm(P2)
	#theta1 = math.acos(np.dot(P1,P2))*180/pi
	theta_arm[0] = math.acos(np.dot(P1,P2))*180/pi
	if(Yend_0>Yco_0):
		theta_arm[0] = -theta_arm[0] # get the cotton located on the right
	T4_0_arm_new = fkine(alpha_arm,a_arm,theta_arm,d_arm)
	X4_0 = T4_0_arm_new[0,3]
	print('X4_0')
	print(X4_0)
	Y4_0 = T4_0_arm_new[1,3]
	Z4_0 = T4_0_arm_new[2,3]

	L1 = math.sqrt((Xco_0-X4_0)*(Xco_0-X4_0)+(Yco_0-Y4_0)*(Yco_0-Y4_0)+(Zco_0-Z4_0)*(Zco_0-Z4_0))
	L2 = math.sqrt(L1*L1-Pend_4_ini[0]*Pend_4_ini[0])
	P3 = np.array([Pend_4_ini[0],L2,0])
	P3 = P3/np.linalg.norm(P3)
	P4 = np.array([-(Z4_0-Zco_0),math.sqrt((Xco_0-X4_0)*(Xco_0-X4_0)+(Yco_0-Y4_0)*(Yco_0-Y4_0)),0])
	P4 = P4/np.linalg.norm(P4)
	theta3 = 90.0 - math.acos(np.dot(P3,P4))*180/pi
	theta_arm[2] = theta3
	print('theta3')
	print(theta3)
	d_arm[3]   = L2-Pend_4_ini[2] # need to test the center of the arm
	print(Pend_4_ini[2])
	print('d4')
	print(d_arm[3])
	print(L2)
	T4_0 = fkine(alpha_arm,a_arm,theta_arm,d_arm)
	print('T4_0')
	print(T4_0)
	Tend_0 = np.dot(T4_0,Tend_4_ini)
	print(Tend_0)
	return theta_arm[0],theta_arm[2], d_arm[3]

def CottonToTraj(theta1,theta3,d4):
	t0 = 0
	tf = 3
	theta10 = 0
	theta1f = theta1
	theta30 = 0
	theta3f = theta3
	d40     = 0
	d4f     = d4
	theta1, vtheta1 = Traj(t0,tf,theta10,theta1f)
	print('stop sign')
	theta3, vtheta3 = Traj(t0,tf,theta30,theta3f)
	d4, vd4         = Traj(t0,tf,d40,d4f)
	return theta1, vtheta1, theta3, vtheta3, d4, vd4
	
def CottonGetTraj(t0,tf,theta3,d4):
	theta30 = 0	
	theta3f = 90-theta3
	d40     = 0
	d4f     = -d4
	theta3, vtheta3 = Traj(t0,tf,theta30,theta3f)
	d4, vd4         = Traj(t0,tf,d40,d4f)
	theta50 = 0
	theta5f = 180
	theta5, vtheta5  = Traj(t0,tf-1,theta50,theta5f)
	return theta3, vtheta3, d4, vd4, theta5, vtheta5

def CottonPutTraj(t0,tf,theta1, theta3 = -30, d4 = 0,  theta5 = 180):
	# Input the seting angle, and put the cotton to the container
	theta1f = -45-theta1 # set the angle go back to the container place
	theta10 = 0
	theta1, vtheta1 = Traj(t0,tf,theta10,theta1f)
	theta30 = 0
	theta3f = theta3
	theta3, vtheta3 = Traj(t0,tf-0.5,theta30,theta3f)
	theta50 = 0
	theta5f = -theta5
	theta5, vtheta5 = Traj(t0,tf-1.5,theta50,theta5f)

	return theta1,vtheta1,theta3,vtheta3,theta5,vtheta5

def RemindRealSense(sig):
	if sig>0:
		# RealSense start work
		return Pco_ca
	break # jump out of the harvest function
		
def MainCotton():
	# 1, navigation go to the point
	num = 4 # difine to get four cottons
	while(num):
		# 2, RealSense detect the cotton (specific number)
		Pco_ca = RemindRealSense(num)
		# 3, Robotic arm work
		## 3.1, Inverse Kinematic get the 1st, third, fourth reference angle
		theta1,theta3,d4 = CottonArm(Pco_ca)

		## 3.2, Do trajectory to approach the cotton
		### 3.2.1 Do trajectory planning
		theta1, vtheta1, theta3, vtheta3, d4, vd4 = CottonToTraj(theta1,theta3,d4)
		### 3.2.2 Send the reference angle to the Arduino
		
		## 3.3 Get the signal from the arduino, do the harvest
		time.sleep(5)
		theta3, vtheta3, d4, vd4, theta5, vtheta5 = CottonGetTraj(0,3,theta3,d4)
		### 3.3.1 Send the reference angle to the Arduino

		## 3.4 Put cotton to the container
		theta1 = -45.0-theta1
		theta1,vtheta1,theta3,vtheta3,theta5,vtheta5 = CottonPutTraj(0,3,theta1, theta3 = -30.0, d4 = 0,  theta5 = 180)
		### 3.4.1 Sent the reference angle to the Arduio
		## 3.5 Robotic Arm go back to the origin point
		num = num-1







Pco_ca = np.array([100,4.5,150])
theta1,theta3,d4 = CottonArm(Pco_ca)
print(theta1)
print(theta3)
print(d4)
theta1, vtheta1, theta3, vtheta3, d4, vd4 = CottonToTraj(theta1,theta3,d4)
print('theta1')
print(sum(theta1))
plt.plot(theta3)
plt.show()
#def ContainerTraj():

 	#return ang


#def CottonHarvest():