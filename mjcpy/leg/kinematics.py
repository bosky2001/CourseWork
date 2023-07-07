
import mujoco as mj
import numpy as np
from numpy.linalg import inv
from enum import Enum


class Params(Enum):
    M_hip = 5
    M_l1 = 0.5
    M_l2 = 0.5
    M_toe = 0.1
    M_total = M_hip + M_l1+ M_l2 + M_toe

    l1 = 1
    l2 = 1
    r1 = 0.05
    r2 = 0.05
    r_toe = 0.07

    I1 = (1/12)* M_l1*(l1**2 + 3*r1**2)
    I2 = (1/12)* M_l2*(l2**2 + 3*r2**2)
    I_toe = (2/5)*M_toe*r_toe**2
    
class Index(Enum):

    x = 0
    z = 1
    q1 = 2
    q2 = 3

def forward_kinematics(model,data):
    q1 = data.qpos[Index.q1.value]
    q2 = data.qpos[Index.q2.value]
    g= 1
    x = - Params.l1.value * np.sin(q1) - Params.l2.value * np.sin(q1+q2)
    y = 0
    z = -Params.l1.value * np.cos(q1) - Params.l2.value * np.cos(q1+q2)
    return np.array([x,y,z])

def foot_jacobian(model, data):
    J = np.zeros((3,2))
    q1 = data.qpos[Index.q1.value]
    q2 = data.qpos[Index.q2.value]
    J[0,0] = - Params.l1.value * np.cos(q1) - Params.l2.value * np.cos(q1+q2)
    J[0,1] = - Params.l2.value * np.cos(q1+q2)
    
    J[2,0] =  Params.l1.value * np.sin(q1)+ Params.l2.value * np.sin(q1+q2)
    J[2,1]=   Params.l2.value * np.sin(q1+q2)
    return J

def radial(model, data):
    q1 = data.qpos[Index.q1.value]
    q2 = data.qpos[Index.q2.value] 


   
    z  = forward_kinematics(model, data)[2]
    x = forward_kinematics(model, data)[0]

    l1 = Params.l1.value
    l2 = Params.l2.value

    r = np.sqrt(l1**2 + l2**2  -2*l1*l2*np.cos(np.pi - q2))

    theta = -np.arctan2(x, -z)
    # print("X:",x, "Z:",z)
    return r, theta

def Jq_r(model, data):
    J = np.zeros((2, 2))

    q1 = data.qpos[Index.q1.value]
    q2 = data.qpos[Index.q2.value]

    l1 = Params.l1.value
    l2 = Params.l2.value

    r,_ = radial(model, data)
    J[0,0] = 0
    J[0,1] = -l1*l2*np.sin(np.pi - q2)/ r

    J[1,0] = 1

    J[1,1] = 1/2
    return J