import mujoco as mj
from mujoco import _functions
from mujoco.glfw import glfw
import numpy as np
from numpy.linalg import inv
import os
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from enum import Enum
import time


from momentum_obs import *
xml_path = 'leg2.xml'
simend = 6
animate = False

step_no = 0
THETA_FF = -0.055
modes = []     
z_height = []

applied_x = []
applied_y = []
applied_z = []

mode = 0

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0
error_i = np.zeros(2)

contact_x = []
contact_y = []
contact_z = []

obs_x =[]
obs_y =[]
obs_z =[]

contact_force_tol = 50
theta_des_SLIP = THETA_FF


def get_comPos(model, data):

    q1 = data.qpos[Index.q1.value]
    q2 = data.qpos[Index.q2.value]

    l1 = Params.l1.value
    l2 = Params.l2.value

    m_hip = Params.M_hip.value
    m_l1 = Params.M_l1.value
    m_l2 = Params.M_l2.value
    m_toe = Params.M_toe.value

    I_l1 = Params.I1.value
    I_l2 = Params.I2.value
    I_t = Params.I_toe.value

    q1dot = data.qvel[Index.q1.value]
    q2dot = data.qvel[Index.q2.value]

    #World Frame
    xhip = data.qpos[Index.x.value]
    zhip = data.qpos[Index.z.value]

    xdot = data.qvel[Index.x.value]
    zdot = data.qvel[Index.z.value]
    #Body Frame
    # xhip = 0
    # zhip = 0

    xcom = (xhip*m_hip + (xhip -(1/2)*l1*np.sin(q1))*m_l1 + (xhip -l1*np.sin(q1) \
           - (1/2)*l2*np.sin(q1+q2))*m_l2 + (xhip -l1*np.sin(q1)  \
           - l2*np.sin(q1 + q2))*m_toe) / Params.M_total.value
    
    zcom = (zhip*m_hip + (zhip -(1/2)*l1*np.cos(q1))*m_l1 + (zhip -l1*np.cos(q1) \
           - (1/2)*l2*np.cos(q1+q2))*m_l2 + (zhip -l1*np.cos(q1)  \
           - l2*np.cos(q1 + q2))*m_toe) / Params.M_total.value
    
    xcomdot = -0.5*(l1*(m_l1 + 2*(m_l2 + m_toe))*np.cos(q1) \
                    + l2*(m_l2 + 2*m_toe)*np.cos(q1 + q2))*q1dot \
                    - 0.5*l2*(m_l2 + 2*m_toe)*np.cos(q1+q2)*q2dot \
                    + Params.M_total.value*xdot
    
    zcomdot = 0.5*(l1*(m_l1 + 2*(m_l2 + m_toe))*np.sin(q1) \
                    + l2*(m_l2 + 2*m_toe)*np.sin(q1 + q2))*q1dot \
                    + 0.5*l2*(m_l2 + 2*m_toe)*np.sin(q1+q2)*q2dot \
                    + Params.M_total.value*zdot

    return xcom,zcom, xcomdot, zcomdot

def pid_controller(model, data):
    """
    This function implements a controller that
    mimics the forces of a fixed joint before release
    """
    global error_i
    kp = 0.5*np.array([7,14])
 
    kd = np.array([1,2])
    ki = np.array([0.004, 0.01])*0
    q = np.array([data.qpos[Index.q1.value], data.qpos[Index.q2.value]])
    qdes = np.array([-0.4, 0.8])
    vdes = np.zeros(2)
    v = np.array([data.qvel[Index.q1.value], data.qvel[Index.q2.value]])
    error_i += qdes-q
    tau = np.multiply(kp,qdes-q) + np.multiply(kd,vdes-v) + np.multiply(ki,error_i)

    jac_foot = np.zeros((3, model.nv))
    mj.mj_jacSubtreeCom(model, data, jac_foot, model.body('foot').id)
    J = foot_jacobian(model, data)
    # print(J)
    # print("and")
    # print(jac_foot)
    # print("------------------------")
    ff = -J.T @ np.array([0,0,-6.5* 9.81])
    data.ctrl[0] = tau[0] - ff[1]
    data.ctrl[1] =  tau[1]- ff[2]



def SLIP_flight(model, data, rdes = 1.7, theta_des = -0.2):
    
    r, theta = radial(model, data)
    J = Jq_r(model, data)
    dR = J @ data.qvel[Index.q1.value:]
    g =  np.array([-9.81 * (Params.M_total.value-Params.M_hip.value), 0 ])
    kp = 20
    kd = 30
    
   
    f = np.array([kp*(rdes - r) + kd*(-dR[0]), 80*(theta_des - theta) +10*(-dR[1]) ])

    tau = J.T@ f 
 

    data.ctrl[0] = tau[0]
    data.ctrl[1] = tau[1]

def Toe_control(model, data, zdes):
    z = forward_kinematics(model, data)[2]
    Jtoe = foot_jacobian(model, data)

    zdot = Jtoe @ data.qvel[Index.q1.value:]

    f = np.array([0, 0, 150*(zdes- z) + 50*(-zdot[2])])
    g =  np.array([0, 0, -9.81 * Params.M_total.value ])
    
    tau = np.transpose(Jtoe)@(f+g)

    data.ctrl[0] = tau[0]
    data.ctrl[1] = tau[1]
    


def SLIP_stance(model, data):
    q1 = data.qpos[Index.q1.value]
    q2 = data.qpos[Index.q2.value]
    
    r, theta = radial(model, data)
    #zdot = data.qvel[Index.z.value]
    J = Jq_r(model, data)
    dR = J @ data.qvel[Index.q1.value:]
    #nominal height
    p = 1.5
    w = 23
    phi = np.arctan2( w*(p-r),-dR[0])
    
    beta = 1.5
    ka = 30
    
    #fx = 25*(-x)+ 15*(-dX[0])
    E = w*w*(p-r)- beta*dR[0] -ka*np.cos(phi)
    u = np.array([E,  0])  

    g =  np.array([ -9.81 * Params.M_total.value,0])
    U = u - g
    U[0] = np.max([0.0,U[0]])
    tau = J.T@ U

    
    data.ctrl[0] = tau[0]
    data.ctrl[1] = tau[1]

def touchdown_angle(xdot,rtd = 1.5,xd = 0):
    T = 0.2
    k_x = 0.15
    print (xdot)

    return np.arcsin((-xdot*T/2 + k_x*(xd-xdot))/rtd)

body_x_velocity = []
com_pos = []
contact = []

def SLIP(model, data, con = 1):
    tol = 7*1e-2
    stance_h = 1.7
    
    #v_flight(model, data, stance_h)
    g =  np.array([9.81 * Params.M_total.value, 0])
    J = Jq_r(model, data)
    dR = J @ data.qvel[Index.q1.value:]
    global mode
    # mode 0 - flight/touchdown
    # mode 1 - stance
 
    r,_ = radial(model, data)
    
    delta_z = abs(r-stance_h)
    global THETA_FF
    global theta_des_SLIP
    vel = foot_jacobian(model, data) @ data.qvel[Index.q1.value:]
    
    xdot = -vel[0]
    
    #momentum observer
    mom_obs = momentum_observer(model= model, data= data)
    obs_x.append(mom_obs[0])
    obs_y.append(mom_obs[1])
    obs_z.append(mom_obs[2])
    k = 20
    #print(vel[0])


    
    modes.append(mode)
    #z_height.append(xdot)
    #xcom,_,xcomdot, zcomdot = get_comPos(model, data)
    forcetorque = np.zeros(6)
    for j,c in enumerate(data.contact):
        mj.mj_contactForce(model, data, j, forcetorque)

    q1q2 = np.pi/2#data.qpos[Index.q1.value] + data.qpos[Index.q2.value]
    toe_frame = np.array([[np.cos(q1q2),0,-np.sin(q1q2)],[0,1,0],[np.sin(q1q2),0,np.cos(q1q2)]])
    forcetorque_w= toe_frame@ forcetorque[0:3]
    
    #contact_x.append(forcetorque_w[0])
    #contact_y.append(forcetorque_w[1])
    contact_z.append(forcetorque_w[2])
    
    

    #Quasistatic
    f_xyz = np.linalg.pinv(foot_jacobian(model, data).T)@ data.actuator_force
    
    # applied_x.append(-f_xyz[0])
    # applied_y.append(-f_xyz[1])
    applied_z.append(-f_xyz[2])

    #plot when actually in contact
    if data.nefc:
        contact.append(1)
    else:
        contact.append(0)
    
    #COM plots

    if mode == 0:
        SLIP_flight(model, data, rdes = stance_h, theta_des = theta_des_SLIP)
        #if obs_z[-1] >= contact_force_tol and dR[0]<=0 :
        #if applied_z[-1] >= contact_force_tol and dR[0]<0:
        if con == 1:
            if delta_z >= tol and dR[0]<0:

                mode = 1
        
        elif con == 2:
            if obs_z[-1] >= contact_force_tol and dR[0]<=0 :
                mode = 1
        
        elif con == 3:
            if applied_z[-1] >= contact_force_tol and dR[0]<0:
                mode = 1

    elif mode == 1:
        #Toe_control(model, data, forward_kinematics(model, data)[2])
        SLIP_stance(model, data)
        if delta_z <= tol and dR[0]>0:
        #if obs_z[-1] <= k and dR[0]>0:
        #if applied_z[-1] >= contact_force_tol and dR[0]<0:
            theta_ff = 0.055
            theta_des_SLIP = touchdown_angle(xdot, stance_h, 0) + theta_ff
            mode = 0

    
    


def v_flight(model,data, zdes= -2 ,vdes=0):
    q1 = data.qpos[Index.q1.value]
    q2 = data.qpos[Index.q2.value]
    X = forward_kinematics(model,data) # [x,0,z]^T
    J = foot_jacobian(model, data)
    dX = J @ data.qvel[Index.q1.value:]
    #g =  np.array([0, 0, -9.81 * Params.M_total.value ])
    kp = 150
    kd = 20
    
    fz = np.array([10*(-X[0]) + 15*(- dX[0]), 0, kp*(zdes-X[2]) + kd*(vdes - dX[2])])
    tau = J.T@( fz)
    # _foot = data.xpos[3, 2]
    # print(_foot)
    data.ctrl[0] = tau[0]
    data.ctrl[1] = tau[1]

def vhad_stance(model, data):
    q1 = data.qpos[Index.q1.value]
    q2 = data.qpos[Index.q2.value]
    z = forward_kinematics(model, data)[2]
    x = forward_kinematics(model, data)[0]
    #zdot = data.qvel[Index.z.value]
    dX = (foot_jacobian(model, data)@data.qvel[Index.q1.value:])
    zdot = dX[2]
    #nominal height
    p = -1.5
    w = 23
    phi = np.arctan2( w*(p-z),-zdot)
    
    beta = 1.5
    ka = 48
    kd = 1
    w_i = 0 #kd*zdot*np.sin(phi)
    fx = 25*(-x)+ 15*(-dX[0])
    u = np.array([0, 0, w*w*(p-z)- beta*zdot -ka*np.cos(phi) ])  
    J = foot_jacobian(model, data)

    g =  np.array([0, 0, -9.81 * Params.M_total.value])
    tau = J.T@(u -g )
    
    data.ctrl[0] = tau[0]
    data.ctrl[1] = tau[1]

def vhad_control(model, data):
    
    tol = 8*1e-2
    stance_h = -1.5
    
    
    #v_flight(model, data, stance_h)
    global mode
    z = forward_kinematics(model, data)[2]
    delta_z = abs(z-stance_h)
    if mode == 0:
 
        v_flight(model, data, stance_h)

        if delta_z >= tol:
            
            mode = 1
    
    elif mode == 1:
        vhad_stance(model, data)
        if delta_z <= tol:
            mode = 0

    
    z_height.append(forward_kinematics(model, data)[2])

def inverse_kinematic(r_des = 1.7,th_des=THETA_FF):
    l1 = Params.l1.value
    l2 = Params.l2.value
    # print("QUICK TEST:",(l1*l1 + l2*l2 - r_des*r_des)/(2*l1*l2))
    q2 = np.pi - np.arccos((l1**2 + l2**2-r_des**2)/(2*l1*l2)) 

    q1 = th_des - 0.5*q2
    return q1, q2

def init_controller(model,data):
    stance = 1.7
    theta =-0.055
    q1,q2 = inverse_kinematic(stance, theta)
    # q2 = 1.7
    # q1 = -q2/2
    data.qpos[Index.q1.value] = q1
    data.qpos[Index.q2.value] = q2
    print("INIT:", q1, "   ", q2)
    
    


def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)

#Momentum Observer segment

#helper to get constraint momentas
def constraint(data):
    if data.nefc:
        J = data.efc_J.reshape((4,4))
        f = data.efc_force
        return J@f
    else: return np.zeros(4)





#get the full path

dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path('leg2.xml')  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options
if animate:
    # Init GLFW, create window, make OpenGL context current, request v-sync
    glfw.init()
    window = glfw.create_window(1200, 900, "Demo", None, None)
    glfw.make_context_current(window)
    glfw.swap_interval(1)


    # initialize visualization data structures
    mj.mjv_defaultCamera(cam)
    scene = mj.MjvScene(model, maxgeom=10000)
    context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

    # initialize visualization contact forces
    mj.mjv_defaultOption(opt)
    opt.flags[mj.mjtVisFlag.mjVIS_CONTACTPOINT] = True
    opt.flags[mj.mjtVisFlag.mjVIS_CONTACTFORCE] = True
    opt.flags[mj.mjtVisFlag.mjVIS_TRANSPARENT] = True
    # # tweak scales of contact visualization elements
    # model.vis.scale.contactwidth = 0.1
    # model.vis.scale.contactheight = 0.03
    # model.vis.scale.forcewidth = 0.05
    # model.vis.map.force = 0.3

    # install GLFW mouse and keyboard callbacks
    glfw.set_key_callback(window, keyboard)
    glfw.set_cursor_pos_callback(window, mouse_move)
    glfw.set_mouse_button_callback(window, mouse_button)
    glfw.set_scroll_callback(window, scroll)

    cam.azimuth = 89.608063
    cam.elevation = -11.588379
    cam.distance = 5.0
    cam.lookat = np.array([0.0, 0.0, 1.5])

def run(con = 1):

    global THETA_FF
    global theta_des_SLIP
    global mode
    global window, scene, context
    global model, data, opt, cam

    mj.mj_resetData(model, data)
    theta_des_SLIP = THETA_FF
    mode = 0
    init_controller(model,data)
    #v_flight(model, data, -1.5)
    #set the controller

    forcetorque = np.zeros(6)
    # contact_x = []
    # contact_y = []
    # contact_z = []
    M = np.zeros((model.nv,model.nv))

    # obs_x =[]
    # obs_y =[]
    # obs_z =[]
    # theta1 = []
    # theta2 = []

    tau_prev = P_prev =  np.zeros(model.nq)

    going_up_Q = False
    while not animate or not glfw.window_should_close(window):
        simstart = data.time
        # realtimestart = time.clock_gettime_ns()

        while (data.time - simstart < 1.0/60.0):
            #simulation step
            mj.mj_step(model, data)
            # Apply control
            
            #SLIP_flight(model, data)
            #SLIP_stance(model, data)
            SLIP(model, data, con)
        
        
        #z_height.append(radial(model, data)[1])

        forcetorque = np.zeros(6)
        
        if (data.time>=simend):
            break;
        

        if data.qvel[Index.z.value] > 0:
            going_up_Q =True
        if (going_up_Q and data.qvel[Index.z.value] < 0):
            break
        
        
        if animate:    
            
            viewport_width, viewport_height = glfw.get_framebuffer_size(
                window)
            viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

            # Show joint frames
            opt.flags[mj.mjtVisFlag.mjVIS_JOINT] = 1

            # Update scene and render
            #cam.lookat[0] = data.qpos[2] #camera follows the robot
            mj.mjv_updateScene(model, data, opt, None, cam,
                            mj.mjtCatBit.mjCAT_ALL.value, scene)
            mj.mjr_render(viewport, scene, context)

            cam.lookat = np.array([data.qpos[0], 0.0, 1.5])
            # swap OpenGL buffers (blocking call due to v-sync)
            glfw.swap_buffers(window)

            # process pending GUI events, call GLFW callbacks
            glfw.poll_events()

def t_delta(mode, contact):
    tdelta = 0
    for i in range(len(mode)):
        if mode[i] == 0 and contact[i] == 1:
            tdelta += 1
        if mode[i] == 1 and contact[i] == 1:
            break
    
    return tdelta

modes = []
contact =[]
run(1)
modes1 = np.array(modes)
contact1 = np.array(contact)
mj_contact1 = np.array(contact_z)
obs_contact1 = np.array(obs_z)


obs_x=[]
obs_z =[]
modes = []
contact =[]
contact_z = []
run(2)
modes2 = np.array(modes)
contact2 = np.array(contact)
z_contact = np.array(obs_z)
mj_contact2 = np.array(contact_z)






applied_z =[]
modes = []
contact =[]
contact_z = []
run(3)
modes3 = np.array(modes)
contact3 = np.array(contact)
mj_contact3 = np.array(contact_z)

glfw.terminate()

x = np.arange(0,len(modes1))


#try plotting delta t?
figures, axs = plt.subplots(3, sharex=True)


#detect = axs[0].get_ybound()[1]*np.array(contact1)

x = np.arange(0,len(modes1))
axs[0].plot(mj_contact1, color="royalblue", label="Mujoco")
axs[0].plot(obs_contact1, color="crimson", label="Observer")
axs[0].fill_between(x,axs[0].get_ybound()[1]*contact1,color="slategrey", alpha=0.8)
axs[0].fill_between(x,axs[0].get_ybound()[1]*modes1,color="orange", alpha=0.25)
axs[0].set_title(" Delta r guard")
axs[0].legend()



x = np.arange(0,len(modes2))
axs[1].plot(mj_contact2, color="royalblue", label="Mujoco")
axs[1].plot(z_contact, color="crimson", label="Observer")
axs[1].fill_between(x,axs[1].get_ybound()[1]*contact2,color="slategrey", alpha=0.8)
axs[1].fill_between(x,axs[1].get_ybound()[1]*modes2,color="orange", alpha=0.25)
axs[1].axhline(contact_force_tol, linestyle= "--")
axs[1].set_title(" Observer guard")
axs[1].legend()

x = np.arange(0,len(modes3))
axs[2].plot(mj_contact3, color="royalblue", label="Mujoco")
axs[2].plot(applied_z, color="crimson", label="Applied force")
axs[2].fill_between(x,axs[2].get_ybound()[1]*contact3,color="slategrey", alpha=0.8)
axs[2].fill_between(x,axs[2].get_ybound()[1]*modes3,color="orange", alpha=0.25)
axs[2].axhline(contact_force_tol, linestyle= "--")
axs[2].set_title("Applied force guard")
axs[2].legend()


plt.subplots_adjust(hspace=0.5)
plt.show()

