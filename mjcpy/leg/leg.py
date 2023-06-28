import mujoco as mj
from mujoco import _functions
from mujoco.glfw import glfw
import numpy as np
from numpy.linalg import inv
import os
import matplotlib.pyplot as plt
from enum import Enum

xml_path = 'leg.xml'
simend = 15

step_no = 0

class Index(Enum):

    #x = 0
    z = 0
    q1 = 1
    q2 = 2

FSM_AIR1 = 0
FSM_STANCE1 = 1
FSM_STANCE2 = 2
FSM_AIR2 = 3

fsm = FSM_AIR1

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0
error_i = np.zeros(2)

def forward_kinematics(model,data):
    q1 = data.qpos[1]
    q2 = data.qpos[2]
    g= 1
    x = np.sin(q1) + np.sin(q1+q2)
    y = 0
    z = -np.cos(q1) - np.cos(q1+q2)
    return np.array([x,y,z])

def foot_jacobian(model, data):
    J = np.zeros((3,2))
    q1 = data.qpos[Index.q1.value]
    q2 = data.qpos[Index.q2.value]
    J[0,0] = (np.cos(q1)+ np.cos(q1+q2))
    J[0,1] = np.cos(q1+q2)
    
    J[2,0] = (np.sin(q1)+np.sin(q1+q2))
    J[2,1]=  (np.sin(q1+q2))
    return J

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
    ff = -J.T @ np.array([0,0,-1.3 * 9.81])
    data.ctrl[0] = tau[0] - ff[1]
    data.ctrl[1] =  tau[1]- ff[2]

def v_flight(model,data, zdes= -2 ,vdes=0):
    q1 = data.qpos[Index.q1.value]
    q2 = data.qpos[Index.q2.value]
    X = forward_kinematics(model,data) # [x,0,z]^T
    J = foot_jacobian(model, data)
    dX = J @ data.qvel[1:3]
    g =  np.array([0, 0, -9.81*1.1])
    kp = 250
    kd = 50
    
    fz = np.array([0, 0, kp*(zdes-X[2]) + kd*(vdes - dX[2])])
    tau = J.T@( fz+ g)
    _foot = data.xpos[3, 2]
    print(_foot)
    data.ctrl[0] = tau[0]
    data.ctrl[1] = tau[1]

def vhad_stance(model, data):
    q1 = data.qpos[Index.q1.value]
    q2 = data.qpos[Index.q2.value]
    z = -np.cos(q1)- np.cos(q1+q2)
    x = np.sin(q1)+ np.sin(q1+q2)
    #zdot = data.qvel[Index.z.value]
    dX = (foot_jacobian(model, data)@data.qvel[Index.q1.value:])
    zdot = data.qvel[0] - dX[2]
    #nominal height
    p = -1.5
    w = 1
    phi = np.arctan2( w*(p-z),-zdot)
    
    beta = 1
    ka = 10
    kd = 1
    w_i = 0 #kd*zdot*np.sin(phi)
    fx = 1*(-x)+ 10*(-dX[0])
    u = np.array([fx, 0, 100*(p-z)- beta*zdot -ka*np.cos(phi) + w_i])  # (w*w)*(p-z) - beta*zdot-ka*np.cos(phi) + w
    J = foot_jacobian(model, data)
    tau = J.T@(u)
    data.ctrl[0] = tau[0]
    data.ctrl[1] = tau[1]
     
def vhad_control(model, data):
    
    stance_height = -1.3
    z = forward_kinematics(model, data)[2]
    if(z > stance_height): v_flight(model, data, -1.5)
    else: 
        print("kekw")
        data.ctrl[0] = 0
        data.ctrl[1] = 0
    
    
def init_controller(model,data):
    data.ctrl[0] = 0
    data.ctrl[1] = 0
    data.qpos[1] = -0.4
    data.qpos[2] = 0.8
    
    

def set_position_servo(actuator_no, kp):
    model.actuator_gainprm[actuator_no, 0] = kp
    model.actuator_biasprm[actuator_no, 1] = -kp

def set_velocity_servo(actuator_no, kv):
    model.actuator_gainprm[actuator_no, 0] = kv
    model.actuator_biasprm[actuator_no, 2] = -kv

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
def g_force(data):
    mhip = 1
    mleg = 1
    mtoe= 0.1
    g = 9.81
    d1 = 0.5
    d2 = 0.125
    q = data.qpos[2]
    l = data.qpos[3]

    return np.array([0,g*(mtoe+mhip+ mleg), g*(d2*mtoe +d1*(mleg+mtoe) +mtoe*l)
                     *np.sin(q), -g*mtoe*np.cos(q)])
    
tau_prev = P_prev =  np.zeros(4)

def get_C(model, data):
    mhip = 1
    mleg = 1
    mtoe= 0.1
    g = 9.81
    d1 = 0.5
    d2 = 0.125
    q = data.qpos[2]
    l = data.qpos[3]
    qdot = data.qvel[2]
    ldot = data.qvel[3]
    C = np.zeros((model.nv, model.nv))

    C[0,2] = mtoe*np.cos(q)*ldot - (d2*mtoe+d1*(mleg+mtoe)+mtoe*l)*np.sin(q)*qdot
    C[0,3] = mtoe*np.cos(q)*qdot
    C[1,3] = mtoe*np.sin(q)*qdot
    C[1,2] = mtoe*np.sin(q)*ldot + np.cos(q)*(d2*mtoe + d1*(mleg + mtoe) + mtoe*l) *qdot
    C[2,2] = 0.5*(mtoe*l + mtoe*(2*(d1+d2)+l))*ldot
    C[2,3] = 0.5*(mtoe*l + mtoe*(2*(d1+d2)+l))*qdot
    C[3,2] = 0.5*(mtoe*l + mtoe*(2*(d1+d2)+l))*qdot
    return C

def jacobian_toe(model,data):
    
    d1 = 0.5
    d2 = 0.125
    q = data.qpos[2]
    l = data.qpos[3]
    qdot = data.qvel[2]
    ldot = data.qvel[3]

    J = np.zeros((3,model.nv))
    J[0,0] = J[-1,1]=1
    J[0,2] = np.cos(q)*(d1+d2 +l)
    J[0,3] = np.sin(q)

    J[-1,2] = np.sin(q)*(d1+d2 +l)
    J[-1,3] = -np.cos(q)

    return J
def momentum_observer(model,data):

    global P_prev, tau_prev

    #Constructing M from the sparse matrix qM
    M = np.zeros((model.nv, model.nv))
    _functions.mj_fullM(model, M, data.qM)

    #GETTING JACOBIAN
    jac_foot = np.zeros((3, model.nv))
    mj.mj_jacSubtreeCom(model, data, jac_foot, model.body('foot').id)

    C = get_C(model, data) 
    # C = get_C(model, data) # Cq + G term
    v = data.qvel
    tau_ext = data.actuator_force
    P = M@v
    
    # observer 
    t_delta = 0.001
    freq = 100 # cut-off frequency 
    gamma = np.exp(-freq*t_delta)
    beta = (1-gamma)/(gamma*t_delta)
    alpha_k = beta*P + tau_ext + C.T@data.qvel - g_force(data)
    tau_d = beta*(P -gamma*P_prev) + gamma*(tau_prev)+(gamma-1)*alpha_k  

    tau_prev = tau_d
    P_prev = P

    

    #contact force from joint torque
    # hip_rot = np.array([[np.cos(q),0,np.sin(q)],[0,1,0],[-np.sin(q),0,np.cos(q)]])
    J = jacobian_toe(model, data)
    contact = np.linalg.pinv(J.T) @ tau_d
    return contact


#get the full path

dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path('leg.xml')  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options



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

init_controller(model,data)

#set the controller

forcetorque = np.zeros(6)
contact_x = []
contact_y = []
contact_z = []
M = np.zeros((model.nv,model.nv))
jac_com = np.zeros((3, model.nv))

# theta1 = []
# theta2 = []
z_height = []
zdes = -1.5
while not glfw.window_should_close(window):
    simstart = data.time

    while (data.time - simstart < 1.0/60.0):
        #simulation step
        mj.mj_step(model, data)
        # Apply control
        #pid_controller(model, data)
        #v_flight(model, data,zdes)
        #vhad_stance(model, data)
        vhad_control(model, data)

    # theta1.append(data.qpos[1])
    # theta2.append(data.qpos[2])
    z_height.append(forward_kinematics(model, data)[2])
    if (data.time>=simend):
        break;

    # get framebuffer viewport
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

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

        

glfw.terminate()

# fig,axs = plt.subplots(2)

# axs[0].plot(theta1)
# axs[0].axhline(-0.4,color = 'y', linestyle = '--')
# axs[0].set_title("Theta1")

# axs[1].plot(theta2)
# axs[1].axhline(0.8,color = 'y', linestyle = '--')
# axs[1].set_title("Theta2")
# plt.subplots_adjust(hspace=0.5)
plt.plot(z_height)
plt.axhline(zdes,color = 'y', linestyle = '--')
plt.show()
