import mujoco as mj
from mujoco import _functions
from mujoco.glfw import glfw

from mujoco import viewer
import numpy as np
from numpy.linalg import inv
import os
import matplotlib.pyplot as plt
from enum import Enum

import time
xml_path = 'twist.xml'
simend = 30

step_no = 0

class Params:
    def __init__(self, model, leg, Fixed = True):
        if( leg == "FL"):
            self.abd_id = model.joint('joint_8').id
            self.Upper_id = model.joint('joint_0').id
            self.Lower_id = model.joint('joint_1').id
            self.body_id = model.body('lower0').id
            self.jac_s = 7
            self.jac_e = 10
            self.ctrl_range = [model.actuator("FL Abductor").id , model.actuator("FL Upper").id, model.actuator("FL Lower").id]
        
        if( leg == "RL"):
            self.abd_id = model.joint('joint_9').id
            self.Upper_id = model.joint('joint_2').id
            self.Lower_id = model.joint('joint_3').id
            self.body_id = model.body('lower2').id
            self.jac_s = 10
            self.jac_e = 13
            self.ctrl_range = [model.actuator("RL Abductor").id , model.actuator("RL Upper").id, model.actuator("RL Lower").id]
        
        if( leg == "FR"):
            self.abd_id = model.joint('joint_10').id
            self.Upper_id = model.joint('joint_4').id
            self.Lower_id = model.joint('joint_5').id
            self.body_id = model.body('lower1').id
            self.jac_s = 13
            self.jac_e = 16
            self.ctrl_range = [model.actuator("FR Abductor").id , model.actuator("FR Upper").id, model.actuator("FR Lower").id]
        
        if( leg == "RR"):
            self.abd_id = model.joint('joint_11').id
            self.Upper_id = model.joint('joint_6').id
            self.Lower_id = model.joint('joint_7').id
            self.body_id = model.body('lower3').id
            self.jac_s = 16
            self.jac_e = 19
            self.ctrl_range = [model.actuator("RR Abductor").id , model.actuator("RR Upper").id, model.actuator("RR Lower").id]
        

        if Fixed == False:
            if( leg == "FL"):
                self.abd_id = 8#model.joint('joint_8').id
                self.Upper_id = 9#model.joint('joint_0').id
                self.Lower_id = 10 #model.joint('joint_1').id
                self.body_id = model.body('lower0').id
                self.jac_s = 7
                self.jac_e = 10
                self.ctrl_range = [model.actuator("FL Abductor").id , model.actuator("FL Upper").id, model.actuator("FL Lower").id]
        
            if( leg == "RL"):
                self.abd_id = 14#model.joint('joint_9').id
                self.Upper_id = 15#model.joint('joint_2').id
                self.Lower_id = 16#model.joint('joint_3').id
                self.body_id = model.body('lower2').id
                self.jac_s = 10
                self.jac_e = 13
                self.ctrl_range = [model.actuator("RL Abductor").id , model.actuator("RL Upper").id, model.actuator("RL Lower").id]
            
            if( leg == "FR"):
                self.abd_id = 11#model.joint('joint_10').id
                self.Upper_id = 12#model.joint('joint_4').id
                self.Lower_id = 13# model.joint('joint_5').id
                self.body_id = model.body('lower1').id
                self.jac_s = 13
                self.jac_e = 16
                self.ctrl_range = [model.actuator("FR Abductor").id , model.actuator("FR Upper").id, model.actuator("FR Lower").id]
        
            if( leg == "RR"):
                self.abd_id = 17 #model.joint('joint_11').id
                self.Upper_id = 18 #model.joint('joint_6').id
                self.Lower_id = 19 #model.joint('joint_7').id
                self.body_id = model.body('lower3').id
                self.jac_s = 16
                self.jac_e = 19
                self.ctrl_range = [model.actuator("RR Abductor").id , model.actuator("RR Upper").id, model.actuator("RR Lower").id]





# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

def FL_pose(model, data, leg_id, pose = np.array([0, -0.8, -1.6]) , posevel=np.array([0, 0, 0])):
    abd_des = pose[0]
    Upper_des = pose[1]
    lower_des = pose[2]

    abdvel_des = posevel[0]
    Uppervel_des = posevel[1]
    lowervel_des = posevel[2]

    leg = Params(model, leg_id, Fixed= True)
    
    # Kp = np.array([2.5, 3.5, 1]) #FOR FIXED
    # Kd = np.array([1, 2, 1])

    Kp = np.array([ 75, 45, 35]) 
    Kd = np.array([ 1.5, 1.5, 1.5])

    p_error = np.array([abd_des - data.qpos[leg.abd_id], 
                        Upper_des - data.qpos[leg.Upper_id],
                        lower_des - data.qpos[leg.Lower_id]])
    
    d_error = np.array([abdvel_des - data.qvel[leg.abd_id], 
                        Uppervel_des - data.qvel[leg.Upper_id],
                        lowervel_des - data.qvel[leg.Lower_id]])
    
    
    
    u = Kp*p_error + Kd*d_error
    jac = np.zeros((3,model.nv))
    mj.mj_jacSubtreeCom(model, data, jac, leg.body_id)
    tau = u
    
    data.ctrl[leg.ctrl_range[0]] = tau[0]
    data.ctrl[leg.ctrl_range[1]] = tau[1]
    data.ctrl[leg.ctrl_range[2]] = tau[2]

def ad_control(model, data):
    qhip = data.qpos[model.joint("joint_0").id]
    qknee = data.qpos[model.joint("joint_1").id]
    l1 = l2 = 0.206 
    r = l1*np.cos(qhip) + l2*np.cos(qknee )
    print(r)

    
def spine_control(model, data, qdes =  0, qdotdes = 0):
    q = data.qpos[model.joint("spine").id]
    qdot =  data.qvel[model.joint("spine").id]
    Kp = 250
    Kd = 15
    u = Kp*(qdes-q) + Kd*(qdotdes-qdot)
    data.ctrl[model.actuator("Spine Torque").id] = u
    
    
def controller(model, data):
    """
    This function implements a controller that
    mimics the forces of a fixed joint before release
    """
    FL_pose(model, data, "FL")
    
    FL_pose(model, data, "RL")
    FL_pose(model, data, "FR", np.array([0, -1.57, -3.14]))
    FL_pose(model, data, "RR", np.array([0, -1.57, -3.14]))
    
    spine_control(model, data)
    
    ad_control(model, data)
   

def init_controller(model,data):
    # pservo-hip
    pass


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




#get the full path

dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
# xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
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
# opt.flags[mj.mjtVisFlag.mjVIS_TRANSPARENT] = True
# # tweak scales of contact visualization elements
# model.vis.scale.contactwidth = 0.1
# model.vis.scale.contactheight = 0.03
# model.vis.scale.forcewidth = 0.05
# model.vis.map.force = 0.3

# install GLFW mouse and keyboard callbacks
# glfw.set_key_callback(window, keyboard)
# glfw.set_cursor_pos_callback(window, mouse_move)
# glfw.set_mouse_button_callback(window, mouse_button)
# glfw.set_scroll_callback(window, scroll)

cam.azimuth = 15.608063
cam.elevation = -11.588379
cam.distance = 5.0
cam.lookat = np.array([0.0, 0.0, 1.5])

init_controller(model,data)

#set the controller
mj.set_mjcb_control(controller)
forcetorque = np.zeros(6)
contact_x = []
contact_y = []
contact_z = []


    
obs_x =[]
obs_y =[]
obs_z =[]
last_time = 0

with viewer.launch_passive(model, data) as viewer:
  # Close the viewer automatically after 30 wall-seconds.
  start = time.time()
  while viewer.is_running() : #and time.time() - start < simend
    step_start = time.time()

    # mj_step can be replaced with code that also evaluates
    # a policy and applies a control signal before stepping the physics.
    mj.mj_step(model, data)

    # Example modification of a viewer option: toggle contact points every two seconds.
    with viewer.lock():
    #   viewer.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)
      viewer.opt.flags[mj.mjtVisFlag.mjVIS_CONTACTFORCE] = 1

    # Pick up changes to the physics state, apply perturbations, update options from GUI.
    viewer.sync()

    # Rudimentary time keeping, will drift relative to wall clock.
    time_until_next_step = model.opt.timestep - (time.time() - step_start)
    if time_until_next_step > 0:
      time.sleep(time_until_next_step)



