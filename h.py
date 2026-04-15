import numpy as np
import math
from math import cos, sin
import matplotlib
matplotlib.use('TkAgg')
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation as fanim
from mpl_toolkits.mplot3d import Axes3D


pos = np.array([0.0,0.0,0.0])
vel = np.array([0.0,0.0,0.0])
time_step = 0.01

y= 0.0
vert_vel=0.0
mass=1.0
g=9.81
throttle = 9.81
con_torq = np.array([0.0,0.0,0.0])
torq_str = 0.5

roll = 0.0
pitch = 0.0
yaw = 0.0

om = np.array([0.0,0.0,0.0])
inertia =np.array([0.02,0.02,0.02])

e_c = 15
e_t = 0
damping = 6

#animation
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.set_xlim(-20, 20)
ax.set_ylim(-20, 20)
ax.set_zlim(0, 10)
ax.set_box_aspect([1.0, 1.0, 1.0])
line, = ax.plot([], [], [], 'o-', lw=3)


def rot_mat(roll, pitch, yaw):
    cr = cos(roll)
    sr = sin(roll)

    cp = cos(pitch)
    sp = sin(pitch)

    cy = cos(yaw)
    sy = sin(yaw)

    R = np.array([[cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
                 [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
                 [-sp, cp*sr, cp*cr]])

    return R

def control(st):
    global  throttle, con_torq
    print(st.key)
    if st.key == 'w':
        throttle +=2
    elif st.key == 'x':
        throttle -= 2
    elif st.key == 'a':
        con_torq[0] = torq_str
    elif st.key == 'd':
        con_torq[0] = -torq_str
    elif st.key == 'z':
        con_torq[2] = torq_str
    elif st.key == 'c':
        con_torq[2] = -torq_str
    print(con_torq)
    print(throttle)

fig.canvas.mpl_connect('key_press_event', control)

def key_r(st):
    global con_torq
    con_torq[:] = 0.0
fig.canvas.mpl_connect('key_release_event', key_r)

def update(action):
    global pos, vel, roll, pitch, yaw, om, throttle

    torques = con_torq

    ang_acc = torques / inertia
    om += ang_acc * time_step

    roll  += om[0] * time_step
    pitch += om[1] * time_step
    yaw   += om[2] * time_step

    R = rot_mat(roll, pitch, yaw)

    body_thrust = np.array([0, 0, throttle])
    world_thrust = R @ body_thrust

    gravity = np.array([0, 0, -mass * g])
    force = world_thrust + gravity

    acc = force / mass
    vel += acc * time_step
    pos += vel * time_step


    arm = 0.5
    arms_body = np.array([ [ arm, 0, 0],
                           [-arm, 0, 0],
                           [0,  arm, 0],
                           [0, -arm, 0]])

    arms_world = (R @ arms_body.T).T + pos

    range_size = 10
    ax.set_xlim(pos[0]-range_size,pos[0] + range_size)
    ax.set_ylim(pos[1]-range_size,pos[1] + range_size)
    ax.set_zlim(pos[2]-range_size,pos[2] + range_size)

    line.set_data(arms_world[:, 0], arms_world[:, 1])
    line.set_3d_properties(arms_world[:, 2])

    return line,


ani=fanim(fig, update, interval=10, cache_frame_data=False)
plt.title('test')
plt.show()
