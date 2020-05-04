# -*- coding: utf-8 -*-
"""
Created on Mon Apr 27 10:09:15 2020

@author: scheu
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class CarState:
    x = 0
    y = 0
    theta = 0
    velocity = 0
    steer_angle = 0
    angular_velocity = 0
    slip_angle = 0
    st_dyn = True

class CarParam:
    wheelbase = 0.3302
    friction_coeff = 0.523
    h_cg = 0.074
    l_f = 0.17145
    l_r = 0.15875
    cs_f = 4.718
    cs_r =5.4562
    mass = 3.47
    I_z = 0.04712


def update(start: CarState, accel, steer_angle_vel, p: CarParam, dt):
    end = CarState()
    g = 9.81
    x_dot = start.velocity * np.cos(start.theta + start.slip_angle)
    y_dot = start.velocity * np.sin(start.theta + start.slip_angle)
    v_dot = accel
    steer_angle_dot = steer_angle_vel
    theta_dot = start.angular_velocity
    rear_val = g * p.l_r - accel * p.h_cg
    front_val = g * p.l_f + accel * p.h_cg
    if (start.velocity == 0):
        vel_ratio = 0
        first_term = 0
    else:
        vel_ratio = start.angular_velocity / start.velocity
        first_term = p.friction_coeff / (start.velocity * (p.l_r + p.l_f))
    
    theta_double_dot = (p.friction_coeff * p.mass / (p.I_z * p.wheelbase)) * (p.l_f * p.cs_f * start.steer_angle * (rear_val) + start.slip_angle * (p.l_r * p.cs_r * (front_val) - p.l_f * p.cs_f * (rear_val)) - vel_ratio * ((p.l_f**2) * p.cs_f * (rear_val) + (p.l_r**2) * p.cs_r * (front_val)))
    slip_angle_dot = (first_term) * (p.cs_f * start.steer_angle * (rear_val) - start.slip_angle * (p.cs_r * (front_val) + p.cs_f * (rear_val)) + vel_ratio * (p.cs_r * p.l_r * (front_val) - p.cs_f * p.l_f * (rear_val))) - start.angular_velocity
    
    end.x = start.x + x_dot * dt
    end.y = start.y + y_dot * dt
    end.theta = start.theta + theta_dot * dt
    end.velocity = start.velocity + v_dot * dt
    end.steer_angle = start.steer_angle + steer_angle_dot * dt
    end.angular_velocity = start.angular_velocity + theta_double_dot * dt
    end.slip_angle = start.slip_angle + slip_angle_dot * dt
    end.st_dyn = True
    return end

def get_slip(velocity, steering_angle, eps):
    initial = CarState()
    initial.velocity = velocity
    initial.steer_angle = steering_angle
    new = initial
    params = CarParam()
    diff = 10
    old_slip = 10
    while diff > eps:
        new = update(new, 0, 0, params, 0.01)
        diff = np.abs(new.slip_angle - old_slip)
        old_slip = new.slip_angle
    return old_slip

if __name__ == "__main__":
    desired_slip = 0.3
    velocities = np.arange(0.5, 4.6, 0.1)
    steering_angles = np.arange(0.01, 0.4189, 0.01)
    vel_plot = np.array([])
    steer_plot = np.array([])
    slip_angles = np.array([])
    vel_plot_3 = np.array([])
    steer_plot_3 = np.array([])
    slip_angles_3 = np.array([])
    for vel in velocities:
        for ang in steering_angles:
            slip = get_slip(vel, ang, 0.0000000001)
            vel_plot_3 = np.append(vel_plot_3, vel)
            steer_plot_3 = np.append(steer_plot_3, ang)
            slip_angles_3 = np.append(slip_angles_3, slip)
            if (abs(slip + desired_slip) < 0.01):
                vel_plot = np.append(vel_plot, vel)
                steer_plot = np.append(steer_plot, ang)
                slip_angles = np.append(slip_angles, slip)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(vel_plot, steer_plot, 'b.')
    ax.plot([3.6, 4.5], [0.40, 0.23], 'r')
    ax.set_xlabel('Velocity')
    ax.set_ylabel('Steering Angle')


    ## 3d
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(vel_plot_3, steer_plot_3, slip_angles_3)
    ax.set_xlabel('Velocity')
    ax.set_ylabel('Steering Angle')
    ax.set_zlabel('Slip Angle')
    plt.show()
    ax.set_zlim([-0.5,0])
    plt.show()

        
