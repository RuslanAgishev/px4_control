#!/usr/bin/env 

import time
from scipy.integrate import odeint
import numpy as np
from math import *


def MassSpringDamper(state,t,F, mode='overdamped'):
	x = state[0]
	xd = state[1]
	if mode=='oscillations':
		m = 1.0; k = 2; b = 0 # oscillations
	elif mode=='underdapmped':
		m = 1.0; k = 2; b = 2*sqrt(m*k)-2 # underdamped
	elif mode=='overdamped':
		m = 1.0; k = 2; b = 2*sqrt(m*k) + 30 # overdamped
	else:
		m = 1.0; k = 2.0; b = 2*sqrt(m*k) # critically damped
	xdd = -(b/m)*xd - (k/m)*x + F/m
	return [xd, xdd]


def velocity_impedance(velocity, imp_pose_prev, imp_vel_prev, time_prev):
	F_coeff = 0.5 # 7, 12
	time_step = time.time() - time_prev
	time_prev = time.time()
	t = [0. , time_step]
	F = F_coeff * abs(velocity)

	state0_x = [imp_pose_prev[0], imp_vel_prev[0]]
	state_x = odeint(MassSpringDamper, state0_x, t, args=(F[0],))
	state_x = state_x[1]

	state0_y = [imp_pose_prev[1], imp_vel_prev[1]]
	state_y = odeint(MassSpringDamper, state0_y, t, args=(F[1],))
	state_y = state_y[1]

	state0_z = [imp_pose_prev[2], imp_vel_prev[2]]
	state_z = odeint(MassSpringDamper, state0_z, t, args=(F[2],))
	state_z = state_z[1]

	imp_pose = np.array( [state_x[0], state_y[0], state_z[0]] )
	imp_vel  = np.array( [state_x[1], state_y[1], state_z[1]] )

	return imp_pose, imp_vel, time_prev


def distance_impedance(dist_to_ground, imp_pose_prev, imp_vel_prev, time_prev, mode='critically_damped'):
	F_coeff = 0.01
	time_step = time.time() - time_prev
	time_prev = time.time()
	t = [0. , time_step]
	F =   - F_coeff / dist_to_ground

	state0_z = [imp_pose_prev, imp_vel_prev]
	state_z = odeint(MassSpringDamper, state0_z, t, args=(F, mode,))
	state_z = state_z[1]

	imp_pose = state_z[0]
	imp_vel  = state_z[1]

	return imp_pose, imp_vel, time_prev

