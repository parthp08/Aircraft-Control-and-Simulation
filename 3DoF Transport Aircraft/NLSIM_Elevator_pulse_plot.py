from plot_3d import plot3d_anim
import numpy as np

## extracting time and state array from the output
A = np.genfromtxt('NLSIM_Elevator_pulse_output.csv', delimiter=',')

y_array = A[:, 1:]  # state array
t_array = A[:, 0]   # time array

### Longitudinal 3D Animation  (elevator pulse)
times = t_array   # time array # times at which states are given
y_zero = np.zeros(len(y_array), dtype=float)        # zeros array for some states
states = np.zeros((12, len(y_array)), dtype=float)
## states : [Vt, alpha, beta, phi, theta, psi, p, q, r, dx, dy, h]
states[0] = y_array[:,0]
states[1] = y_array[:,1]
states[2] = y_zero
states[3] = y_zero
states[4] = y_array[:,2]
states[5] = y_zero
states[6] = y_zero
states[7] = y_array[:,3]
states[8] = y_zero
states[9] = y_array[:,5]
states[10] = y_zero
states[11] = y_array[:,4]

states = states.T
# print(states)

plot3d_anim(times, states, filename="longitudinal_dynamics.gif")
# this usually takes around 2-3 minutes to complete
# can take upto to 6-8 minutes if the skip frame is set to lower limit

