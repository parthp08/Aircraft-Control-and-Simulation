# Nonlinear Time History Simulation

"""
# References
- [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
and simulation: dynamics, controls design, and autonomous systems. John Wiley
& Sons. (page 176)
"""

import numpy as np
import matplotlib.pyplot as plt
from RK4 import RK4
from VDPOL import vdpol    # import State Equation file
name = vdpol

#### Initial condition from file
data = open("VDP.dat", "r")
tmp = []
data = data.readlines()
for i in data:
	tmp.append(float(i.strip()))
n = int(tmp[0])  # number of states
m = int(tmp[1])  # number of controls
x = tmp[2:n+2]  # initial state
u = tmp[n+2:n+m+2] # control input

runtime  = float(input("Enter Run-Time : "))
dt = float(input("Enter Integration Time-step : "))
N = int(runtime/dt)  # total number of steps for integration
k = 0
# NP = int(max(1, N/500))
NP = 1
time = 0.0
xd = name(time, x, u)  # set variables in state equations

# initialize output y-array
y = np.zeros((N,n))    # (N , n) zero matrix   ## n could be more

for i in range(0,N-1):
	time = i*dt
	if i % NP == 0:
		k += 1
		y[k,0] = x[0]    # record data as needed
		y[k,1] = x[1]
		# y[k,2] = 

	x = RK4(name, time, dt, x, u)

# phase portrait plot
t = NP*dt*np.arange(0,k+1)
plt.figure(figsize=(16,9))
plt.plot(y[:, 0], y[:, 1])   # For Van der Pol
plt.grid(True)
plt.axis([-3, 3, -4, 5])
plt.xlabel("X[0]")
plt.ylabel("X[1]")
plt.show()

