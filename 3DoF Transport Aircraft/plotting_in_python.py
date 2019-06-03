from numpy import array, linspace
from control.matlab import *
import matplotlib.pyplot as plt

## this file is plotting the systems which are giving errors in Julia
## particularly D!=0 Error in Simulation of Step

num = array([1, 1.669292, 3.4649642142159998])
den = array([1, 1.182478, 1.125731603137])
G = tf(num,den)
print(G)

t = linspace(0,1000,num=2000)
y, t = step(G*0.1, t)
plt.plot(t, y)
plt.show()

rlocus(G)
plt.show()
