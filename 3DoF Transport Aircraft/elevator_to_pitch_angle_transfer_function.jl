## Transport Aircraft Elevator to Pitch Angle Response Transfer Function
## [1], problem. 3.8.4, page 247
## model was trimmed for level flight condition at sea level in the
# clean configuration(landing = 0), with Xcg = 0.25, VT = 250 ft/sec, h = 0, gamma = 0
# Jacobian matrices A and B are obtained from Linearization_function.jl for 
# given trim and initial condition.

"""
# References
- [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
and simulation: dynamics, controls design, and autonomous systems. John Wiley
& Sons. (page 247)
"""

using ControlSystems
using Plots
pyplot()    # python backend for plots
include("transp.jl")
include("Trim.jl")
include("Linearization_function.jl")
include("print_matrix.jl")

### Trim initial condition
x_ = [200, 0, 0, 0.0, 0.0]       # random guess
u_ = [0.1, -10, 0.25, 0]         # random guess
inputs = [250, 0, 0, 0.25, 0]  # not random # given initial condition
trim_model = 1  # 1 = steady level flight
x_trim, u_trim = trim(x_, u_, inputs, trim_model)

### Linearization
A_matrix, B_matrix = numerical_Linearization(transp, x_trim, u_trim)
# print("A_matrix = ")
# print_matrix(A_matrix)
B_matrix = B_matrix[:, 2]   # 2 = Elevator Control
# print("B_matrix = ")
# print_matrix(B_matrix)
C_matrix = reshape([0, 0, 1, 0, 0], 1,5)
D_matrix = [0]

###========================================================================================
########### Throttle to Height T.F
sys_ss = ss(A_matrix,B_matrix,C_matrix,D_matrix)
sys_tf = tf(sys_ss)
# println(sys_tf)
## 1.7763568394002505e-15*s^4 - 0.011008118902434472*s^3 - 0.007190891134506411*s^2 - 0.00032401934173099944*s - 2.85883945770599e-8
## ----------------------------------------------------------------------------------------------------------------------------------
## 1.0*s^5 + 1.1830009568495463*s^4 + 1.150635416861414*s^3 + 0.029306920646996977*s^2 + 0.02733984954674466*s + 2.224164606253769e-6

### in zero, pole, gain form
sys_tf_zpk = zpk(sys_tf)
# println(sys_tf_zpk)
##                          (1.0*s - 6.1970200233824e12)(1.0*s + 0.6045542739086285)(1.0*s + 0.04859255682382135)(1.0*s + 8.840393744534328e-5)
## 1.7763568394002505e-15-------------------------------------------------------------------------------------------------------------------------------------------
##                       (1.0*s^2 + 1.1824773712561785*s + 1.125732079595433)(1.0*s^2 + 0.0004422260388315391*s + 0.02428417317059426)(1.0*s + 8.135955453551717e-5)

### zero, pole, gain data
z_tf, p_tf, k_tf = zpkdata(sys_tf)
# print("Zeros = ")
# print_matrix(z_tf)
# print("Poles = ")
# print_matrix(p_tf)
# print("Gain = ")
# print_matrix(k_tf)
## Zeros = [
## Array{Complex{Float64},1}[[6.19702e12+0.0im, -0.604554+0.0im, -0.0485926+0.0im, -8.84039e-5+0.0im]]
## ]
## Poles = [
## Array{Complex{Float64},1}[[-0.591239+0.881004im, -0.591239-0.881004im, -0.000221113+0.155834im, -0.000221113-0.155834im, -8.13596e-5+0.0im]]
## ]
## Gain = [
## [1.77636e-15]
## ]

# ### Step response for openloop system. (full state)
# ### open-loop velocity response to Elevator step of 0.1
# ### effect include both short-period and phugoid mode
t = range(0, stop=1000, length=2000)
y, t = step(0.1*sys_tf, t)
p1 = plot(t, y, legend=false, reuse=false)
title!("Step response for openloop Elevator to Pitch Angle T.F")
xlabel!("Time(seconds)")
ylabel!("Pitch Angle")
ylims!(-0.1,0.1)
# ### Root Locus plot
p2 = rlocusplot(sys_tf)
title!("Root Locus plot for openloop Elevator to Pitch Angle T.F(full state)")
xlims!(-4,1)
ylims!(-2,2)
# # ### Bode Plot
gr()    # pyplot backend doesnt work for bode plot
p3 = bodeplot(sys_tf)
title!("Bode plot for openloop Elevator to Pitch Angle T.F(full state)")
pyplot()

###========================================================================================
########### Short Period
z_sp = [6.19702e12+0.0im]
p_sp = [-0.591239+0.881004im, -0.591239-0.881004im]
k_sp = 1.77636e-15
sys_tf_sp = zpk(z_sp, p_sp, k_sp)
# println(sys_tf_sp)
##                1.0*s - 6.19702e12  
##   1.77636e-15 -------------------------------------
##                1.0*s^2 + 1.182478*s + 1.125731603137

### Step response for openloop system. (Short Period)
### open-loop velocity response to Elevator step of 0.1
### effect include both short-period and phugoid mode
t = range(0, stop=1000, length=2000)
y, t = step(0.1*sys_tf_sp, t)
p1 = plot(t, y, legend=false, reuse=false)
title!("Step response for openloop Elevator to Pitch Angle T.F(Short Period)")
xlabel!("Time(seconds)")
ylabel!("Pitch Angle")
### Root Locus plot
p2 = rlocusplot(sys_tf_sp)
title!("Root Locus plot for openloop Elevator to Pitch Angle T.F(Short Period)")
xlims!(-4,1)
ylims!(-2,2)
### Bode Plot
gr()    # pyplot backend doesnt work for bode plot
p3 = bodeplot(sys_tf_sp)
title!("Bode plot for openloop Elevator to Pitch Angle T.F(Short Period)")
pyplot()

###========================================================================================
########## phugoid
z_p = [-0.604554+0.0im, -0.0485926+0.0im, -8.84039e-5+0.0im]
p_p = [-0.000221113+0.155834im, -0.000221113-0.155834im, -8.13596e-5+0.0im]
k_p = 1
sys_tf_p = zpk(z_p, p_p, k_p)
gr()    # pyplot backend doesnt work for bode plot
p5 = bodeplot(sys_tf_p)
title!("Bode plot for openloop Elevator to Pitch Angle T.F(Phugoid)")

###========================================================================================
## Comparision of Full T.F and Short Period Approx. o bode plot
p4 = bodeplot(sys_tf)
bodeplot!(sys_tf_sp)
bodeplot!(sys_tf_p)
title!("Comparision of Bode Plot of Full State and Short Period and Phugoid Approx")
pyplot()

