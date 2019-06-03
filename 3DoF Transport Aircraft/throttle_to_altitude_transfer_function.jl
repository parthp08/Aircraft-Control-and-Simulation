## Transport Aircraft Throttle to Height(altitude) Response Transfer Function
## [1], problem. 3.8.3, page 247
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
B_matrix = B_matrix[:, 1]   # 1 = Throttle Control
# print("B_matrix = ")
# print_matrix(B_matrix)
C_matrix = reshape([0, 0, 0, 0, 1], 1,5)
D_matrix = [0]

###========================================================================================
########### Throttle to Height T.F
sys_ss = ss(A_matrix,B_matrix,C_matrix,D_matrix)
sys_tf = tf(sys_ss)
# println(sys_tf)
## -8.881784197001252e-16*s^4 + 1.6281782145018708*s^3 + 3.3136670209972485*s^2 + 6.636093315601682*s + 2.064303735060852
## ----------------------------------------------------------------------------------------------------------------------------------
## 1.0*s^5 + 1.1830009568495463*s^4 + 1.150635416861414*s^3 + 0.029306920646996977*s^2 + 0.02733984954674466*s + 2.224164606253769e-6

### in zero, pole, gain form
sys_tf_zpk = zpk(sys_tf)
# println(sys_tf_zpk)
##                             (1.0*s - 1.8331657000308488e15)(1.0*s^2 + 1.6692910946782504*s + 3.4649710629457675)(1.0*s + 0.36590811845662313)
## -8.881784197001252e-16 * -------------------------------------------------------------------------------------------------------------------------------------------
##                          (1.0*s^2 + 1.1824773712561785*s + 1.125732079595433)(1.0*s^2 + 0.0004422260388315391*s + 0.02428417317059426)(1.0*s + 8.135955453551717e-5)

### zero, pole, gain data
z_tf, p_tf, k_tf = zpkdata(sys_tf)
## Zeros = [
## Array{Complex{Float64},1}[[1.83317e15+0.0im, -0.834646+1.66383im, -0.834646-1.66383im, -0.365908+0.0im]]
## ]
## Poles = [
## Array{Complex{Float64},1}[[-0.591239+0.881004im, -0.591239-0.881004im, -0.000221113+0.155834im, -0.000221113-0.155834im, -8.13596e-5+0.0im]]
## ]
## Gain = [
## [-8.88178e-16]
## ]

### Step response for openloop system. (full state)
### open-loop velocity response to throttle step of 0.1
### effect include both short-period and phugoid mode
t = range(0, stop=1000, length=2000)
y, t = step(0.1*sys_tf, t)
p1 = plot(t, y, legend=false, reuse=false)
title!("Step response for openloop Throttle to Altitude T.F")
xlabel!("Time(seconds)")
ylabel!("Height")
### Root Locus plot
p2 = rlocusplot(sys_tf)
title!("Root Locus plot for openloop Throttle to Altitude T.F(full state)")
xlims!(-4,1)
ylims!(-2,2)
### Bode Plot
gr()    # pyplot backend doesnt work for bode plot
p3 = bodeplot(sys_tf)
title!("Bode plot for openloop Throttle to Altitude T.F(full state)")
pyplot()

###========================================================================================
########### Short Period
z_sp = [-0.834646+1.66383im, -0.834646-1.66383im]
p_sp = [-0.591239+0.881004im, -0.591239-0.881004im]
k_sp = 1
sys_tf_sp = zpk(z_sp, p_sp, k_sp)
# println(sys_tf_sp)
##    1.0*s^2 + 1.669292*s + 3.4649642142159998
##   -----------------------------------------
##     1.0*s^2 + 1.182478*s + 1.125731603137

### Step response for openloop system. (Short Period)
### open-loop velocity response to throttle step of 0.1
### effect include both short-period and phugoid mode
# t = range(0, stop=1000, length=2000)
# y, t = step(0.1*sys_tf_sp, t)
# p1 = plot(t, y, legend=false, reuse=false)
# title!("Step response for openloop Throttle to Altitude T.F(Short Period)")
# xlabel!("Time(seconds)")
# ylabel!("Height")
### Root Locus plot
p2 = rlocusplot(sys_tf_sp)
title!("Root Locus plot for openloop Throttle to Altitude T.F(Short Period)")
xlims!(-4,1)
ylims!(-2,2)
# ### Bode Plot
gr()    # pyplot backend doesnt work for bode plot
p3 = bodeplot(sys_tf_sp)
title!("Bode plot for openloop Throttle to Altitude T.F(Short Period)")
pyplot()

###========================================================================================
########### Phugoid Mode
z_p = [1.83317e15+0.0im, -0.365908+0.0im]
p_p = [-0.000221113+0.155834im, -0.000221113-0.155834im, -8.13596e-5+0.0im]
k_p = -8.88178e-16
sys_tf_p = zpk(z_p, p_p, k_p)
# println(sys_tf_p)
##                   (1.0*s - 1.83317e15)(1.0*s + 0.365908)
## -8.88178e-16 -------------------------------------------------------------------
##              (1.0*s^2 + 0.000442226*s + 0.02428428444695877)(1.0*s + 8.13596e-5)

### Step response for openloop system. (Phugoid Mode)
### open-loop velocity response to throttle step of 0.1
### effect include both short-period and phugoid mode
t = range(0, stop=1000, length=2000)
y, t = step(0.1*sys_tf_p, t)
p1 = plot(t, y, legend=false, reuse=false)
title!("Step response for openloop Throttle to Altitude T.F(Phugoid Mode)")
xlabel!("Time(seconds)")
ylabel!("Height")
### Root Locus plot
p2 = rlocusplot(sys_tf_p)
title!("Root Locus plot for openloop Throttle to Altitude T.F(Phugoid Mode)")
xlims!(-4,1)
ylims!(-2,2)
### Bode Plot
gr()    # pyplot backend doesnt work for bode plot
p3 = bodeplot(sys_tf_p)
title!("Bode plot for openloop Throttle to Altitude T.F(Phugoid Mode)")
pyplot()


