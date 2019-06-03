## Transport Aircraft Throttle to Speed Response Transfer Function
## [1], ex. 3.8.5, page 211
## model was trimmed for level flight condition at sea level in the
# clean configuration(landing = 0), with Xcg = 0.25, VT = 250 ft/sec, h = 0, gamma = 0
# Jacobian matrices A and B are obtained from Linearization_function.jl for 
# given trim and initial condition.

"""
# References
- [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
and simulation: dynamics, controls design, and autonomous systems. John Wiley
& Sons. (page 211)
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
### Linearization
A_matrix, B_matrix = numerical_Linearization(transp, x_trim, u_trim)
# print("A_matrix = ")
# print_matrix(A_matrix)
B_matrix = B_matrix[:, 1]   # 1 = Throttle Control
# print("B_matrix = ")
# print_matrix(B_matrix)
C_matrix = reshape([1, 0, 0, 0, 0], 1,5)
D_matrix = [0]

###========================================================================================
########### Throttle to Speed T.F (neglecting "h" state)
### neglecting h-state because of its very low coupling of with others states
A = A_matrix[1:4, 1:4]
B = B_matrix[1:4,:]
C = C_matrix[:, 1:4]
D = D_matrix

### Throttle to Speed Transfer Function (Vt / delta_t)
sys_ss = ss(A,B,C,D)
sys_tf = tf(sys_ss)
# println(sys_tf)
## 9.96788864373923*s^3 + 11.505085218388745*s^2 + 10.669195172177872*s - 0.6954889939514182
## -------------------------------------------------------------------------------------------------------
## 1.0*s^4 + 1.183000956849546*s^3 + 1.1497079193511823*s^2 + 0.028812967230526846*s + 0.02655519128360669

### zero, pole, gain data
z_tf, p_tf, k_tf = zpkdata(sys_tf)
## Zeros = [
## Array{Complex{Float64},1}[[-0.607591+0.880497im, -0.607591-0.880497im, 0.0609668+0.0im]]
## ]
## Poles = [
## Array{Complex{Float64},1}[[-0.591095+0.88077im, -0.591095-0.88077im, -0.000405056+0.153627im, -0.000405056-0.153627im]]
## ]
## Gain = [
## [9.96789]
## ]

### Step response for openloop system.
### open-loop velocity response to throttle step of 0.1
### effect include both short-period and phugoid mode
t = range(0, stop=1000, length=2000)
y, t = step(0.1*sys_tf, t)
p1 = plot(t, y, legend=false, reuse=false)
title!("Step response for openloop Throttle to Speed T.F")
xlabel!("Time(seconds)")
ylabel!("Velocity")
### Root Locus plot
p2 = rlocusplot(sys_tf)
title!("Root Locus plot for openloop Throttle to Speed T.F")
xlims!(-5,1)
ylims!(-1,1)
### Bode Plot
plotly()    # pyplot backend doesnt work for bode plot
p3 = bodeplot(sys_tf)
title!("Bode plot for openloop Throttle to Speed T.F")
pyplot()

###========================================================================================
########### Throttle to Speed T.F (only Short Period Mode)
## Short Period T.F
z_sp = z_tf[1][1:2]     # zeros of short period mode
p_sp = p_tf[1][1:2]     # poles of short period mode
k_sp = 1    # dc gain
sys_tf_sp = zpk(z_sp, p_sp, k_sp)   # Short Period T.F
# println(sys_tf_sp)
##         1.0*s^2 + 1.2151816329202116*s + 1.144442277227994
## 1.0 * --------------------------------------------------
##         1.0*s^2 + 1.182190844830184*s + 1.1251487178314734

### Step response for Short Period openloop system.
### open-loop velocity response to throttle step of 0.1
### effect include both short-period and phugoid mode
# t = range(0, stop=1000, length=2000)
# y, t = step(0.1*sys_tf_sp, t)
# p1 = plot(t, y, legend=false, reuse=false)
# title!("Step response for openloop Throttle to Speed T.F(Short Period only)")
# xlabel!("Time(seconds)")
# ylabel!("Velocity")
# ### Root Locus plot
p2 = rlocusplot(sys_tf_sp)
title!("Root Locus plot for openloop Throttle to Speed T.F(Short Period only)")
xlims!(-5,1)
ylims!(-1,1)
# ### Bode Plot
plotly()
p3 = bodeplot(sys_tf_sp)
title!("Bode plot for openloop Throttle to Speed T.F(Short Period only)")
pyplot()

###========================================================================================
########### Throttle to Speed T.F (only Phugoid Mode)
## Phugoid T.F
z_p = z_tf[][3:end]     # zeros of Phugoid mode
p_p = p_tf[1][3:4]     # poles of Phugoid mode
k_p = k_tf[1]    # dc gain
sys_tf_p = zpk(z_p, p_p, k_p)   # Short Period T.F
# println(sys_tf_p)
##                         1.0*s - 0.06096677061684698
## 9.96788864373923  * --------------------------------------------------------
##                         1.0*s^2 + 0.0008101120193619079*s + 0.023601494507132524

### Step response for Phugoid openloop system.
### open-loop velocity response to throttle step of 0.1
### effect include both short-period and phugoid mode
t = range(0, stop=1000, length=2000)
y, t = step(0.1*sys_tf_p, t)
p1 = plot(t, y, legend=false, reuse=false)
title!("Step response for openloop Throttle to Speed T.F(Phugoid Mode only)")
xlabel!("Time(seconds)")
ylabel!("Velocity")
### Root Locus plot
p2 = rlocusplot(sys_tf)
title!("Root Locus plot for openloop Throttle to Speed T.F(Phugoid Mode only)")
xlims!(-5,1)
ylims!(-1,1)
### Bode Plot
plotly()
p3 = bodeplot(sys_tf)
title!("Bode plot for openloop Throttle to Speed T.F(Phugoid Mode only)")
pyplot()

###========================================================================================
############### More accurate T.F. including altitide(h) state.
# if h is included in the A_matrix, it is found that because of the atmosphere model, there
# are small coupling terms from altitude to several other states.
# throttle to speed T.F.
sys_ss2 = ss(A_matrix,B_matrix,C_matrix,D_matrix)
sys_tf2 = tf(sys_ss2)
# println(sys_tf2)
## 9.967888643739217*s^4 + 11.505085218388738*s^3 + 10.678527509925551*s^2 - 0.6905450413048928*s + 0.007773401551629279
## ----------------------------------------------------------------------------------------------------------------------------------
## 1.0*s^5 + 1.1830009568495463*s^4 + 1.150635416861414*s^3 + 0.029306920646996977*s^2 + 0.02733984954674466*s + 2.224164606253769e-6

### in zero, pole, gain form
sys_tf2_zpk = zpk(sys_tf2)
# println(sys_tf2_zpk)
## (1.0*s^2 + 1.2154402554496104*s + 1.1450275546398359)(1.0*s - 0.0466147845859421)(1.0*s - 0.014610608560302598)
## 9.967888643739217-------------------------------------------------------------------------------------------------------------------------------------------
##                  (1.0*s^2 + 1.1824773712561785*s + 1.125732079595433)(1.0*s^2 + 0.0004422260388315391*s + 0.02428417317059426)(1.0*s + 8.135955453551717e-5)


### Step response for openloop system. (full state)
### open-loop velocity response to throttle step of 0.1
### effect include both short-period and phugoid mode
t = range(0, stop=1000, length=2000)
y, t = step(0.1*sys_tf2, t)
p1 = plot(t, y, legend=false, reuse=false)
title!("Step response for openloop Throttle to Speed T.F(full state)")
xlabel!("Time(seconds)")
ylabel!("Velocity")
### Root Locus plot
p2 = rlocusplot(sys_tf2)
title!("Root Locus plot for openloop Throttle to Speed T.F(full state)")
xlims!(-5,1)
ylims!(-1,1)
# ### Bode Plot
gr()    # pyplot backend doesnt work for bode plot
p3 = bodeplot(sys_tf2)
title!("Bode plot for openloop Throttle to Speed T.F(full state)")
pyplot()
