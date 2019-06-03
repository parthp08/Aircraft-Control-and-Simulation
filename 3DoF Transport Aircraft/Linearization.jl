## Numerical Linearization of transport aircraft longitudinal model.
## using small disturbance theory
# X = [VT, alpha, theta, Q, h]     # (5,1)
# u = [throttle, elevator, Xcg, land]   # (4,1)
# inputs = [VT=250, h=0, gamma=15, Xcg=0.25, land=0]
# trim_model = 1    # steady level flight
## object is to find A and B jacbian matrices for 
## equation in state-space form using finite-difference method.
## x_dot = Ax + Bu
## and identify aircraft dynamic modes using model decomposition

# x_dot = A x  +  B u
# y = C x  +  D u
# There are 5 state variables, 4 inputs(1 active), and 1 outputs, making the matrix sizes:
# A: 5x5, B: 5x1, C: 1x5, D: 0x0
# y (outputs): [vt]
# active control: throttle

"""
# References
- [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
and simulation: dynamics, controls design, and autonomous systems. John Wiley
& Sons. (page 202 & 205-207)
"""

using LinearAlgebra
include("Trim.jl")
include("transp.jl")
include("print_matrix.jl")
include("numerical_jacobian.jl")

f = transp    # equation of motion function

## For numerical Linearization  # ex. 3.7.1 page 202 [1]
x_ = [200, 0, 0, 0.0, 0.0]       # random guess
u_ = [0.1, -10, 0.25, 0]         # random guess
inputs = [200, 0, 15, 0.25, 0]  # not random
trim_model = 2
x_trim, u_trim = trim(x_, u_, inputs, trim_model)
# println("final_x = $x")
# println("final_u = $u")
## answers
## final_x = [200.0, 0.242764, 0.504564, 0.0, 0.0]
## final_u = [1.0137, -12.3225, 0.25, 0.0]

## initial conditions
n = length(x_trim)   # number of states
m = length(u_trim)  # number of controls
x = x_trim  # initial states
u = u_trim  # initial control

mm = 1 # no of active control  # only throttle in this case
nn = n  #  no of state variables

tol = 1e-6

## Cacluating A_matrix
dx = 0.1    # purturbation in states
A = A_matrix_calc(f, x, u, dx, n, tol)
print("A = ")
print_matrix(A)
## A = [
## [-0.0283301, 16.3269, -30.8506, 0.0, 5.77977e-5]
## [-0.00135277, -0.514072, -0.0455322, 1.0, 4.15026e-6]
## [0.0, 0.0, 0.0, 1.0, 0.0]
## [-0.000118863, -0.495613, 0.00526285, -0.423813, 2.74418e-7]
## [0.258819, -193.794, 191.798, 0.0, 0.0]
## ]

## Cacluating B_matrix
du = 0.1    # purturbation in controls 
B = B_matrix_calc(f, x, u, du, mm, tol)
print("B = ")
print_matrix(B)
## B = [
## [10.1727]
## [-0.0125963]
## [0.0]
## [0.0270169]
## [0.0]
## ]

### Transport Aircraft Longitudinal Modes
# neglecting height state as it has very low effects on the other states
A2 = A[1:4, 1:4]
## Eigen Values and Eigen Vectors
## damping(E), natural frequency(wn), Time Period(T)
## eigen values(lambda), eigen vectors(lv)
F = LinearAlgebra.eigen(A2)
lv = F.vectors  # eigen vectors
lambda = F.values   # eigen values
print("lv = ")
print_matrix(lv)
# lv = [
# Complex{Float64}[0.996969+0.0im, 0.996969-0.0im, 0.999985+0.0im, 0.999985-0.0im]
# Complex{Float64}[0.0486596+0.0229975im, 0.0486596-0.0229975im, -0.00101146+3.98146e-5im, -0.00101146-3.98146e-5im]
# Complex{Float64}[0.0408929-0.0109942im, 0.0408929+0.0109942im, -0.00189944-0.00492801im, -0.00189944+0.00492801im]
# Complex{Float64}[-0.0124372+0.0347757im, -0.0124372-0.0347757im, 0.000726303-0.000357803im, 0.000726303+0.000357803im]
# ]
print("lambda = ")
print_matrix(lambda)
# lambda = [
# Complex{Float64}[-0.496863+0.716827im]
# Complex{Float64}[-0.496863-0.716827im]
# Complex{Float64}[0.0137555+0.152684im]
# Complex{Float64}[0.0137555-0.152684im]
# ]

### Short Period Mode (sp)
lambda_sp = lambda[1:2]
lv_sp = lv[:, 1:2]
wn_sp = sqrt(imag(lambda_sp[1])^2 + real(lambda_sp[1])^2)    # natural frequency
E_sp = -real(lambda_sp[1]) / wn_sp       # damping
T_sp = (2*pi/wn_sp) * (1/sqrt(1 - E_sp^2))   # time period
println("\nShort Period Mode :")
println("---------------------------")
println("Eigen_values = ")
print_matrix(lambda_sp)
println("Eigen_vector = ")
print_matrix(lv_sp)
println("E_sp  = $E_sp")
println("T_sp  = $T_sp sec")
# Eigen_values =
# [
# Complex{Float64}[-0.496863+0.716827im]
# Complex{Float64}[-0.496863-0.716827im]
# ]
# Eigen_vector =
# [
# Complex{Float64}[0.996969+0.0im, 0.996969-0.0im]                  # VT
# Complex{Float64}[0.0486596+0.0229975im, 0.0486596-0.0229975im]    # alpha
# Complex{Float64}[0.0408929-0.0109942im, 0.0408929+0.0109942im]    # theta
# Complex{Float64}[-0.0124372+0.0347757im, -0.0124372-0.0347757im]  # Q (pitch rate)
# ]
# E_sp  = 0.5696731373839168        # damping
# T_sp  = 8.765269826838386 sec     # time period

### Phugoid Mode (p)
lambda_p = lambda[3:4]
lv_p = lv[:, 3:4]
wn_p = sqrt(imag(lambda_p[1])^2 + real(lambda_p[1])^2)    # natural frequency
E_p = -real(lambda_p[1]) / wn_p       # damping
T_p = (2*pi/wn_p) * (1/sqrt(1 - E_p^2))   # time period
println("\nPhugoid Mode :")
println("---------------------------")
println("Eigen_values = ")
print_matrix(lambda_p)
println("Eigen_vector = ")
print_matrix(lv_p)
println("E_sp  = $E_p")
println("T_sp  = $T_p sec")
# Eigen_values =
# [
# Complex{Float64}[0.0137555+0.152684im]
# Complex{Float64}[0.0137555-0.152684im]
# ]
# Eigen_vector =
# [
# Complex{Float64}[0.999985+0.0im, 0.999985-0.0im]                        # VT
# Complex{Float64}[-0.00101146+3.98146e-5im, -0.00101146-3.98146e-5im]    # alpha
# Complex{Float64}[-0.00189944-0.00492801im, -0.00189944+0.00492801im]    # theta
# Complex{Float64}[0.000726303-0.000357803im, 0.000726303+0.000357803im]  # Q (pitch rate)
# ]
# E_p  = -0.08972776861554681      # damping
# T_p  = 41.15142902064004 sec     # time period

## phyiscal meaning of above values 
## discussion on F-16 example on page 206-207 [1]

