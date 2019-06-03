## Test for numerical_jacobian.jl

include("Trim.jl")
include("transp.jl")
include("print_matrix.jl")
include("numerical_jacobian.jl")

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
time = 0.0
dx = 0.1    # purtubation

f = transp

A = A_matrix_calc(f, x, u, dx, 4, tol)
print("A_matrix = ")
print_matrix(A)
## A_matrix = [
## [-0.0283301, 16.3269, -30.8506, 0.0]
## [-0.00135277, -0.514072, -0.0455322, 1.0]
## [0.0, 0.0, 0.0, 1.0]
## [-0.000118863, -0.495613, 0.00526285, -0.423813]
## ]

du = 0.1
mm = 2      # max active cotrol == 2 ==> throttle, elevator
B = B_matrix_calc(f, x, u, du, mm, tol)
print("B_matrix = ")
print_matrix(B)
## use [1]=throttle  and [2] = elevator
## B_matrix = [
## [10.1727, -0.0]
## [-0.0125963, -0.0]
## [0.0, -0.0]
## [0.0270169, -0.0070452]
## [0.0, -0.0]
## ]
