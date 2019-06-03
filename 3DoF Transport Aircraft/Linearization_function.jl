include("numerical_jacobian.jl")

export numerical_Linearization

function numerical_Linearization(f::Function, x::Array, u::Array)#, dx::Float64=0.1, du::Float64=0.1, no_of_active_control::Int64=1, tol::Float64=1e-6)
    
    dx = 0.1
    du = 0.1
    no_of_active_control = 2
    tol = 1e-6

    ## initial conditions
    n = length(x)   # number of states
    # m = length(u)  # number of controls

    ## Cacluating A_matrix
    A = A_matrix_calc(f, x, u, dx, n, tol)

    ## Cacluating B_matrix
    mm = no_of_active_control
    B = B_matrix_calc(f, x, u, du, mm, tol)
    
    return A, B
end


### Test
# include("Trim.jl")
# include("transp.jl")
# include("print_matrix.jl")
# ## For numerical Linearization  # ex. 3.7.1 page 202 [1]
# x_ = [200, 0, 0, 0.0, 0.0]       # random guess
# u_ = [0.1, -10, 0.25, 0]         # random guess
# inputs = [200, 0, 15, 0.25, 0]  # not random
# trim_model = 2
# x_trim, u_trim = trim(x_, u_, inputs, trim_model)
# println("final_x = $x_trim")
# println("final_u = $u_trim")
# # answers
# # final_x = [200.0, 0.242764, 0.504564, 0.0, 0.0]
# # final_u = [1.0137, -12.3225, 0.25, 0.0]
# A, B = numerical_Linearization(transp, x_trim, u_trim)
# print("A = ")
# print_matrix(A)
# print("B = ")
# print_matrix(B)
