## 3DoF Trim program for Transport Aircraft Model
# It is only necessary to choose the speed and altitude,
# set the pitch rate state to zero, and adjust the throttle
# and elevator controls and the angle of attack state.
# Instead of the ROC constraint , we can specify the 
# flight-path angle and constrain the pich-attitude state
# to be equal to the angle of attack plus the flight-path angle.

## choose VT and alt
## Q = zero # pitch rate = 0
## adjust throttle, Elevator, alpha <== optimiing params
## specify gamma(flight path angle) and
## theta = alpha + gamma

# Lonigtudinal Trim
# X       = [VT, alpha, theta, Q, h]    # 5*1
# X_index = [ 0,     1,     2, 3, 4]
# u       = [thrtl, elev, Xcg, land]    # 4*1
# u_index = [    0,    1,   2,    3]
# inputs  =      [VT, h, gamma, Xcg, land]
# inputs_index = [ 0, 1,     2,   3,    4]

# using PyCall
# spo = pyimport("scipy.optimize")
using Optim
include("transp.jl")
include("cost.jl")

export trim


function trim(x_guess::Array, u_guess::Array, inputs::Array, trim_model::Int64)

    """
    Longitudinal Trim Analysis for Transport aircraft

    -- only necessary to choose VT, alt, gamma
    -- initial guesses for alpha, and controls : throttle setting, elevator angle
    -- controls to specify : CG location and landing configuration

    # References
    - [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
    and simulation: dynamics, controls design, and autonomous systems. John Wiley
    & Sons. (page 189 & 724)
    """
    
    x = copy(x_guess)
    u = copy(u_guess)

    # constants
    # constants = gamma, sin(gamma), PitchRate(Q), trim_model
    constants   = [0.0, 0.0, 0.0, 1]
    RTOD = 57.29578

    # trim_model
    # 1-> wing level (gamma =  0)
    # 2-> wing level (gamma != 0)
    constants[end] = trim_model

    # inputs = [VT, h, gamma, Xcg, land]
    # if trim_model == 1:   # wing level (gamma = 0)
    x[1] = inputs[1]    # VT
    x[5] = inputs[2]    # h
    u[3] = inputs[4]    # Xcg
    u[4] = inputs[5]    # landing config

    if trim_model == 2     # wing level (gamma != 0)
        gamma = inputs[3]
        constants[1] = gamma/RTOD
        constants[2] = sin(constants[1])
        x[3] = x[2] + gamma
    end

    # s- array bulidup      # optimizating paramters
    if (trim_model ==1) | (trim_model == 2)      # == 1   and  == 2
        s = zeros(Float64, (3,1))
        s[1] = u[1]     # throttle
        s[2] = u[2]     # elevator
        s[3] = x[2]     # alpha
    end

    ##------------------------------------------------------
    ## PYTHON SCIPY OPTIMIZE METHOD
    # maxiter = 1000
    # tol = 1e-7
    # minimize_tol = 1e-9
    # res = spo.minimize(cost_func, s, args=(x, u, constants), method="Nelder-Mead", tol=minimize_tol)#, options={'maxiter': maxiter})
    # # above function updates x and u to trim optimize values
    # println("cost = $(res["fun"])")
    # s = res["x"]    # optimized values

    # # assigning optimized value to state and control vectors
    # x[2] = s[3]
    # if trim_model == 2
    #     x[3] =  s[3] + (gamma/RTOD)
    # else
    #     x[3] = s[3]
    # end
    # u[1] = s[1]
    # u[2] = s[2] 
    ##------------------------------------------------------

    ##------------------------------------------------------
    ## JULIA OPTIM METHOD
    ## Wrapper for trim_cost_function with trimmer
    trimming_function(s) = cost_func(s, x, u, constants)
    ## minimization algorithm for Trim
    result = optimize(trimming_function, s,
                      Optim.Options(
                        g_tol=1e-25,
                        iterations=5000
                        );
    )
    # print(result)
    ##------------------------------------------------------

    return x, u
end


####=========================================================================
### TESTS :>

# x_ = [170, 0, 0, 0.0, 0.0]       # random guess
# u_ = [0.1, -10, 0.25, 0]        # random guess
# inputs = [250, 0, 0, 0.25, 0]  # not random
# trim_model = 1
# x, u = trim(x_, u_, inputs, trim_model)
# println("final_x = $x")
# println("final_u = $u")
## answers
# final_x = [2.50000000e+02 1.61919412e-01 1.61919412e-01 0.00000000e+00
# 0.00000000e+00]
# final_u = [ 0.18449576 -9.21841856  0.25        0.        ]


# # for [1] Example 3.7.1    page 202 [1]
# # For NUmerical Linearization
# x_ = [200, 0, 0, 0.0, 0.0]       # random guess
# u_ = [0.1, -10, 0.25, 0]         # random guess
# inputs = [200, 0, 15, 0.25, 0]  # not random
# trim_model = 2
# x, u = trim(x_, u_, inputs, trim_model)
# println("final_x = $x")
# println("final_u = $u")
# ## answers
# ## final_x = [200.  0.24276447   0.50456386   0.    0.  ]
# ## final_u = [  1.01370375 -12.32253564   0.25    0.  ]

