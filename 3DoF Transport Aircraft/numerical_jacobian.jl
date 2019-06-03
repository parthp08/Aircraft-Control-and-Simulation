export A_matrix_calc, B_matrix_calc

"""
# References
- [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
and simulation: dynamics, controls design, and autonomous systems. John Wiley
& Sons. (page 201)
"""

##-------------------------------------------------------------------------
function A_matrix_calc(f::Function, x::Array, u::Array, dx_::Float64, nn::Int, tol::Float64)
    """
    #### A_matrix
    # dx = puturbation
    # nn = number of states to consider
    # f = function name # xdot file # equation of motion file
    # x = state array 
    """

    # number of states
    # n = length(x)
    n = nn

    dx = dx_.*x      # purturbation of 0.1 in all state variables
    for i = 1:n   # set perturbations
        if dx[i] == 0.0
            dx[i] = dx_
        end
    end

    time = 0.0

    last = zeros(Float64, n,1)  # last column of matrix
    A = zeros(Float64, n,n)     # initialize A-matrix

    for j = 1:n
        xt = x
        for i = 1:10
            xt[j] = x[j] + dx[j]    # adding dx to x
            xd1 = f(time, xt, u)[1:n]  # by using [1:n] supressing 6th number x-position output
            xt[j] = x[j] - dx[j]
            xd2 = f(time, xt, u)[1:n]  # by using [1:n] supressing 6th number x-position output
            A[:, j] = (transpose(xd1 - xd2) ./ (2*dx[j]))
            if false in (max.(abs.(A[:,j] - last) ./ abs.(A[:,j] .+ 1e-12)) .< tol)
                break
            end
            dx[j] = 0.5*dx[j]  # reducing dx further to improve accuracy of the step
            last = A[:,j]
            ## column = j
            iteration = i
            if iteration == 10
                print("not converged on A, column {j}")
            end
        end
    end

    A_ = A.*2
    return A_
end

##-------------------------------------------------------------------------
function B_matrix_calc(f::Function, x::Array, u::Array, du_::Float64, mm::Int, tol::Float64)
    """
    #### B_matrix
    # du = puturbation
    # mm = number of active control
    # f = function name # xdot file # equation of motion file
    # x = state array 
    """

    # number of states
    n = length(x)
    m = length(u)

    du = du_.*u      # purturbation of 0.1 in all state variables
    for i = 1:m   # set perturbations
        if du[i] == 0.0
            du[i] = du_
        end
    end

    time = 0.0

    last = zeros(Float64, n,1)  # last column of matrix
    B = zeros(Float64, n,mm)     # initialize A-matrix

    for j = 1:mm
        ut = u
        for i = 1:10
            ut[j] = u[j] + du[j]    # adding du to u
            ut = ut
            xd1 = f(time, x, ut)[1:n]  # by using [1:n] supressing 6th number x-position output
            ut[j] = u[j] - du[j]
            ut = ut
            xd2 = f(time, x, ut)[1:n]  # by using [1:n] supressing 6th number x-position output
            B[:, j] = (transpose(xd1 - xd2) ./ (2*du[j]))
            if false in (max.(abs.(B[:,j] - last) ./ abs.(B[:,j] .+ 1e-12)) .< tol)
                break
            end
            dx[j] = 0.5*du[j]  # reducing du further to improve accuracy of the step
            last = B[:,j]
            ## column = j
            iteration = i
            if iteration == 10
                print("not converged on B, column {j}")
            end
        end
    end

    B_ = B.*2
    return B_
end

##-------------------------------------------------------------------------

