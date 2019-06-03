include("transp.jl")
include("constraint.jl")

export cost_func

function cost_func(s::Array, x::Array, u::Array, constants::Array)
    """
    cost function to compute cost of Array for longitudinal trim model

    # References
    - [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
    and simulation: dynamics, controls design, and autonomous systems. John Wiley
    & Sons. (page 190 & 724)
    """
    
    Sgamma = constants[2]
    gamma = asin(Sgamma)

    if length(s) == 3
        u[1] = s[1]
        u[2] = s[2]
        x[2] = s[3]
        x[3] = x[2] + gamma
    end

    # constraints
    x, u = constraints(x, u, constants)

    # x_dot
    time = 0.0
    xd = transp(time, x, u)

    # cost(f)
    f = (xd[1]^2) + (100*(xd[2]^2)) + (10*(xd[4]^2))

    # scale cost if its less than 1
    if f < 1.0
        f = f^0.5
    end

    return f
end

