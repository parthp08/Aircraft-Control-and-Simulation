export constraints

function constraints(x::Array, u::Array, constants::Array)
    """
    constraints for longitudinal flight for Steady level or climbing flight

    # References
    - [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
    and simulation: dynamics, controls design, and autonomous systems. John Wiley
    & Sons. (page 190 & 725)
    """

    gamma, Sgamma, PitchRate, trim_model = constants

    # Steady level Flight
    if trim_model == 1
        # applying constraint
        x[4] = PitchRate    # Q
    end

    # Steady Climbing Flight
    if trim_model == 2
        # applying constraint
        x[3] = x[2] + gamma    # theta = alpha + gamma
        x[4] = PitchRate    # Q
    end

    return x, u
end

