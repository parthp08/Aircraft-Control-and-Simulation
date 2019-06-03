export adc

function adc(vt::Float64, alt::Number)
    """
    compute atmospheric propeties from given altitude

    # References
    - [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
    and simulation: dynamics, controls design, and autonomous systems. John Wiley
    & Sons. (page 715)
    """

    # vt = freestream air speed

    rho_0 = 2.377e-3
    Tfac = 1 - 0.703e-5 * alt    # temprature

    if alt >= 35000
        T = 390
    else
        T = 519 * Tfac # 3 rankine per atmosphere (3 rankine per 1000 ft)
    end

    # rho = freestream mass density
    rho = rho_0 * Tfac^4.14

    a = sqrt(1.4 * 1716.3 * T)  # a = sqrt(gamma * R* T) # speed of sound
    # amach = mach number
    amach = vt / a

    # qbar = dynamic pressure
    qbar = 0.5 * rho * vt * vt

    return amach, qbar
end

