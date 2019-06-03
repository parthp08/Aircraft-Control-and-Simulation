from numpy import array

def vdpol(time, x, u):
	"""
	State Equation for Van der Pol Oscillator

	# References
    - [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
    and simulation: dynamics, controls design, and autonomous systems. John Wiley
    & Sons. (page 178)
	"""
	xd = array([x[1], -u[0]*(x[0]**2-1)*x[1]-x[0]])
	return xd
