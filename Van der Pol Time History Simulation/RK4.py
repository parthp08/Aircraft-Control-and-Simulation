def RK4(f, time, dt, xx, u):
	"""
	fourth-order Runge-Kutta Algorithm

	# References
    - [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
    and simulation: dynamics, controls design, and autonomous systems. John Wiley
    & Sons. (page 177)
	"""
	# k1
	xd = f(time, xx, u)
	xa = xd*dt
	# k2
	x = xx + 0.5*xa
	t = time + 0.5*dt
	xd = f(t, x, u)
	q = xd*dt
	# k3
	x = xx + 0.5*q
	xa = xa + 2.0*q
	xd = f(t, x, u)
	q = xd*dt
	# k4
	x = xx + q
	xa = xa + 2.0*q
	time = time + dt
	xd = f(time, x, u)
	
	# x_new	
	xnew = xx + (xa + xd*dt)/6.0
	return xnew

