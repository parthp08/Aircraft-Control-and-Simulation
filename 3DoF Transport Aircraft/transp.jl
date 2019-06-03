include("adc.jl")

export transp


function transp(time::Float64, x::Array, u::Array)
	"""
	Mediaum-sized transport aircraft, Longitudinal Dynamics

    # References
    - [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
    and simulation: dynamics, controls design, and autonomous systems. John Wiley
    & Sons. (page 180)
	"""
	S = 2170.0		# wing area, ft^2
	cbar = 17.5
	mass = 5.0e3	# total mass of the aircraft, lb
	Iyy = 4.1e6		# pitch-axis inertia, slug-ft^2
	TSTAT = 6.0e4
	DTDV = -38.0		# change in Thrust due to change in velocity
	ZE = 2.0		# distance between thrust vector and CG
	CDCLS = 0.042	# Drag polar
	CLA = 0.085		# per degree  # change in Lift with change in AOA
	CMA = -0.022	# per degree  # change in Pitching Moment with change in AOA
	CMDE = -0.016	# per degree  # change in Pitching Moment with change in elevetor angle
	CMQ = -16.0		# per radian  # change in Pitching Moment with change in pitch rate
	CMADOT = -6.0	# per radian  # slope of CM -> AOA curve
	CLADOT = 0.0	# per radian  # slope of CL -> AOA curve
	RTOD = 57.29578	# radian to degree
	GD = 32.17		# gravity, ft/sec^2

	# initialize arrays
	# u = zeros(Float64, 4, 1)
	# x = zeros(Float64, 6, 1)	#(5,1)
	THTL = u[1]		# throttle
	ELEV = u[2]		# elevetor
	XCG = u[3]		# cg position
	LAND = u[4]		# landing gear
	VT = x[1]		# TAS, feet per sec
	ALPHA = RTOD*x[2] # AOA
	THETA = x[3]	# pitch attitude
	Q = x[4]		# pitch rate
	H = x[5] 		# altitude

	mach, Qbar = adc(VT, H)
	QS = Qbar*S  	# dynamic pressure
	SALP = sin(x[2])
	CALP = cos(x[2])
	GAM = THETA - x[2]
	SGAM = sin(GAM)
	CGAM = cos(GAM)
	if LAND == 0	# Clean
		CLO = 0.20
		CDO = 0.016
		CMO = 0.05
		DCDG = 0.0
		DCMG = 0.0
	elseif LAND == 1 	# Landing Flaps and Gear
		CLO = 1.0
		CDO = 0.08
		CMO = -0.20
		DCDG = 0.02
		DCMG = -0.05
	else
		println("Landing Gear and Flaps ??")
	end

	THR = (TSTAT + VT*DTDV) * max(THTL,0)		# Thrust
	CL = CLO + CLA*ALPHA					# Non-dimensional Lift
	CM = DCMG + CMO + CMA*ALPHA + CMDE*ELEV + CL*(XCG-0.25) # Moment
	CD = DCDG + CDO + CDCLS*CL*CL 				# Drag polar

	# State Equations Next
	xd = zeros(Float64, 6,1)
	xd[1] = (THR*CALP - QS*CD)/mass - GD*SGAM
	xd[2]=(-THR*SALP - QS*CL + mass*(VT*Q + GD*CGAM)) / (mass*VT + QS*CLADOT)
	xd[3] = Q
	D = 0.5*cbar*(CMQ*Q + CMADOT*xd[2]) / VT 	# PITCH DAMPING
	xd[4] = (QS*cbar*(CM + D) + THR*ZE) / Iyy 	# Q-DOT
	xd[5] = VT*SGAM 	# VERTICAL SPEED
	xd[6] = VT*CGAM 	# HORIZNTL. SPEED

	return xd
end

## Test 
# x = [10,2,1,1,1000]
# u = [0.3, 2, 0.4, 1]
# println(transp(0.0,x,u))
## answer
# [25.3338; 2.35907; 1.0; -0.0207585; -8.41471; 5.40302]

