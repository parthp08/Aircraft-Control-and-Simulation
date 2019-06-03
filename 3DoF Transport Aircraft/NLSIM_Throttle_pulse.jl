# Nonlinear Time History Simulation
# for Simulated response of Transport aircraft
# to a Throttle Pulse

"""
# References
- [1] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control
and simulation: dynamics, controls design, and autonomous systems. John Wiley
& Sons. (page 196 & 197)
""" 
 
using DelimitedFiles
using Plots
# plotly()    # web based plotting backend
# gr()    # gui backend for ploting
pyplot()  # python based backend
# InspectDR()
include("RK4.jl")
include("transp.jl")    # importing State Equation file
name = transp

## Initial Trim Conditions
# initial condition is trim condition for
# VT = 250 ft/s , h = 0 ft, gamma = 0
# is used here as initial condition
# to examine.
initial_condition = [6,         # number of state variables
                    4,          # number of control inputs
                    250,        # initial_state,    # Velocity
                    0.16192,    # initial_state,    # AOA
                    0.16192,    # initial_state,    # theta, pitch attitude
                    0,          # initial_state,    # pitch rate, Q
                    0,          # initial_state,    # altitude
                    0,          # initial_state,    # x-position
                    0.1845,     # initial_control_inputs,    # throttle
                    -9.2184,    # initial_control_inputs,    # elevator
                    0.25,       # initial_control_inputs,    # cg position
                    0           # initial_control_inputs,    # landing config
]

# assign initial conditions to different variables
n = Int(initial_condition[1])  # number of states
m = Int(initial_condition[2])  # number of controls
x = initial_condition[3:n+2]  # initial state
u = initial_condition[n+3:n+m+2] # control input

RTOD = 57.29578
# change runtime to different values to see the full responses
runtime  = 100 # float(input("Enter Run-Time : "))
dt = 0.02  # float(input("Enter Integration Time-step : "))
N = Int(floor(runtime/dt))  # total number of steps for integration
# NP = int(max(1, N/500))
NP = 1
time = 0.0
xd = name(time, x, u)  # set variables in state equations
# initialize output y-array
y = zeros(Float64, N,n)    # (N , n) zero matrix   ## n could be more

# for throttle pulse,  example 3.6.4 [1]
save=u[1]	# trim throttle input

k = 0
for i = 1:N
    global k, x, u, y, NP, dt, save, name
	time = i*dt
	if i % NP == 0
		k += 1
		y[k,1] = x[1]    # record data as needed
		y[k,2] = x[2]
		y[k,3] = x[3]
		y[k,4] = x[4]
		y[k,5] = x[5]
    y[k,6] = x[6]
    end

	## Throttle Pulse
	if time >= 7   ### for example 3.6.4 [1]  # Elevator impulse
		u[1] = save
    elseif time >= 4
		u[1] = save - 0.1
    elseif time >= 1
		u[1] = save + 0.1
	else
		u[1] = save
    end

	# intergation step
	x = RK4(name, time, dt, x, u)
end

##====================================================
## Simulated Response to a Throttle Pulse
##-------------------------------------------
## Theta & Alpha Vs Time to visulalize
##  effect of Throttle pulse on flight path
t = NP*dt*Array(1:k)
plot(t, y[:, 3]*RTOD, w=1, 
    xlim=[0, runtime], ylim=[8.5, 10],
    label="Theta (deg)") # Theta and Alpha Vs Time
plot!(t, y[:,2]*RTOD, w=1, 
    xlim=[0, runtime], ylim=[8.5, 10],
    label="Alpha (deg)") # Theta and Alpha Vs Time
xlabel!("Time(sec)")
ylabel!("Theta & Alpha (deg.)")


# ## all states (X) Vs Time  (throttle pulse)  
t = NP*dt*Array(1:k)

p1 = plot(t, y[:, 1], w=1, xlim=[0, runtime], ylim=[230, 280], label="Velocity(ft/sec)")
xlabel!("Time(sec)")
ylabel!("Velocity VT (ft/sec)")

p2 = plot(t, y[:, 4], w=1, xlim=[0, runtime], ylim=[-0.004, 0.004], label="Pitch Rate")
xlabel!("Time(sec)")
ylabel!("Pitch Rate")

p3 = plot(t, y[:, 5], w=1, xlim=[0, runtime], ylim=[-15, 15], label="Altitude(ft)")
xlabel!("Time(sec)")
ylabel!("Altitude h (ft)")

p4 = plot(t, y[:, 3]*RTOD, w=1, 
        xlim=[0, runtime], ylim=[8.5, 10],
        label="Theta (deg)") # Theta and Alpha Vs Time
p4 = plot!(t, y[:,2]*RTOD, w=1, 
        xlim=[0, runtime], ylim=[8.5, 10],
        label="Alpha (deg)") # Theta and Alpha Vs Time
xlabel!("Time(sec)")
ylabel!("Theta & Alpha (deg.)")

plot(p1,p2,p3,p4, layout=(2,2), legend=true)

##(description of physical meaning on page 196, 197 of [1])


# # for plotting use
# y_array = y
# t_array = t
# A = hcat(t_array, y_array)
# # saving array to csv file
# DelimitedFiles.writedlm( "NLSIM_Throttle_pulse_output.csv",  A, ',')

