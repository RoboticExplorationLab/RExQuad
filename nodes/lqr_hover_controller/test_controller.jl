using Pkg; Pkg.activate(joinpath(@__DIR__, "..", ".."))
using JSON
include(joinpath(@__DIR__, "..", "constants.jl"))
include(joinpath(@__DIR__, "..", "visualizer.jl"))
include(joinpath(@__DIR__, "..", "quadrotor_model.jl"))
include(joinpath(@__DIR__, "generate_lqr_gains.jl"))

function read_LQR_gain_from_file()::SMatrix{4,12,Float64,48}
    file = open(LQR_gain_file, "r")
    data = JSON.parse(file)
    close(file)
    K = hcat(Vector{Float64}.(data)...)
    return SMatrix{4,12}(K)
end
vis = QuadVisualizer()
open(vis)

##
model = gen_quadrotormodel()
generate_LQR_hover_gains([10,10,100, 10,10,10, 1,1,1, 1,1,1],fill(1e-2,4))

# %%
r0 = SA[0.0; 0; 1.0]
q0 = SA[1.0; 0; 0; 0]
v0 = @MVector zeros(3)
ω0 = @MVector zeros(3)
x0 = RobotDynamics.build_state(model, r0, q0, v0, ω0)
uhover = trim_controls(model)

K = read_LQR_gain_from_file()

x_init = zeros(model)[1]
h = 1e-2
times = range(0, 3, step=h)
x = RobotDynamics.build_state(model, [0,0,0], RotX(deg2rad(0)), zeros(3), zeros(3))
visualize!(vis, x)
X = [x for t in times]
discrete_dynamics(RK4, model, x, uhover, 0.0, 0.1)
for (i,t) in enumerate(times)
    # visualize!(vis, x)
    dx = RobotDynamics.state_diff(model, x, x0)  # uses Cayley map by default
    du = -K*dx
    u = du + uhover
    x = discrete_dynamics(RK4, model, x, u, t, h)
    X[i] = x
end
x = x_init
[x[2] for x in X]
visualize!(vis.vis, vis.model, 3.0, X)