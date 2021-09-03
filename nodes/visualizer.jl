using Pkg; Pkg.activate(joinpath(@__DIR__, ".."))
using CoordinateTransformations
using TrajOptPlots
using RobotZoo
using MeshCat
using Colors

include(joinpath(@__DIR__, "quadrotor_model.jl"))
include(joinpath(@__DIR__, "constants.jl"))
include(joinpath(@__DIR__, "utils", "PubSubBuilder.jl"))
include(joinpath(@__DIR__, "..", "msgs", "filtered_state_msg_pb.jl"))

struct QuadVisualizer
    vis::MeshCat.Visualizer
    model::RExQuad
end
function QuadVisualizer()
    vis = MeshCat.Visualizer()
    model = gen_quadrotormodel()
    QuadVisualizer(vis, model)
end

function Base.open(vis::QuadVisualizer)
    MeshCat.open(vis.vis)
    TrajOptPlots.set_mesh!(vis.vis, vis.model)
end

function visualize_filteredstate(vis::QuadVisualizer)
    ctx = Context(1)
    state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                            quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                            vel_x=0., vel_y=0., vel_z=0.,
                            ang_x=0., ang_y=0., ang_z=0.)
    state_sub() = PubSubBuilder.subscriber_thread(ctx, state, filtered_state_sub_ip, filtered_state_sub_port)
    state_thread = Task(state_sub)
    schedule(state_thread)
    try
        while true
            # TODO: Run controller here
            TrajOptPlots.visualize!(vis, state)
        end
    catch e
        close(ctx)
        if e isa InterruptException
            println("Process terminated by you")
        else
            rethrow(e)
        end
    end
end

function TrajOptPlots.visualize!(vis::QuadVisualizer, x::AbstractVector)
    TrajOptPlots.visualize!(vis.vis, vis.model, x)
end

function TrajOptPlots.visualize!(vis::QuadVisualizer, x::FILTERED_STATE)
    TrajOptPlots.visualize!(vis, statemsg2vector(x))
end

function TrajOptPlots._set_mesh!(vis, model::RExQuad; scaling=1.0, color=colorant"black")
    obj = joinpath(mesh_folder, "quadrotor_scaled.obj")
    robot_obj = MeshFileGeometry(obj)
    mat = MeshPhongMaterial(color=color)
    setobject!(vis["geom"], robot_obj, mat)
    model.ned && settransform!(vis["geom"], LinearMap(RotX(pi)))
end
function TrajOptPlots.visualize!(vis, model::RobotZoo.Quadrotor, x::StaticVector, addrobot::Bool = true)
    xbar = RBState(model, x)
    if model.ned
        r = position(xbar)
        v = linear_velocity(xbar)
        r = SA[r[1],-r[2],-r[3]]
        v = SA[v[1],-v[2],-v[3]]
        xbar = RBState(r, RotX(pi)*orientation(xbar), v, angular_velocity(xbar)) 
    end
    TrajOptPlots.visualize!(vis, xbar, addrobot)
end