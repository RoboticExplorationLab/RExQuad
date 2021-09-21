using CoordinateTransformations
using TrajOptPlots
using RobotZoo
using MeshCat
using Colors

include(joinpath(@__DIR__, "..", "quadrotor_model.jl"))
include(joinpath(@__DIR__, "..", "constants.jl"))
include(joinpath(@__DIR__, "..", "..", "msgs", "filtered_state_msg_pb.jl"))

"""
A type for facilitating the visualization of the quadrotor in a 3D MeshCat
    environment
"""
struct QuadVisualizer
    vis::MeshCat.Visualizer
    model::RExQuad
end
function QuadVisualizer()
    vis = MeshCat.Visualizer()
    model = gen_quadrotormodel()
    QuadVisualizer(vis, model)
end

"""
Opens the visualizer in a web browser with the quadrotor model loaded
"""
function Base.open(vis::QuadVisualizer)
    MeshCat.open(vis.vis)
    TrajOptPlots.set_mesh!(vis.vis, vis.model)
end

# Define visualization methods for the RExQuad model
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

function add_copy!(vis::QuadVisualizer)
    TrajOptPlots.set_mesh!(vis.vis["copy"], vis.model, color=colorant"blue")
end

function TrajOptPlots.visualize!(vis::QuadVisualizer, x::AbstractVector, x2::AbstractVector)
    TrajOptPlots.visualize!(vis.vis, vis.model, x)
    TrajOptPlots.visualize!(vis.vis["copy"], vis.model, x2)
end