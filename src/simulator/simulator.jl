import Pkg; Pkg.activate(@__DIR__)
using ZMQ
using Sockets
using MeshCat

include("dynamics.jl")

struct MeasurementMsg
    x::Float32   # position
    y::Float32
    z::Float32
    qw::Float32  # orientation
    qx::Float32
    qy::Float32
    qz::Float32
    ax::Float32  # linear acceleration
    ay::Float32
    az::Float32
    wx::Float32  # angular velocity 
    wy::Float32
    wz::Float32
end
function MeasurementMsg()
    MeasurementMsg(
        0f0, 0f0, 0f0,
        1f0, 0f0, 0f0, 0f0,
        0f0, 0f0, 0f0,
        0f0, 0f0, 0f0,
    )
end

function copyto!(buf::AbstractVector{UInt8}, msg::MeasurementMsg)
    floattobytes!(buf, msg.x, 0 * 4)
    floattobytes!(buf, msg.y, 1 * 4)
    floattobytes!(buf, msg.z, 2 * 4)
    floattobytes!(buf, msg.qw, 3 * 4)
    floattobytes!(buf, msg.qx, 4 * 4)
    floattobytes!(buf, msg.qy, 5 * 4)
    floattobytes!(buf, msg.qz, 6 * 4)
    floattobytes!(buf, msg.ax, 7 * 4)
    floattobytes!(buf, msg.ay, 8 * 4)
    floattobytes!(buf, msg.az, 9 * 4)
    floattobytes!(buf, msg.wx, 10 * 4)
    floattobytes!(buf, msg.wy, 11 * 4)
    floattobytes!(buf, msg.wz, 12 * 4)
    buf
end

function MeasurementMsg(buf::AbstractVector{UInt8})
    MeasurementMsg(
        bytestofloat(buf, 0 * 4),
        bytestofloat(buf, 1 * 4),
        bytestofloat(buf, 2 * 4),
        bytestofloat(buf, 3 * 4),
        bytestofloat(buf, 4 * 4),
        bytestofloat(buf, 5 * 4),
        bytestofloat(buf, 6 * 4),
        bytestofloat(buf, 7 * 4),
        bytestofloat(buf, 8 * 4),
        bytestofloat(buf, 9 * 4),
        bytestofloat(buf, 10 * 4),
        bytestofloat(buf, 11 * 4),
        bytestofloat(buf, 12 * 4),
    )
end

function floattobytes!(buf::AbstractVector{UInt8}, f::Float32, offset = 0)
    mask = 0x0000_00ff
    fi = reinterpret(UInt32, f)
    buf[1 + offset] = fi & mask
    buf[2 + offset] = (fi >> 8) & mask
    buf[3 + offset] = (fi >> 16) & mask
    buf[4 + offset] = (fi >> 24) & mask
    buf 
end

function bytestofloat(buf::AbstractVector{UInt8}, offset=0)
    reinterpret(Float32, view(buf, (1:4) .+ offset))[1]
end

struct Simulator
    buf_out::ZMQ.Message
    ctx::ZMQ.Context
    pub::ZMQ.Socket
    vis::Visualizer
end

function Simulator(pub_port=5555)
    buf_out = ZMQ.Message(sizeof(MeasurementMsg))
    ctx = ZMQ.Context()
    pub = ZMQ.Socket(ctx, ZMQ.PUB)
    ZMQ.bind(pub, "tcp://*:$pub_port")

    vis = Visualizer()
    RobotMeshes.setdrone!(vis)
    Simulator(buf_out, ctx, pub, vis)
end

function finish(sim::Simulator)
    ZMQ.close(sim.pub)
end

function runsim(sim::Simulator, x0; dt=0.01)
    
    x = copy(x0)
    t = 0.0
    buf = ZMQ.Message(sizeof(MeasurementMsg))
    while true
        # Get control
        u = getcontrol(sim, x, t) 

        # Propagate dynamics
        x = dynamics_rk4(x, u, dt)

        # Visualize
        RobotMeshes.visualize!(sim.vis, sim, x)

        # Get measurement
        y = getmeasurement(vis, x, u, t)

        # Send measurement
        sendmeasurement(sim, y, t)

        t += dt
    end
end

function getcontrol(sim, x, t)
    # TODO: Get this from ZMQ
    return zeros(4)
end

function getmeasurement(vis::Simulator, x, u, t)
    # TODO: add noise
    xdot = cont_dynamics(x, u)
    MeasurementMsg(
        x[1], x[2], x[3],
        x[4], x[5], x[6], x[7],
        xdot[8], xdot[9], xdot[10],
        x[11], x[12], x[13]
    )
end

function sendmeasurement(sim::Simulator, y::MeasurementMsg, t)
    copyto!(sim.buf_out, y)
    ZMQ.send(sim.pub, sim.buf_out)
end


vis = Visualizer()
open(vis)
RobotMeshes.setdrone!(vis)
x = [0,0,1, 1,0,0,0, 0,0,0, 0,0,0.]
RobotMeshes.visualize!(vis, (), x)