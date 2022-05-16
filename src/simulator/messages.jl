
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

function bytestofloat2(buf)
    reinterpret(Float32, UInt32(buf[1]) | UInt32(buf[2]) << 8 | UInt32(buf[3]) << 16 | UInt32(buf[4]) << 24)
end

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

struct PoseMsg
    x::Float32   # position
    y::Float32
    z::Float32
    qw::Float32  # orientation
    qx::Float32
    qy::Float32
    qz::Float32
end
function PoseMsg(buf::AbstractVector{UInt8}, off::Integer = 0)
    PoseMsg(
        bytestofloat(buf, 0 * 4 + off),
        bytestofloat(buf, 1 * 4 + off),
        bytestofloat(buf, 2 * 4 + off),
        bytestofloat(buf, 3 * 4 + off),
        bytestofloat(buf, 4 * 4 + off),
        bytestofloat(buf, 5 * 4 + off),
        bytestofloat(buf, 6 * 4 + off),
    )
end
function Base.copyto!(buf::AbstractVector{UInt8}, msg::PoseMsg, off::Integer=0)
    floattobytes!(buf, msg.x, 0 * 4 + off)
    floattobytes!(buf, msg.y, 1 * 4 + off)
    floattobytes!(buf, msg.z, 2 * 4 + off)
    floattobytes!(buf, msg.qw, 3 * 4 + off)
    floattobytes!(buf, msg.qx, 4 * 4 + off)
    floattobytes!(buf, msg.qy, 5 * 4 + off)
    floattobytes!(buf, msg.qz, 6 * 4 + off)
end

struct ControlMsg
    u1::Float32  # front left
    u2::Float32  # front right
    u3::Float32  # back right
    u4::Float32  # back left
end
function ControlMsg(buf::AbstractVector{UInt8}, off::Integer=0)
    ControlMsg(
        bytestofloat(buf, 0 * 4 + off),
        bytestofloat(buf, 1 * 4 + off),
        bytestofloat(buf, 2 * 4 + off),
        bytestofloat(buf, 3 * 4 + off),
    )
end
function Base.copyto!(buf::AbstractVector{UInt8}, msg::PoseMsg, off::Integer=0)
    floattobytes!(buf, msg.u1, 0 * 4 + off)
    floattobytes!(buf, msg.u2, 1 * 4 + off)
    floattobytes!(buf, msg.u3, 2 * 4 + off)
    floattobytes!(buf, msg.u4, 3 * 4 + off)
end
function tovector(msg::ControlMsg)
    SA[msg.u1, msg.u2, msg.u3, msg.u4]
end
function ControlMsg(u::AbstractVector{<:Real})
    ControlMsg(u[1], u[2], u[3], u[4])
end

struct StateControlMsg
    x::Float32   # position
    y::Float32
    z::Float32
    qw::Float32  # orientation
    qx::Float32
    qy::Float32
    qz::Float32
    vx::Float32  # linear acceleration
    vy::Float32
    vz::Float32
    wx::Float32  # angular velocity 
    wy::Float32
    wz::Float32
    u::SVector{4,Float32}
end

function copyto!(buf::AbstractVector{UInt8}, msg::StateControlMsg, off=0)
    floattobytes!(buf, msg.x, 0 * 4 + off)
    floattobytes!(buf, msg.y, 1 * 4 + off)
    floattobytes!(buf, msg.z, 2 * 4 + off)
    floattobytes!(buf, msg.qw, 3 * 4 + off)
    floattobytes!(buf, msg.qx, 4 * 4 + off)
    floattobytes!(buf, msg.qy, 5 * 4 + off)
    floattobytes!(buf, msg.qz, 6 * 4 + off)
    floattobytes!(buf, msg.vx, 7 * 4 + off)
    floattobytes!(buf, msg.vy, 8 * 4 + off)
    floattobytes!(buf, msg.vz, 9 * 4 + off)
    floattobytes!(buf, msg.wx, 10 * 4 + off)
    floattobytes!(buf, msg.wy, 11 * 4 + off)
    floattobytes!(buf, msg.wz, 12 * 4 + off)
    floattobytes!(buf, msg.u[1], 13 * 4 + off)
    floattobytes!(buf, msg.u[2], 14 * 4 + off)
    floattobytes!(buf, msg.u[3], 15 * 4 + off)
    floattobytes!(buf, msg.u[4], 16 * 4 + off)
    buf
end

function StateControlMsg(buf::AbstractVector{UInt8})
    u = SA_F32[
      bytestofloat(buf, 13 * 4),
      bytestofloat(buf, 14 * 4),
      bytestofloat(buf, 15 * 4),
      bytestofloat(buf, 16 * 4),
    ]
    StateControlMsg(
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
        u
    )
end