using Pkg; Pkg.activate(joinpath(@__DIR__, "..", ".."))
using ProtoBuf
using BenchmarkTools
using SerialCOBS
using ZMQ

function testwrite(iob, message)
    message.pos_x = 1
    # msg_size = writeproto(iob, message)
    # return msg_size
    return nothing
end
include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                        quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                        vel_x=0., vel_y=0., vel_z=0.,
                        ang_x=0., ang_y=0., ang_z=0.,
                        time=0.)
iob = IOBuffer()
ctx = ZMQ.Context(1)
socket = ZMQ.Socket()

function testwrite2(socket, iob, message, meta, msg)
    # a = message.pos_x
    # b = message.pos_y
    msg_size = writeproto(iob, message, meta)
    zmsg = Message(msg_size)
    copyto!(zmsg, 1, iob.data, 1, msg_size)
    ZMQ.send(socket, msg)
    # return a + b
    return nothing
end
using TOML
setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

testwrite2(socket, iob, state, metadata, msg)
@which ZMQ._send(socket, msg, false)
@btime testwrite2($socket, $iob, $state, $metadata, $msg)
ZMQ.msg_send(socket, msg, (ZMQ.ZMQ_SNDMORE*false) | ZMQ.ZMQ_DONTWAIT)
length(msg)

metadata = meta(typeof(state))
attributes = metadata.ordered
attrib = attributes[1]
attrib.ptyp
getproperty(state, attrib.fld)

iob = IOBuffer()
writeproto(iob, state)

function testread(iob, msg)
    readproto(iob, msg)
end

@btime readproto($iob, $state, $metadata)
seek(iob, 0)
io = iob
fldnum, wiretyp = ProtoBuf._read_key(io)
fldnum = Int(fldnum)
attrib = get(metadata.numdict, fldnum, nothing)

attrib = get(metadata.numdict, fldnum, nothing)
@btime begin
    seek($io, 1)
    ProtoBuf.read_field($io, $state, $attrib, $wiretyp, nothing)
    # ProtoBuf.read_fixed($iob, Float64)
    # reinterpret(Float64, ProtoBuf._read_fixed($io, convert(UInt64, 0), 8))
end
@which ProtoBuf.read_fixed(io, Float64)
ProtoBuf.read_field(io, state, attrib, wiretyp, nothing)
position(io)

seek(iob, 0)
@which readproto(iob, state)
@btime convert($IOStream, $zmsg)

