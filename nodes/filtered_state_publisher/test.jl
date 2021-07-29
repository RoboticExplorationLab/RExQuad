# module TestZMQ

# begin
    using BenchmarkTools
    using ZMQ
    using ProtoBuf

    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")

    function test()
        iob = IOBuffer(; sizehint=500)
        a = 0
        for i in 1:100
            write(iob, "Was it zero allocation?")
            a = read(iob)
        end
        return a
    end


    # iob = PipeBuffer()
    # filtered_state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
    #                                 quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
    #                                 vel_x=0., vel_y=0., vel_z=0.,
    #                                 ang_x=0., ang_y=0., ang_z=0.)
    # s1 = Socket(REP)
    # s2 = Socket(REQ)
    # bind(s1, "tcp://*:5557")
    # connect(s2, "tcp://localhost:5557")

    # try
    #     # while true
    #     for i in 1:1
    #         writeproto(iob, filtered_state)
    #         # println(take!(iob))
    #         println(iob.size)

    #         # send(s2, take!(iob))
    #         # bin_data = recv(s1)
    #         # io = seek(convert(IOStream, bin_data), 0)
    #         # readproto(io, filtered_state)

    #         # send(s1, take!(iob))
    #         # bin_data = recv(s2)
    #         # io = seek(convert(IOStream, bin_data), 0)
    #         # readproto(io, filtered_state)
    #     end
    # catch e
    #     println("Shutting down test.jl")
    #     rethrow(e)
    # finally
    #     close(s1)
    #     close(s2)
    # end
# end
