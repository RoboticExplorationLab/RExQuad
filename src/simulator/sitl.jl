struct SITL
    ctx::ZMQ.Context
    pub::ZMQ.Socket   # send out mocap/imu measurements
    sub::ZMQ.Socket   # receive control/state estimate
    buf_in::ZMQ.Message
    opts::Dict{Symbol,Real}
    xhat::Vector{Float64}
    u::Vector{Float64}
end

function SITL(pub_port=5555, sub_port=pub_port+1)
    ctx = ZMQ.Context()
    pub = ZMQ.Socket(ctx, ZMQ.PUB)
    try
        ZMQ.bind(pub, "tcp://*:$pub_port")
        println("Connected to Publisher at port $pub_port")
    catch err
        if err isa StateError
            throw(StateError("Error opening Pubisher"))
        else
            rethrow(err)
        end
    end

    sub = ZMQ.Socket(ctx, ZMQ.SUB)
    try
        ZMQ.subscribe(sub)
        set_conflate(sub, 1)
        ZMQ.bind(sub, "tcp://127.0.0.1:$sub_port")
        println("Connected to Subscriber at port $sub_port")
    catch err
        close(pub)
        if err isa StateError
            throw(StateError("Error opening Subscriber"))
        else
            rethrow(err)
        end
    end
    opts = Dict{Symbol, Real}(
        :recvtimeout_ms => 500,
    )
    buf_in = ZMQ.Message(2 * msgsize(StateControlMsg))
    xhat = zeros(13)
    xhat[4] = 1
    u = trim_controls() 
    return SITL(ctx, pub, sub, buf_in, opts, xhat, u)
end

function initialize!(sitl::SITL, x0; kwargs...)
    return nothing
end

function recv_with_timeout(sub, buf, timeout_ms)
    tstart = time_ns()
    timeout_ns = timeout_ms * 1e6
    bytes_read = 0
    while (time_ns() - tstart < timeout_ns)
        bytes_read = ZMQ.msg_recv(sub, buf, ZMQ.ZMQ_DONTWAIT)
        bytes_read > 0 && break
    end
    # ZMQ.getproperty(sub, :events)
    return bytes_read 
end

"""
If `y` is a MeasurementMsg, expects a StateMsg
If `y` is a StateMsg, expects a ControlMsg
"""
function sendandreceive!(sitl::SITL, y)

    # Send message
    zmsg = ZMQ.Message(msgsize(y))
    copyto!(zmsg, y)
    ZMQ.send(sitl.pub, zmsg)
    
    # Wait for response
    bytes_read = recv_with_timeout(sitl.sub, sitl.buf_in, sitl.opts[:recvtimeout_ms])

    if bytes_read > 0 
        id = sitl.buf_in[1]
        if y isa MeasurementMsg && id == msgid(StateMsg)
            state = StateMsg(sitl.buf_in)
            sitl.xhat .= getstate(state)
        elseif y isa StateMsg && id == msgid(ControlMsg)
            control = ControlMsg(sitl.buf_in)
            sitl.u .= getcontrol(control)
        else
            @warn "Didn't get the expected response."
        end
    else
        @warn "Didn't receive any data before timing out. Is the interface process running?"
    end
    return bytes_read
end

function getcontrol(sitl::SITL, x, y, t)
    y_state = StateMsg(x)
    sendandreceive!(sitl, y_state)
    return sitl.u
end

function get_state_estimate!(sitl::SITL, y_imu, y_mocap, dt)
    y_msg = MeasurementMsg(y_mocap..., 0, 0, 0, y_imu...)
    sendandreceive!(sitl, y_msg)
    return sitl.xhat
end

function finish(sitl::SITL)
    ZMQ.close(sitl.pub)
    ZMQ.close(sitl.sub)
end