# This node is run of the Jetson, it is a simple LQR controller to
# stabilize the quadrotor around a hover

module LqrHoverController
using TOML
using ZMQ
using ProtoBuf
using StaticArrays

include(joinpath(@__DIR__, "..", "constants.jl"))
include(joinpath(@__DIR__, "..", "quadrotor_model.jl"))

include("$(@__DIR__)/../utils/PubSubBuilder.jl")
using .PubSubBuilder

include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
include("$(@__DIR__)/../../msgs/motors_msg_pb.jl")
include("$(@__DIR__)/../../msgs/messaging.jl")

"""
    read_LQR_gain_from_file()

Reads the hover LQR gain from the data file.
"""
function read_LQR_gain_from_file()::SMatrix{4,12,Float64,48}
    file = open(LQR_gain_file, "r")
    data = JSON.parse(file)
    close(file)
    K = hcat(Vector{Float64}.(data)...)
    return SMatrix{4,12}(K)
end

function motor_commander(
    filtered_state_sub_ip::String,
    filtered_state_sub_port::String,
    motor_pub_ip::String,
    motor_pub_port::String;
    freq::Int64 = 200,
    debug::Bool = false,
)
    ctx = Context(1)

    # Subscribe to the filtered state
    state = FILTERED_STATE(
        pos_x = 0.0,
        pos_y = 0.0,
        pos_z = 0.0,
        quat_w = 0.0,
        quat_x = 0.0,
        quat_y = 0.0,
        quat_z = 0.0,
        vel_x = 0.0,
        vel_y = 0.0,
        vel_z = 0.0,
        ang_x = 0.0,
        ang_y = 0.0,
        ang_z = 0.0,
    )
    state_sub() =
        subscriber_thread(ctx, state, filtered_state_sub_ip, filtered_state_sub_port)
    state_thread = Task(state_sub)
    schedule(state_thread)

    # Setup motor command publisher
    motors = MOTORS(
        front_left = 0.0,
        front_right = 0.0,
        back_right = 0.0,
        back_left = 0.0,
        time = 0.0,
    )
    motors_pub = create_pub(ctx, motor_pub_ip, motor_pub_port)
    iob = PipeBuffer()

    # Set up controller
    model = gen_quadrotormodel()
    K = read_LQR_gain_from_file()

    r0 = SA[0.0; 0; 1.0]
    q0 = SA[1.0; 0; 0; 0]
    v0 = @MVector zeros(3)
    ω0 = @MVector zeros(3)
    x0 = RobotDynamics.build_state(model, r0, q0, v0, ω0)
    uhover = trim_controls(model)

    state_time = time()

    try
        while true
            # Prediction
            if state.time > state_time
                # TODO: Run controller here
                x = SA[
                    state.pos_x,
                    state.pos_y,
                    state.pos_z,
                    state.quat_w,
                    state.quat_x,
                    state.quat_y,
                    state_.quat_z,
                    state.vel_x,
                    state.vel_y,
                    state.vel_z,
                    state.ang_x,
                    state.ang_y,
                    state.ang_z,
                ]
                dx = RobotDynamics.state_diff(model, x, x0) # uses Cayley map by default
                du = K * dx
                u = du + uhover
                # controller()

                publish(imu_pub, imu_vicon.imu)

                state_time = state.time
            end
        end
    catch e
        close(motors_pub)
        close(ctx)

        if e isa InterruptException
            println("Process terminated by you")
        else
            rethrow(e)
        end
    end
end

# Launch IMU publisher
function main()
    setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

    zmq_filtered_state_ip = setup_dict["zmq"]["jetson"]["filtered_state"]["server"]
    zmq_filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state"]["port"]
    zmq_motors_state_ip = setup_dict["zmq"]["jetson"]["motors"]["server"]
    zmq_motors_state_port = setup_dict["zmq"]["jetson"]["motors"]["port"]

    fs_pub() = motor_commander(
        zmq_filtered_state_ip,
        zmq_filtered_state_port,
        zmq_motors_state_ip,
        zmq_motors_state_port;
        freq = 200,
        debug = false,
    )
    fs_thread = Task(fs_pub)
    schedule(fs_thread)

    return fs_thread
end
end
