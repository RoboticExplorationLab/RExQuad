# This node is run of the Jetson, it is a MPC controller to
# stabilize the quadrotor around a hover
module MPCController
    using TOML
    using ZMQ
    using ProtoBuf
    using TrajectoryOptimization
    using Altro
    using RobotDynamics
    const RD = RobotDynamics
    const TO = TrajectoryOptimization

    using LinearAlgebra
    using StaticArrays
    using ForwardDiff
    using BlockDiagonals
    using ControlSystems
    
    include("$(@__DIR__)/../utils/PubSubBuilder.jl")
    using .PubSubBuilder

    include("$(@__DIR__)/../../msgs/filtered_state_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/motors_msg_pb.jl")
    include("$(@__DIR__)/../../msgs/messaging.jl")

    include("mpc_helper.jl")


    function motor_commander(filtered_state_sub_ip::String, filtered_state_sub_port::String,
                             motor_pub_ip::String, motor_pub_port::String,
                             serial_port::String, baud_rate::Int;
                             freq::Int64=200, debug::Bool=false)
        rate = 1/freq
        ctx = Context(1)
        ard = Arduino(serial_port, baud_rate);

        # Initalize Subscriber threads
        state = FILTERED_STATE(pos_x=0., pos_y=0., pos_z=0.,
                               quat_w=0., quat_x=0., quat_y=0., quat_z=0.,
                               vel_x=0., vel_y=0., vel_z=0.,
                               ang_x=0., ang_y=0., ang_z=0.)
        state_sub() = subscriber_thread(ctx, state, filtered_state_sub_ip, filtered_state_sub_port)
        # Setup and Schedule Subscriber Tasks
        state_thread = Task(state_sub)
        schedule(state_thread)

        # Setup Filtered state publisher
        motors = MOTORS(front_left=0., front_right=0., back_right=0., back_left=0.,
                        time=0.)
        motors_pub = create_pub(ctx, motor_pub_ip, motor_pub_port)
        pb = PipeBuffer()

        state_time = time()

        quadstate = zeros(Float64, 13)

        try
            while true
                # Prediction
                if state.time > state_time
                    # Save state 
                    quadstate[1] = state.pos_x
                    quadstate[2] = state.pos_y
                    quadstate[3] = state.pos_z
                    quadstate[4] = state.quat_x
                    quadstate[5] = state.quat_y
                    quadstate[6] = state.quat_z
                    quadstate[7] = state.quat_w
                    quadstate[8] = state.vel_x
                    quadstate[9] = state.vel_y
                    quadstate[10] = state.vel_z
                    quadstate[11] = state.ang_x
                    quadstate[12] = state.ang_y
                    quadstate[13] = state.ang_z

                    # Run  MPC controller
                    usend = mpc_controller(quadstate)

                    # Each motor is a force 
                    motors.front_left = usend[1]
                    motors.front_right = usend[2]
                    motors.back_right = usend[3]
                    motors.back_left = usend[4]

                    # Linear conversion from force to pwm

                    # Send the force to the motors
                    msg_size = writeproto(pb, motors);
                    message(ard, take!(pb))

                    # Send the force to the motors
                    publish(motors_pub, motors)

                    state_time = state.time
                    sleep(rate)
                    GC.gc(false)
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

    
    function main()
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        zmq_filtered_state_ip = setup_dict["zmq"]["jetson"]["filtered_state"]["server"]
        zmq_filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state"]["port"]

        zmq_motors_state_ip = setup_dict["zmq"]["jetson"]["motors"]["server"]
        zmq_motors_state_port = setup_dict["zmq"]["jetson"]["motors"]["port"]

        serial_port = setup_dict["serial"]["jetson"]["motors_arduino"]["serial_port"]
        baud_rate = setup_dict["serial"]["jetson"]["motors_arduino"]["baud_rate"]

        mtr_pub() = motor_commander(zmq_filtered_state_ip, zmq_filtered_state_port,
                                    zmq_motors_state_ip, zmq_motors_state_port,
                                    serial_port, baud_rate;
                                    freq=200, debug=false)
        mtr_thread = Task(mtr_pub)
        schedule(mtr_thread)

        return mtr_thread
    end
end
