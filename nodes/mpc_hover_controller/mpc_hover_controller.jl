
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


    #Quaternion stuff
    function hat(v)
        return [0 -v[3] v[2];
                v[3] 0 -v[1];
                -v[2] v[1] 0]
    end
    function L(q)
        s = q[1]
        v = q[2:4]
        L = [s    -v';
             v  s*I+hat(v)]
        return L
    end
    T = Diagonal([1; -ones(3)])
    H = [zeros(1,3); I]
    function qtoQ(q)
        return H'*T*L(q)*T*L(q)*H
    end
    function G(q)
        G = L(q)*H
    end
    function rptoq(ϕ)
        (1/sqrt(1+ϕ'*ϕ))*[1; ϕ]
    end
    function qtorp(q)
        q[2:4]/q[1]
    end

    #Quadrotor parameters
    m = 0.5
    ℓ = 0.1750
    J = [0.01566089 0.00000318037 0; 0.00000318037 0.01562078 0; 0 0 0.02226868]
    g = 9.81
    kt=1.0
    km=0.0245

    h = 0.02 #50 Hz

    Nx = 13     # number of states (quaternion)
    Nx̃ = 12     # number of states (linearized)
    Nu = 4     # number of controls
    Tfinal = 5.0 # final time
    Nt = Int(Tfinal/h)+1    # number of time steps
    thist = Array(range(0,h*(Nt-1), step=h));

function E(q)
    E = BlockDiagonal([1.0*I(3), G(q), 1.0*I(6)])
end

function quad_dynamics(x,u)
    r = x[1:3]
    q = x[4:7]
    v = x[8:10]
    ω = x[11:13]
    Q = qtoQ(q)
    
    ṙ = Q*v
    q̇ = 0.5*L(q)*H*ω
    
    v̇ = Q'*[0; 0; -g] + (1/m)*[zeros(2,4); kt*ones(1,4)]*u - hat(ω)*v
    
    ω̇ = J\(-hat(ω)*J*ω + [0 ℓ*kt 0 -ℓ*kt; -ℓ*kt 0 ℓ*kt 0; km -km km -km]*u)
    
    return [ṙ; q̇; v̇; ω̇]
end

#for simulation purposes
function quad_dynamics_rk4(x,u)
    #RK4 integration with zero-order hold on u
    f1 = quad_dynamics(x, u)
    f2 = quad_dynamics(x + 0.5*h*f1, u)
    f3 = quad_dynamics(x + 0.5*h*f2, u)
    f4 = quad_dynamics(x + h*f3, u)
    xn = x + (h/6.0)*(f1 + 2*f2 + 2*f3 + f4)
    xn[4:7] .= xn[4:7]/norm(xn[4:7]) #re-normalize quaternion
    return xn
end

#Initial Conditions
uhover = (m*g/4)*ones(4)
r0 = [0.0; 0; 1.0]
q0 = [1.0; 0; 0; 0]
v0 = zeros(3)
ω0 = zeros(3)
x0 = [r0; q0; v0; ω0]
x̃0 = [r0; 0; 0; 0; v0; ω0];

#Linearize dynamics about hover
A = ForwardDiff.jacobian(x->quad_dynamics_rk4(x,uhover),x0)
B = ForwardDiff.jacobian(u->quad_dynamics_rk4(x0,u),uhover);

#Reduced system
Ã = Array(E(q0)'*A*E(q0))
B̃ = Array(E(q0)'*B);

# Cost weights
Q = Array(I(Nx̃));
R = Array(.1*I(Nu));

#LQR Gain

S = dare(Ã,B̃,Q,R) #cost-to-go
K = dlqr(Ã,B̃,Q,R);

#LQR Feedback controller
function lqr_controller(x)
    
    q0 = x0[4:7]
    q = x[4:7]
    ϕ = qtorp(L(q0)'*q)
    
    Δx̃ = [x[1:3]-r0; ϕ; x[8:10]-v0; x[11:13]-ω0]
    
    u = uhover - K*Δx̃
end

#MPC Setup
Nh = 50 #MPC horizon length (number of time steps)
model = RD.LinearModel(Ã, B̃;dt = h)
Qf = S #use LQR cost-to-go as terminal cost
objective = LQRObjective(Q, R, Qf, x̃0, Nh)

# Constraints
constraints = ConstraintList(Nx̃, Nu, Nh)
bound = BoundConstraint(Nx̃, Nu, u_min=-uhover, u_max=uhover)
add_constraint!(constraints, bound, 1:Nh)


problem = Problem(model, objective, x̃0, (Nh-1)*h, x0=x̃0, constraints=constraints, integration=RD.PassThrough)
solver = ALTROSolver(problem)

tol=1e-4
set_options!(solver,
    show_summary=false,
    projected_newton=false,
    constraint_tolerance=tol,
    cost_tolerance=tol,
    cost_tolerance_intermediate=tol,
    penalty_initial=10.,
    penalty_scaling=10.,
)

# Solve initial problem  ("cold start")
solve!(solver);

#MPC Feedback controller
function mpc_controller(x)
    
    q0 = x0[4:7]
    q = x[4:7]
    ϕ = qtorp(L(q0)'*q)
    
    Δx̃ = [x[1:3]-r0; ϕ; x[8:10]-v0; x[11:13]-ω0]
    
    problem.x0 .= Δx̃
    solve!(solver)
    Δu = controls(solver)
    
    u = uhover + Δu[1]

    return u
    
end






    function motor_commander(filtered_state_sub_ip::String, filtered_state_sub_port::String,
                             motor_pub_ip::String, motor_pub_port::String;
                             freq::Int64=200, debug::Bool=false)
        ctx = Context(1)

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
        iob = PipeBuffer()

        state_time = time()

        try
            while true
                # Prediction
                if state.time > state_time
		    #save state 
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
                    #Run  MPC controller

                    usend = mpc_controller(quadstate)
		    #each motor is a force 
                    motors.front_left = usend[1]
		    motors.front_right = usend[2]
		    motors.back_right = usend[3]
		    motors.back_left = usend[4]

		    #linear conversion from force to be added above 

		    #send the force to the motors
                    writeproto(iob, motors)
                    ZMQ.send(motors_pub, take!(iob))

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

    
    function main()
        setup_dict = TOML.tryparsefile("$(@__DIR__)/../setup.toml")

        zmq_filtered_state_ip = setup_dict["zmq"]["jetson"]["filtered_state"]["server"]
        zmq_filtered_state_port = setup_dict["zmq"]["jetson"]["filtered_state"]["port"]
        zmq_motors_state_ip = setup_dict["zmq"]["jetson"]["motors"]["server"]
        zmq_motors_state_port = setup_dict["zmq"]["jetson"]["motors"]["port"]

        fs_pub() = motor_commander(zmq_filtered_state_ip, zmq_filtered_state_port,
                                   zmq_motors_state_ip, zmq_motors_state_port;
                                   freq=200, debug=false)
        fs_thread = Task(fs_pub)
        schedule(fs_thread)

        return fs_thread
    end
end
