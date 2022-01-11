import Mercury as Hg

RUNNING_NODES = Vector{Hg.Node}()

GROUND_MASTER_TASK = 0
JETSON_MASTER_TASK = 0
STATE_ESTIMATOR_TASK = 0
LQR_CONTROLLER_TASK = 0
MOTOR_SPIN_UP_TASK = 0


function launch_ground_master(; rate = 100.0, debug = false)
    node = GroundMaster.GroundMasterNode(rate, debug);
    Hg.setupIO!(node, Hg.getIO(node))
    node_task = Threads.@spawn Hg.launch(node)

    push!(RUNNING_NODES, node)
    GROUND_MASTER_TASK = node_task
    return
end

function ground_master_running()
    if GROUND_MASTER_TASK isa Task
        return istaskdone(GROUND_MASTER_TASK)
    else
        return false
    end
end

function launch_vicon_listener(; rate = 100.0, debug = false)
    node = ViconListener.ViconListenerNode(rate, debug, )
    Hg.setupIO!(node, Hg.getIO(node))
    node_task = Threads.@spawn Hg.launch(node)

    push!(RUNNING_NODES, node)
    return
end

function launch_jetson_master(; rate=33.0, debug=false)
    node = JetsonMaster.JetsonMasterNode(rate, debug)
    Hg.setupIO!(node, Hg.getIO(node))
    node_task = Threads.@spawn Hg.launch(node)

    push!(RUNNING_NODES, node)
    JETSON_MASTER_TASK = node_task
    return
end

function jetson_master_running()
    if JETSON_MASTER_TASK isa Task
        return istaskdone(JETSON_MASTER_TASK)
    else
        return false
    end
end

function launch_state_estimator(; rate = 100.0, debug = false)
    node = StateEstimator.StateEsitmatorNode(rate, debug)
    Hg.setupIO!(node, Hg.getIO(node))
    node_task = Threads.@spawn Hg.launch(node)

    push!(RUNNING_NODES, node)
    STATE_ESTIMATOR_TASK = node_task
    return
end

function state_estimator_running()
    if STATE_ESTIMATOR_TASK isa Task
        return istaskdone(STATE_ESTIMATOR_TASK)
    else
        return false
    end
end

function launch_lqr_controller(; rate = 100.0, debug = false)
    node = LQRcontroller.LQRcontrollerNode(rate, debug)
    Hg.setupIO!(node, Hg.getIO(node))
    node_task = Threads.@spawn Hg.launch(node)

    push!(RUNNING_NODES, node)
    LQR_CONTROLLER_TASK = node_task
    return node_task
end

function lqr_controller_running()
    if LQR_CONTROLLER_TASK isa Task
        return istaskdone(LQR_CONTROLLER_TASK)
    else
        return false
    end
end

function launch_motor_spin_up(; rate=100.0, debug = false)
    node = MotorSpinUp.MotorSpinNode(rate, debug)
    Hg.setupIO!(node, Hg.getIO(node))
    node_task = Threads.@spawn Hg.launch(node)

    push!(RUNNING_NODES, node)
    MOTOR_SPIN_UP_TASK = node_task
    return node_task
end

function motor_spin_up_running()
    if MOTOR_SPIN_UP_TASK isa Task
        return istaskdone(MOTOR_SPIN_UP_TASK)
    else
        return false
    end
end

function stopall()
    for node in RUNNING_NODES
        Hg.stopnode(node)
    end

    global RUNNING_NODES = Vector{Hg.Node}()
    return
end

Base.atexit(stopall)
