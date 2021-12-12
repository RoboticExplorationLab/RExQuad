import Mercury as Hg

RUNNING_NODES = Vector{Hg.Node}()

GROUND_LINK_TASK = Task(_->1+1)
JETSON_LINK_TASK = Task(_->1+1)
STATE_ESTIMATOR_TASK = Task(_->1+1)
LQR_CONTROLLER_TASK = Task(_->1+1)
MOTOR_SPIN_UP_TASK = Task(_->1+1)


function launch_ground_link()
    node = GroundLink.main(; rate = 33.0, debug = false);
    node_task = Threads.@spawn Hg.launch(node)

    push!(RUNNING_NODES, node)
    GROUND_LINK_TASK = node_task
    return
end

function launch_vicon_listener(; rate = 100.0, debug = false)
    node = ViconListener.ViconListenerNode(rate, debug, )
    Hg.setupIO!(node, Hg.getIO(node))
    node_task = Threads.@spawn Hg.launch(node)

    push!(RUNNING_NODES, node)
    return
end

function launch_jetson_link(; rate=33.0, debug=false)
    node = JetsonLink.JetsonLinkNode(rate, debug)
    Hg.setupIO!(node, Hg.getIO(node))
    node_task = Threads.@spawn Hg.launch(node)

    push!(RUNNING_NODES, node)
    JETSON_LINK_TASK = node_task
    return
end

function launch_state_estimator(; rate = 100.0, debug = false)
    node = StateEstimator.StateEsitmatorNode(rate, debug)
    Hg.setupIO!(node, Hg.getIO(node))
    node_task = Threads.@spawn Hg.launch(node)

    push!(RUNNING_NODES, node)
    STATE_ESTIMATOR_TASK = node_task
    return
end

function launch_lqr_controller(; rate = 100.0, debug = false)
    node = LQRcontroller.LQRcontrollerNode(rate, debug)
    Hg.setupIO!(node, Hg.getIO(node))
    node_task = Threads.@spawn Hg.launch(node)

    push!(RUNNING_NODES, node)
    LQR_CONTROLLER_TASK = node_task
    return node_task
end

function launch_motor_spin_up(; rate=100.0, debug = false)
    node = MotorSpinUp.MotorSpinNode(rate, debug)
    Hg.setupIO!(node, Hg.getIO(node))
    node_task = Threads.@spawn Hg.launch(node)

    push!(RUNNING_NODES, node)
    MOTOR_SPIN_UP_TASK = node_task
    return node_task
end


function stopall()
    for node in RUNNING_NODES
        Hg.stopnode(node)
    end

    global RUNNING_NODES = Vector{Hg.Node}()
    return
end

Base.atexit(stopall)
