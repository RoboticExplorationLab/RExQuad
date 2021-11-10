import Mercury as Hg

local GROUND_LINK_NODE
function launch_ground_link()
    node = GroundLink.main(; rate = 33.0, debug = false);
    node_task = Threads.@spawn Hg.launch(node)
    GROUND_LINK_NODE = node

    return node
end

local JETSON_LINK_NODE
function launch_jetson_link()
    node = JetsonLink.main(; rate = 33.0, debug=false);
    node_task = Threads.@spawn Hg.launch(node)
    JETSON_LINK_NODE = node

    return node
end

local STATE_ESTIMATOR_NODE
function launch_state_estimator()
    node = StateEstimator.main(; rate = 100.0, debug=false);
    node_task = Threads.@spawn Hg.launch(node)
    STATE_ESTIMATOR_NODE = node

    return node
end

local LQR_CONTROLLER_NODE
function launch_lqr_controller()
    node = LQRcontroller.main(; rate = 100.0, debug = false);
    node_task = Threads.@spawn Hg.launch(node)
    LQR_CONTROLLER_NODE = node

    return node
end

function stopall()
    if GROUND_LINK_NODE <: Hg.Node
        Hg.stopnode(GROUND_LINK_NODE)
    end
    if JETSON_LINK_NODE <: Hg.Node
        Hg.stopnode(JETSON_LINK_NODE)
    end
    if STATE_ESTIMATOR_NODE <: Hg.Node
        Hg.stopnode(STATE_ESTIMATOR_NODE)
    end
    if LQR_CONTROLLER_NODE <: Hg.Node
        Hg.stopnode(LQR_CONTROLLER_NODE)
    end
end