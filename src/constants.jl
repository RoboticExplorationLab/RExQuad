"""
This file constains constants that are used across various components of the
autonomy stack.
"""

"""
The location of the data file containing the LQR gain for level, stable flight.
"""
const LQR_gain_file = joinpath(@__DIR__, "data", "lqr_gain.json")
const LQR_equilibrium_input_file = joinpath(@__DIR__, "data", "u_equilibrium.json")

"""
The location of the mesh files
"""
const mesh_folder = joinpath(@__DIR__, "data", "meshes")

const MIN_THROTLE = 1148.0f0
const MAX_THROTLE = 1832.0f0

function zero_FILTERED_STATE()
    state = FILTERED_STATE(
        pos_x = 0.0, pos_y = 0.0, pos_z = 0.0,
        quat_w = 1.0, quat_x = 0.0, quat_y = 0.0, quat_z = 0.0,
        vel_x = 0.0, vel_y = 0.0, vel_z = 0.0,
        ang_x = 0.0, ang_y = 0.0, ang_z = 0.0,
        time = time(),
    )
    return state
end

function zero_MOTORS()
    motors = MOTORS(
        front_left = MIN_THROTLE,
        front_right = MIN_THROTLE,
        back_right = MIN_THROTLE,
        back_left = MIN_THROTLE,
        time = time(),
    )
    return motors
end

function zero_QUAD_INFO()
    quad_info = QUAD_INFO(
        state = zero_FILTERED_STATE(),
        input = zero_MOTORS(),
        time = time(),
    )
    return quad_info
end

function zero_GROUND_INFO()
    ground_info = GROUND_INFO(
        deadman = true,
        time = time(),
    )
    return ground_info
end