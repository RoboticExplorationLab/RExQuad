using StaticArrays: SVector
using Rotations: kinematics, RotMatrix, UnitQuaternion, params
using ForwardDiff: jacobian

"""
"""
function dynamics(state::ImuState, input::ImuInput)
    p, q, v, α, β = getComponents(state)
    v̇ᵢ, ωᵢ = getComponents(input)

    # Body velocity writen in inertia cooridantes
    ṗ = q * v
    # Compute the rotational kinematics
    q̇ = kinematics(q, ωᵢ - β)
    # Translational acceleration
    v̇ = v̇ᵢ - α

    # Rate of change in biases is 0
    α̇ = zeros(3)
    β̇ = zeros(3)

    return ImuState([ṗ; q̇; v̇; α̇; β̇])
end

"""
"""
function process(x::ImuState, u::ImuInput, dt::Float64)::ImuState
    k1 = dynamics(x, u)
    k2 = dynamics(x + 0.5 * dt * k1, u)
    k3 = dynamics(x + 0.5 * dt * k2, u)
    k4 = dynamics(x + dt * k3, u)
    xnext = x + (dt/6.0) * (k1 + 2*k2 + 2*k3 + k4)

    return ImuState(xnext)
end

"""
"""
function process_jacobian(state::ImuState, input::ImuInput, dt::Float64)
    return jacobian(st->process(ImuState(st), input, dt), SVector(state))
end

"""
"""
function measure(state::ImuState)::ViconMeasurement
    p, q, v, α, β = getComponents(state)
    return ViconMeasurement(p..., params(q)...)
end

"""
"""
function measure_jacobian(state::ImuState)
    return jacobian(st->measure(ImuState(st)), state)
end
