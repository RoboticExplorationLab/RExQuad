include("rotations.jl")

mutable struct DelayedMEKF
    Wf::Diagonal{Float64,Vector{Float64}}  # mocap (measurement) noise
    Vf::Diagonal{Float64,Vector{Float64}}  # imu (process) noise
    delay_comp::Int
    imuhist::CircularBuffer{Vector{Float64}}
    xf::Vector{Float64}    # filter state
    Pf::Matrix{Float64}
    xd::Vector{Float64}    # delayed filter state
    Pd::Matrix{Float64}
    xhat::Vector{Float64}  # state estimate
end

function DelayedMEKF(max_delay_comp=10)
    Wf = Diagonal(fill(1e-4,6))                    # [r,ϕ]
    Vf = Diagonal([fill(1e-4, 9); fill(1e-6, 6)])  # [r,ϕ,v,vb,ωb]
    imuhist = CircularBuffer{Vector{Float64}}(max_delay_comp)
    xf = [zeros(3); 1.0; zeros(12)]                # [r,q,v,ab,ωb]
    Pf = Matrix(1.0I,15,15) 
    xd = copy(xf)
    Pd = copy(Pf)
    xhat = [zeros(3); 1.0; zeros(9)]               # [r,q,v,ω]
    DelayedMEKF(Wf, Vf, 0, imuhist, xf, Pf, xd, Pd, xhat)
end

function initialize!(filter::DelayedMEKF, x0;
        Pf0=Matrix(I,15,15),                            # initial filter covariance
        b0=zeros(6),                                    # initial guess for bias
        Wf=0.0001*I(6),                                 # mocap (measurement) covariance
        Vf=Diagonal([fill(0.0001, 9); fill(1e-6, 6)]),  # imu (process) covariance
        delay_comp::Int=0,
    )
    filter.Wf .= Wf
    filter.Vf .= Vf
    filter.delay_comp = delay_comp
    xf0 = [x0[1:10]; b0]  # initial filter state
    empty!(filter.imuhist)
    filter.xf .= xf0
    filter.Pf .= Pf0
    filter.xd .= xf0
    filter.Pd .= Pf0
    filter.xhat .= x0
end

function get_state_estimate!(filter::DelayedMEKF, y_imu, y_mocap, dt)
    xd = filter.xd
    Pd = filter.Pd
    xf = filter.xf
    Pf = filter.Pf
    xhat = filter.xhat

    # Cache the IMU data
    # NOTE: the data type automatically truncates the vector to keep it at a given length
    push!(filter.imuhist, y_imu)
    delay = min(filter.delay_comp, length(filter.imuhist)-1)

    # Use mocap measurement to update the delayed filter state
    if !isnothing(y_mocap)
        # Advance the delayed measurement using the IMU measurement from that time
        y_imu_delayed = filter.imuhist[end-delay]
        xpred, Ppred = state_prediction(filter, xd, y_imu_delayed, Pd, dt) 

        # Update the delayed filter estimate using the mocap measurement
        xd_, Pd_ = measurement_update(filter, xpred, Ppred, y_mocap)
        xd .= xd_
        Pd .= Pd_
    end

    # Use history of IMU data to predict the state at the current time
    xf .= xd
    Pf .= Pd
    for i = 1:delay-1
        y_imu_delayed = filter.imuhist[end-delay+i]
        xf_,Pf_ = state_prediction(filter, xf, y_imu_delayed, Pf, dt)
        xf .= xf_
        Pf .= Pf_
    end

    # Create state estimate from filter state
    y_gyro = y_imu[4:6]      # current angular velocity recorded by IMU
    b_gyro = xf[14:16]       # predicted gyro bias
    ωhat = y_gyro - b_gyro   # predicted angular velocity
    xhat[1:10] .= xf[1:10]   # copy position, attitude, and linear velocity from filter state
    xhat[11:13] .= ωhat      # copy angular velocity
    return xhat
end

function state_prediction(filter::DelayedMEKF, xf, uf, Pf, h)
    Vf = filter.Vf

    rf = xf[1:3]    # inertial frame
    qf = xf[4:7]    # body to inertial
    vf = xf[8:10]   # body frame
    ab = xf[11:13]  # accel bias
    ωb = xf[14:16]  # gyro bias
    
    af = uf[1:3]    # body frame acceleration (from IMU)
    ωf = uf[4:6]    # body frame linear velocity (from IMU)
    
    ahat = af - ab  # predicted acceleration (with bias removed)
    ωhat = ωf - ωb  # predicted angular velocity (with bias removed)

    # Qf = QuatRotation(qf)
    Qf = quat2rotmat(qf)
    g = SA[0,0,9.81]

    H = hmat()
    L = lmat
    R = rmat
    G(q) = lmat(q) * H

    phi1 = -0.5*h*ωhat
    phi2 = +0.5*h*ωhat

    # IMU Prediction
    y = cay(phi1)             # rotation from this time step to the next
    Y = quat2rotmat(y)
    rp = rf + h * Qf * vf                # position prediction
    qp = lmat(qf) * cay(phi2)             # attitude prediction
    vpk = vf + h * (ahat - Qf'g)         # velocity in old body frame
    vp = Y * vpk                         # velocity in new body frame
    xp = [rp; qp; vp; ab; ωb]

    # Jacobian

    # Derivative of Q(q)*v wrt q
    dvdq = drotate(qf, vf) * G(qf)

    # Derivative of Q(q)*g wrt g
    dgdq = drotate(qinv(qf), g) * G(qf)

    # Derivative of vp wrt ωb
    dvdb = 0.5*h*drotate(y, vpk) * dcay(-0.5 * h * ωhat)

    # Derivative of qp wrt ωb
    dqdb = -0.5*h*G(qp)'L(qf) * dcay(0.5 * h * ωhat)

    # Jacobian of prediction (xp wrt xf)
    #    r           q          v           ab          ωb
    Af = [
        I(3)       h*dvdq     h*Matrix(Qf) zeros(3,3) zeros(3,3)
        zeros(3,3) Y          zeros(3,3)   zeros(3,3) dqdb
        zeros(3,3) -h*Y*dgdq  Y            -h*Y       dvdb
        zeros(6,3) zeros(6,3) zeros(6,3)            I(6)
    ]
    Pp = Af*Pf*Af' + Vf

    xp, Pp
end

function measurement_update(filter::DelayedMEKF, xf, Pf, y_mocap)
    Wf = filter.Wf
    Cf = Matrix(I,6,15)   # measurement Jacobian

    rf = xf[1:3]    # inertial frame
    qf = xf[4:7]    # body to inertial
    vf = xf[8:10]   # body frame
    ab = xf[11:13]  # acceleration bias
    ωb = xf[14:16]  # IMU bias

    rm = y_mocap[1:3]
    qm = y_mocap[4:7]

    z = [rm-rf; icay(lmat(qf)'qm)]  # innovation
    S = Cf * Pf * Cf' + Wf
    Lf = (Pf*Cf')/S  # Kalman filter gain
    # S \ Cf*Pf'
    Δx = Lf * z
    xn = [
        rf + Δx[1:3]; 
        lmat(qf)*cay(Δx[4:6]);
        vf + Δx[7:9]; 
        ab + Δx[10:12]; 
        ωb + Δx[13:15]
    ]
    Pn = (I-Lf*Cf)*Pf*(I-Lf*Cf)' + Lf * Wf * Lf'
    return xn,Pn
end