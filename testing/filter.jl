using StaticArrays
using LinearAlgebra: I
using ErrorStateEKF
using DataFrames: DataFrame, sort!, outerjoin
using CSV

include("$(@__DIR__)/imu_dynamics.jl")
include("$(@__DIR__)/buffer.jl")

imu_df = DataFrame(CSV.File("$(@__DIR__)/../data/imu_first_success.csv"))
vicon_df = DataFrame(CSV.File("$(@__DIR__)/../data/vicon_first_success.csv"))
total_df = outerjoin(imu_df, vicon_df, on=:time)
sort!(total_df, :time)

# Load the IMU's brownian motion std and mean
μ = Matrix(DataFrame(CSV.File("$(@__DIR__)/../data/imu_sys_id/data/mean.csv"; header=false)));
Σ = Matrix(DataFrame(CSV.File("$(@__DIR__)/../data/imu_sys_id/data/covariance.csv"; header=false)));
  
nₑ = length(State) - 1
mₑ = length(Measurement) - 1
process_cov = MMatrix{nₑ,nₑ, Float64}(I(nₑ) * .1);
measure_cov = MMatrix{mₑ,mₑ, Float64}(I(mₑ) * .1);

input_buffer = Buffer{Input}(; size=100)
measurement_buffer = Buffer{Measurement}(; size=100)

ekf = ErrorStateFilter(process_cov, measure_cov,
                       process, measure,
                       process_jacobian, measure_jacobian,
                       error_state_jacobian, error_measurement_jacobian)
# ekf.est_state .= State(SA[rand(3)..., params(rand(UnitQuaternion))..., rand(9)...]);
ekf.est_state .= State(SA[0.,0,0, 1,0,0,0, 0,0,0, 0,0,0, 0,0,0]);
ekf.est_cov .= I(nₑ) * 1.0;

# %%

lastTime = total_df[1, :time]

for row in eachrow(total_df)
    time = row[:time]
    input = Input(row[[:acc_x, :acc_y, :acc_z, :gyr_x, :gyr_y, :gyr_z]]...)
    measurement = Measurement(row[[:pos_x, :pos_y, :pos_z, :quat_w, :quat_x, :quat_y, :quat_z]]...)

    if !ismissing(sum(input))
        push!(input_buffer, input, time)
    end
    if !ismissing(sum(measurement))
        push!(measurement_buffer, measurement, time)
    end

    if length(input_buffer) > 0 && length(measurement_buffer) > 0
        input = nab!(input_buffer, time)
        measurement = nab!(measurement_buffer, time)
        dt = time - lastTime
        time = lastTime

        println(dt)
        p, q, v, α, _ = getComponents(ekf.est_state)
        println("State, p: ", p)
        println("State, v: ", v)
        # println("State, q*v: ", q*v)
        println("State, v̇: ", getComponents(input)[1])
        println("State, α: ", α)
        # println("State, p̂: ", getComponents(measurement)[1])
        println("State, p̃: ", getComponents(ekf.process(ekf.est_state, input, dt))[1])
        println("State, ṽ: ", getComponents(ekf.process(ekf.est_state, input, dt))[3])
        println()

        estimateState!(ekf, input, measurement, dt)
    end
end

