using ArduinoSerial
using Printf
using Dates

include("$(@__DIR__)/../msgs/Messages.jl")

# %%
HOLYBRO_BAUDRATE = 57600
ard = Arduino("/dev/tty.usbmodem142401", HOLYBRO_BAUDRATE);

msg_list = [];
time_list = [];

# %%
output = 0
msg = 0
open(ard) do sp
    while true
        if bytesavailable(ard) > 0
            global output = recieve(ard)

            # Check that the message read is the same size as a IMU_message type
            if sizeof(output) == sizeof(Messages.IMU_message)
                global msg = reinterpret(Messages.IMU_message, output)[1]
                # @printf("Accel: (%1.3f, %1.3f, %1.3f)\t Gyro: (%1.3f, %1.3f, %1.3f)\t Mag: (%1.3f, %1.3f, %1.3f)\r",
                #         msg.acc_x, msg.acc_y, msg.acc_z, msg.gyr_x, msg.gyr_y,
                #         msg.gyr_z, msg.mag_x, msg.mag_y, msg.mag_z)

                lastTime = now()

                push!(msg_list, msg)
                push!(time_list, lastTime)

                print("time: ", lastTime, ", # saved: ", length(msg_list), "\r")
            end
        end

        if length(msg_list) >= 10000
            break
        end
    end
end

# %%
using DataFrames
using CSV

data_acc_x = [msg_list[i].acc_x for i in 1:length(msg_list)]
data_acc_y = [msg_list[i].acc_y for i in 1:length(msg_list)]
data_acc_z = [msg_list[i].acc_z for i in 1:length(msg_list)]

data_gyr_x = [msg_list[i].gyr_x for i in 1:length(msg_list)]
data_gyr_y = [msg_list[i].gyr_y for i in 1:length(msg_list)]
data_gyr_z = [msg_list[i].gyr_z for i in 1:length(msg_list)]

data_mag_x = [msg_list[i].mag_x for i in 1:length(msg_list)]
data_mag_y = [msg_list[i].mag_y for i in 1:length(msg_list)]
data_mag_z = [msg_list[i].mag_z for i in 1:length(msg_list)]

df = DataFrame(time=time_list,
               acc_x=data_acc_x, acc_y=data_acc_y, acc_z=data_acc_z,
               gyr_x=data_gyr_x, gyr_y=data_gyr_y, gyr_z=data_gyr_z,
               mag_x=data_mag_x, mag_y=data_mag_y, mag_z=data_mag_z);
df

# %%
CSV.write("data/imu_data.csv", df)

# %%
test = CSV.read("data/imu_data.csv", DataFrame)
test
