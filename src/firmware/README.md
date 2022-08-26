# Firmware

This folder contains the Arduino scripts for the quadrotor. Some are for the onboard computer while others are for the base station. The onboard computer 
is typically designated as the receiver while the base station is designated as the transmitter. Currently, the most important scripts here are:
- `onboard` holds the code that runs on the onboard Feather. Receives pose information over radio, reads accelerometer data, sends commands to the motors,
and sends communications back to the base station via a secondard LoRa radio.
- `feather_tx` code for the base station Feather radio that sends pose information to the quadrotor.
- `motor_control` useful for calibrating the motors and sending them individually

All of the Arduino libraries used by these scripts are located in the `libraries` directory. Additionally, these scripts include and compile against the 
code in `src/common`.


# Main Scripts
basestation/
