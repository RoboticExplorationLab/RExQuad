add_library(rexquad_arduino
  arduino_receiver.hpp
  arduino_receiver.cpp

  motors.hpp
  motors.cpp

  sensors.hpp
  sensors.cpp

  serial_utils.hpp
  serial_utils.cpp

  constants.hpp
)
set(ARDUINO_DIR "/home/brian/.arduino15")
target_include_directories("${ARDUINO_DIR}/packages/arduino/hardware/samd/1.8.13/cores/arduino")