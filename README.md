Install Teensy Loader
Install Arduino CLI

# Install Arduino CLI
cd `~/.local` 
```
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
```
make sure `~/.local` is on your system `PATH`.

# Change install location
Modify `arduino-cli.yaml` to be your home directory (replace `/home/brian/`)

# Install Arduino Library
```
sudo cp 00-teensy.rules /etc/udev/rules.d/
arduino-cli core install teensy:avr
arduino-cli core install adafruit:samd
arduino-cli core install adafruit:avr
arduino-cli core install arduino:avr
arduino-cli core install arcore:avr
arduino-cli lib install "Adafruit LSM6DS"
arduino-cli lib install "Adafruit BusIO"
arduino-cli lib install "Adafruit Unified Sensor"

```

# Deps
9. Clone the libserialport library:
    ```
    git clone git://sigrok.org/libserialport
    ```
10. Install the libserialport library, building from source:
    ```
    cd libserialport
    ./autogen
    ./configure
    make
    sudo make install


# Accelerometer Wiring (SPI)
Accel -> Feather
SCL -> SCK (15)
DO -> MISO (14)
SDA -> MOSI (16)
CS -> Any GPIO (11)

