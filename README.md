Install Teensy Loader
Install Arduino CLI

# Install submodules
If you didn't clone recursively, update the submodules with
```
git submodule init
git submodule update
```

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

## Add yourself to the dialout and tty groups

```
sudo usermod -a -G tty <username>
sudo usermod -a -G dialout <username>
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

Note you may need to install `autoreconf`:
```
sudo apt-get install dh-autoreconf
```

# Accelerometer Wiring (SPI)
Accel -> Feather
SCL -> SCK (15)
DO -> MISO (14)
SDA -> MOSI (16)
CS -> Any GPIO (11)

