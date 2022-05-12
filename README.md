Install Teensy Loader
Install Arduino CLI

```
sudo cp 00-teensy.rules /etc/udev/rules.d/
arduino-cli core install teensy:avr
arduino-cli core install adafruit:samd
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