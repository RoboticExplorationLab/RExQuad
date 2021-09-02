# MocapTest
To build all relevant protocol buffers run the `build.sh` file from the projects root directory.

## Reposititory Structure
### `Arduino` Directory
Set the Arduino app's `Sketchbook Location` to the `MocapTest/Arduino` directory.

### `launch` Directory
Contains scripts to launch multiple nodes simultaneously.

### `msgs` Directory
Contains all built protocol buffer message scripts.

### `nodes` Directory
Contains the nodes running filters, controlers and various publishers/subscribers.

! All relevant ZMQ and Serial ports should be specified in the `TOML` file `nodes/setup.toml`.

### `proto` Directory
Contains all protocol buffer structures (`.proto` files)