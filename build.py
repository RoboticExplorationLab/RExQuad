import argparse
import os
import string

def arduino_cli(scriptfile: str, fqbn: str, action="compile", verbose=False, port="tty/ACM0"):
    if action == "compile" or action == "all":
        cmd = "arduino-cli compile --warnings all --fqbn {} --libraries {} --library {} --build-path {} --build-cache-path {} {}".format(
            fqbn,
            arduino_libs_dir,
            common_lib_dir,
            bin_dir,
            cache_dir,
            scriptfile
        )
        if verbose:
            print(cmd, "\n")
        os.system(cmd)
    if action == "upload" or action == "all":
        cmd = "arduino-cli upload --fqbn {} --port {} --input-dir {} {}".format(
            fqbn,
            port,
            bin_dir,
            scriptfile
        )
        if verbose:
            print(cmd, "\n")
        os.system(cmd)

def build_feather(script, action, **kwargs):
    dir = os.path.join(rootdir, "src", "Arduino", script)
    scriptfile = os.path.join(dir, script + ".ino")
    boardname = "adafruit:samd:adafruit_feather_m0"
    arduino_cli(scriptfile, boardname, action, **kwargs)

# Parse arguments
targets = [
    # "mocap",
    "default",
    "lora_relay",
    "lora_tx",
    "lora_rx",
    "lora_tx_latency",
    "lora_tx_serial",
    "lora_rx_serial",
    "feather_parrot",
]
parser = argparse.ArgumentParser()
parser.add_argument("target",
                    nargs="?",
                    help="The target to build.",
                    type=str,
                    default="default",
                    choices=targets
                    )
parser.add_argument("action",
                    nargs="?",
                    help="Action to perform",
                    type=str,
                    default="all",
                    choices=["compile", "upload", "all"]
                    )
parser.add_argument("-v", "--verbose", action="store_true")
parser.add_argument("-p", "--port",
                    help="Port of the board, e.g. /dev/ttyACM0",
                    default="/dev/ttyACM0"
                    )
args = parser.parse_args()

# Set directories
rootdir = os.path.dirname(os.path.realpath(__file__))
arduino_libs_dir = os.path.join(rootdir, "src", "Arduino", "libraries")
common_lib_dir = os.path.join(rootdir, "src", "common")
bin_dir = os.path.join(rootdir, "bin", args.target)
cache_dir = os.path.join(bin_dir, "cache")

# Call the right build function
if args.target == "default":
    build_feather("lora_tx", args.action, verbose=args.verbose, port="/dev/ttyACM0")
    build_feather("lora_rx", args.action, verbose=args.verbose, port="/dev/ttyACM1")
else:
    build_feather(args.target, args.action, verbose=args.verbose, port=args.port)
    
