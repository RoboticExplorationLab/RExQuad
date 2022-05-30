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

def build_arduino(script, action, board, **kwargs):
    dir = os.path.join(rootdir, "src", "firmware", script)
    scriptfile = os.path.join(dir, script + ".ino")
    if board == "m0":
        boardname = "adafruit:samd:adafruit_feather_m0"
    elif board == "32u4":
        boardname = "adafruit:avr:feather32u4"
    elif board == "teensy":
        boardname = "teensy:avr:teensy40"
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
    "motor_control",
    "onboard",
    "lora_tx_test",
    "lora_rx_test",
    "latency_test",
    "blink",
    "feather_tx",
    "feather_rx",
    "motor_tests",
    "heartbeat_test",
    "wifi_test",
    "radio_lora",
    "lora_bridge",
    "onboard_teensy",
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
parser.add_argument("-b", "--board", 
                    choices=["m0", "32u4", "teensy"],
                    default="m0",
                    type=str,
)
args = parser.parse_args()

# Set directories
rootdir = os.path.dirname(os.path.realpath(__file__))
arduino_libs_dir = os.path.join(rootdir, "src", "firmware", "libraries")
common_lib_dir = os.path.join(rootdir, "src", "common")
bin_dir = os.path.join(rootdir, "bin", args.target)
cache_dir = os.path.join(bin_dir, "cache")

# Call the right build function
if args.target == "default":
    build_arduino("lora_tx_test", args.action, board=args.board, verbose=args.verbose, port="/dev/ttyACM0")
    build_arduino("lora_rx_test", args.action, board=args.board, verbose=args.verbose, port="/dev/ttyACM1")
else:
    build_arduino(args.target, args.action, board=args.board, verbose=args.verbose, port=args.port)
     
