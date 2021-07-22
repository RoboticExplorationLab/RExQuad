#!/usr/local/bin/python3

#Required input: ip address, port number, message name, message file path
from docopt import docopt
import zmq
import os
import subprocess
import time

sub_ip = "127.0.0.1"
sub_port = "5004"
message_name = "messaging.FILTERED_STATE"
message_file_path = "proto/filtered_state_msg.proto"


def create_sub(ctx, sub_ip, sub_port):
    socket = ctx.socket(zmq.SUB)
    socket.connect("tcp://%s:%s" % (sub_ip, sub_port))
    socket.subscribe("")
    return socket


ctx = zmq.Context()
sub = create_sub(ctx, sub_ip, sub_port)

# Open up a directory 
dir_path = os.path.dirname(os.path.realpath(__file__))
tmp_filename = os.path.join(dir_path, 'tmp.bin')

try:
    data = sub.recv()
except KeyboardInterrupt:
    print("Didn't hear anything. Is anything being published to \"tcp://{}}:{}\"".format(sub_ip, sub_port))

with open(tmp_filename, 'wb') as f:
    f.write(data)

while True:
    if (os.path.isfile(tmp_filename)):
        break

time.sleep(1)
os.popen('protoc --decode_raw < {}'.format(tmp_filename))

# output = subprocess.check_output(["protoc", "--decode", message_name, message_file_path, "<", tmp_filename], shell=True)
# (out, err) = proc.communicate()
# print("program output:", out)
# print(output)
# os.popen('protoc --decode_raw < {}'.format(tmp_filename))



try:
    # os.remove(tmp_filename)
    pass
except OSError:
    pass

# 
# Be sure to close the socket 
sub.close()
