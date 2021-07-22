import sys 
sys.path.append("..")
from pymessaging import messaging
import pymessaging.message_pb2 as msg
import zmq 
import time 
import random

# Creating a publisher
ctx = zmq.Context() 
pub = messaging.create_pub(ctx, "5001")

# Creating a message 
imu = msg.IMU_msg()
imu.gyroscope.x = 1.0
# imu.acceleration.x = 1.0
while True:
    imu.gyroscope.x = random.normalvariate(0,1)
    print('sending data')
    imu.time = time.time()
    data = imu.SerializeToString()
    pub.send(data) 
    time.sleep(0.01)