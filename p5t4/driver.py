#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
import struct
import can
import threading
import time

throttle = 0
steer = 0

## ------------
## YOUR CODE
## ------------
def throttle_callback(msg):
    global throttle

    throttle = msg.data

def steering_callback(msg):
    global steer 

    steer = msg.data


def main(args=None):
    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate= 250000)
    
    rclpy.init(args=args)
    node = Node("driver")

    ## ------------
    ## YOUR CODE
    # Subscribe to the manual_throttle and manual_steering commands
    ## ------------
    throttleSubscriber = node.create_subscription(Int64, "/manual_throttle", throttle_callback, 10)
    steeringSubscriber = node.create_subscription(Int64, "/manual_steering", steering_callback, 10)

    thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    thread.start()
 
    rate = node.create_rate(20, node.get_clock())
    while rclpy.ok():

        print('throttle: %d, steering: %d' % (throttle, steer))
        try:
            # DO NOT COMPUTE THE PWM VALUES IN ORIN. Just send the raw command values. 
            can_data = struct.pack('>hhI', throttle, steer, 0)

            ## ------------
            ## YOUR CODE
            # Create a CAN message with can_data and  arbitration_id 0x1, and send it.
            # Hint: See Lecture 4 slides
            ## ------------
            message = can.Message(arbitration_id=0x1, data=can_data, is_extended_id=False)
            ret = bus.send(message)

        except Exception as error:
            print("An exception occurred:", error)
        finally:
            rate.sleep()

    rclpy.spin(node)
    rclpy.shutdown()

	
if __name__ == '__main__':
	main()