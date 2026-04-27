#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int64
import threading
import curses

stdscr = curses.initscr()

# Throttle should be bounded between [-20, +20]
MAX_MANUAL_THROTTLE_FORWARD = 23
MAX_MANUAL_THROTTLE_REVERSE = 57

# Steering should be bounded between [-100, +100]

steering = 0
throttle = 0

def joy_callback(data):
	global steering, throttle

	'''print("\r\n")
	print(data.axes[1])	# Throttle

	print("\r\n")
	print(data.axes[2]) # Steerting'''
	## ------------
	## YOUR CODE
	## ------------
	steering = data.axes[2] * -100
	throttle = data.axes[1] * 100
	if throttle > MAX_MANUAL_THROTTLE_FORWARD:
		throttle = MAX_MANUAL_THROTTLE_FORWARD
	elif throttle < -MAX_MANUAL_THROTTLE_REVERSE:
		throttle = -MAX_MANUAL_THROTTLE_REVERSE
	#if(data.axes[1] < 0):
	#	throttle = data.axes[1] * MAX_MANUAL_THROTTLE_REVERSE
	#else:
	#	throttle = data.axes[1] * MAX_MANUAL_THROTTLE_FORWARD
	

def main(args=None):

	rclpy.init(args=args)
	node = Node("xbox_controller_node")

	## ------------
	## YOUR CODE
	# Subscribe to the 'joy' topic
	# Create publishers for the manual_throttle and manual_steering commands
	## ------------
	joySubscriber = node.create_subscription(Joy, "/joy", joy_callback, 10)

	throttlePublisher = node.create_publisher(Int64, "/manual_throttle", 10)
	steeringPublisher = node.create_publisher(Int64, "/manual_steering", 10)

	thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
	thread.start()

	rate = node.create_rate(20, node.get_clock())
        
	while rclpy.ok():

		try:
			## ------------
			## YOUR CODE
			# Publish actuation commands
			## ------------
			throttlePublisher.publish(Int64(data=int(throttle)))
			steeringPublisher.publish(Int64(data=int(steering)))

			stdscr.refresh()
			stdscr.addstr(1, 25, 'Xbox Controller       ')
			stdscr.addstr(2, 25, 'Throttle: %.2f  ' % throttle)
			stdscr.addstr(3, 25, 'Steering: %.2f  ' % steering)

			rate.sleep()
		except KeyboardInterrupt:
			curses.endwin()
			print("Ctrl+C captured, ending...")
			break
	
	rclpy.shutdown()

if __name__ == '__main__':
	main()