#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from lidar.cfg import LidarTiltConfig

up = 0
down = 0
speed = 0
enable = True
rpi = False

def map(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

def map_angle(angle):
	return map(angle, 0, 180, 1000, 2000)

def callback(config, level):
	up = config.up_tilt_angle
	down = config.down_tilt_angle
	speed = config.tilt_speed
	enable = config.tilt_enable

	if down < up:
		config.up_tilt_angle = down
	return config

def oscillate(event):
	global rpi
	pi = None
	try:
		import pigpio
		rpi = True
		pi = pigpio.pi()
	except ImportError as e:
		rospy.logerr("Could not load the pigpio module, will not oscillate")

	while not rospy.is_shutdown():
		time = rospy.Duration(0, 100)
		if (rpi and enable) :
			
			for x in range(up, down):
				pi.set_servo_pulsewidth(17, map_angle(x))
				rospy.sleep(time)

			for x in range(down, up):
				pi.set_servo_pulsewidth(17, map_angle(x))
				rospy.sleep(time)


if __name__ == "__main__":
    rospy.init_node("dynamic_tutorials", anonymous = True)
    srv = Server(LidarTiltConfig, callback)

    rospy.Timer(rospy.Duration(1), oscillate, oneshot=True)
    rospy.spin()

