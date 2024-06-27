#!/usr/bin/env python3

# Rhino Motor Driver (RMCS 2303) - ROS INTERFACE
# ----------------------------------------------
# Note: This node connects the motor driver with the differential_drive ROS package mentioned below:
# https://github.com/jfstepha/differential-drive

import sys
import time
import rospy
from std_msgs.msg import Int16, Int32, UInt16, UInt32, Float32
import motor_driver


class MotorClass:
    def __init__(self, port_name, slave_id, encoder_topic_name, velocity_topic_name):
        self.motor = motor_driver.Controller(port_name, slave_id)
        self.__time_delay = 0.001
        self.encoder_ticks_16bit = self.motor.get_position_16bit()
        self.encoder_ticks_32bit = self.motor.get_position_32bit()
        self.encoder_ticks_32bit_scaled = 0
        self.__previous_velocity = 0
        self.__current_velocity = 0
        self.__status_rotation_direction = ""
        self.__velocity_scale_factor = 19
        self.__encoder_scale_factor = 0.5
        self.encoder_ticks_pub_32bit = rospy.Publisher(encoder_topic_name + "_32bit", UInt32, queue_size=1)
        rospy.Subscriber(velocity_topic_name, Float32, self.velocity_callback, queue_size=1)

    def velocity_callback(self, data):
        motor_velocity_radians_per_second = data.data
        scaled_velocity = self.__velocity_scale_factor * motor_velocity_radians_per_second
        self.__current_velocity = scaled_velocity
        self.motor.set_speed(scaled_velocity)

        if self.__current_velocity != self.__previous_velocity:
            if motor_velocity_radians_per_second < 0:
                self.motor.turn_motor_cw()
                self.__status_rotation_direction = "CW"
            elif motor_velocity_radians_per_second > 0:
                self.motor.turn_motor_ccw()
                self.__status_rotation_direction = "CCW"
            else:
                self.motor.brake()
                self.__status_rotation_direction = ""
            self.__previous_velocity = self.__current_velocity

    def encoder_transmitter(self):
        self.encoder_ticks_32bit = self.motor.get_position_32bit()
        if self.encoder_ticks_32bit is not None:
            self.encoder_ticks_pub_32bit.publish(int(self.encoder_ticks_32bit))


if __name__ == '__main__':
    rospy.init_node('motor_ros_interface', anonymous=True)
    
    left_motors = [
        MotorClass("/dev/left_wheel_1", 7, "lwheel1_ticks", "lwheel1_vtarget"),
        MotorClass("/dev/left_wheel_2", 8, "lwheel2_ticks", "lwheel2_vtarget"),
        MotorClass("/dev/left_wheel_3", 9, "lwheel3_ticks", "lwheel3_vtarget")
    ]
    
    right_motors = [
        MotorClass("/dev/right_wheel_1", 10, "rwheel1_ticks", "rwheel1_vtarget"),
        MotorClass("/dev/right_wheel_2", 11, "rwheel2_ticks", "rwheel2_vtarget"),
        MotorClass("/dev/right_wheel_3", 12, "rwheel3_ticks", "rwheel3_vtarget")
    ]

    while not rospy.is_shutdown():
        for motor in left_motors + right_motors:
            motor.encoder_transmitter()

    # Cleaning Up
    rospy.loginfo("Stopping Motors before exiting ...")
    for motor in left_motors + right_motors:
        motor.motor.brake()
        del motor
    sys.exit(0)
