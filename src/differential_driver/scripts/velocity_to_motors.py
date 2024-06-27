#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class Velocity_To_Motors():

    def __init__(self):
        rospy.init_node('Vel_To_Motors')
        self.subscribe = rospy.Subscriber('/diffbot_controller/cmd_vel', Twist, self.vel_callback, queue_size=10)
        self.publish_ = rospy.Publisher('wheel_velocities', String, queue_size=10)

        self.wheel_base = 0.68        # in meters
        self.wheel_radius = 0.12      # in meters
        self.max_velocity = 1.256        # in m/s

    def vel_callback(self,msg):
        
        RPM = String()

        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Velocity_Calculation for Individual Wheel

        left_wheel_velocity = linear_vel + angular_vel * (self.wheel_base / 2.0)
        right_wheel_velocity = linear_vel - angular_vel * (self.wheel_base / 2.0)

        # RPM Calculation for Individual Wheel

        left_wheel_rpm = ( left_wheel_velocity / self.max_velocity ) * 100
        right_wheel_rpm = ( right_wheel_velocity / self.max_velocity ) * 100

        # PWM for both Wheels

        pwm =  str(left_wheel_rpm) + "," + str(right_wheel_rpm)
        RPM.data = pwm

        # rospy.loginfo('PWM data is Published')

        # PWM data is Published

        self.publish_.publish(RPM)


def main(args=None):
    rospy.init_node('Vel_To_Motors')
    Vel_To_Motors = Velocity_To_Motors()
    rospy.spin()


if __name__ == '__main__':
    main()


        
        


        
