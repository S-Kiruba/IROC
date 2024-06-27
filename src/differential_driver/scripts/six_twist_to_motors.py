#!/usr/bin/env python3
"""
   twist_to_motors - converts a twist message to motor commands. Needed for navigation stack
   
    Copyright (C) 2012 Jon Stephan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""
import time

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist 

class TwistToMotors():
    def __init__(self):
        rospy.init_node("twist_to_motors")
        nodename = rospy.get_name()
        rospy.loginfo("%s started" % nodename)
    
        self.w = rospy.get_param("~base_width", 0.235)

        # Publishers for each wheel
        self.pub_lwheel1 = rospy.Publisher('lwheel1_vtarget', Float32, queue_size=1)
        self.pub_lwheel2 = rospy.Publisher('lwheel2_vtarget', Float32, queue_size=1)
        self.pub_lwheel3 = rospy.Publisher('lwheel3_vtarget', Float32, queue_size=1)
        self.pub_rwheel1 = rospy.Publisher('rwheel1_vtarget', Float32, queue_size=1)
        self.pub_rwheel2 = rospy.Publisher('rwheel2_vtarget', Float32, queue_size=1)
        self.pub_rwheel3 = rospy.Publisher('rwheel3_vtarget', Float32, queue_size=1)

        rospy.Subscriber('/diffbot_controller/cmd_vel', Twist, self.twistCallback) 

        self.rate = rospy.get_param("~rate", 5)
        self.timeout_ticks = rospy.get_param("~timeout_ticks", 2)
        self.left = 0
        self.right = 0
        self.dx = 0
        self.dr = 0
        self.ticks_since_target = self.timeout_ticks

    def spin(self):
        r = rospy.Rate(self.rate)
        idle = rospy.Rate(5)
    
        while not rospy.is_shutdown():
            while not rospy.is_shutdown() and self.ticks_since_target < self.timeout_ticks:
                self.spinOnce()
                r.sleep()
            idle.sleep()

    def spinOnce(self):
        self.right = 1.0 * self.dx - self.dr * self.w / 2
        self.left = -1.0 * self.dx - self.dr * self.w / 2

        # Publish to all left wheels
        self.pub_lwheel1.publish(self.left)
        self.pub_lwheel2.publish(self.left)
        self.pub_lwheel3.publish(self.left)

        # Publish to all right wheels
        self.pub_rwheel1.publish(self.right)
        self.pub_rwheel2.publish(self.right)
        self.pub_rwheel3.publish(self.right)
            
        self.ticks_since_target += 1

    def twistCallback(self, msg):
        self.ticks_since_target = 0
        self.dx = msg.linear.x
        self.dr = msg.angular.z
        self.dy = msg.linear.y

if __name__ == '__main__':
    try:
        twistToMotors = TwistToMotors()
        twistToMotors.spin()
    except rospy.ROSInterruptException:
        pass
