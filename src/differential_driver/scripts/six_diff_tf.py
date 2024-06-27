#!/usr/bin/env python3

"""
   diff_tf.py - follows the output of a wheel encoder and
   creates tf and odometry messages.
   some code borrowed from the arbotix diff_controller script
   A good reference: http://rossum.sourceforge.net/papers/DiffSteer/

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

   ----------------------------------
   Portions of this code borrowed from the arbotix_python diff_controller.

diff_controller.py - controller for a differential drive
  Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its
        contributors may be used to endorse or promote products derived
        from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
from math import sin, cos
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import UInt32

class DiffTf:
    def __init__(self):
        rospy.init_node("diff_tf")
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)

        # Parameters
        self.rate = rospy.get_param('~rate', 10.0)
        self.ticks_meter = float(rospy.get_param('ticks_meter', 279118))
        self.base_width = float(rospy.get_param('~base_width', 0.235))
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_footprint')
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')
        self.encoder_min = rospy.get_param('encoder_min', 0)
        self.encoder_max = rospy.get_param('encoder_max', 4294967295)

        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min)
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min)

        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = rospy.Time.now() + self.t_delta

        # Internal data
        self.enc_left = [None, None, None]  # Encoders for three left wheels
        self.enc_right = [None, None, None]  # Encoders for three right wheels
        self.left = [0, 0, 0]
        self.right = [0, 0, 0]
        self.lmult = [0, 0, 0]
        self.rmult = [0, 0, 0]
        self.prev_lencoder = [0, 0, 0]
        self.prev_rencoder = [0, 0, 0]
        self.x = 0
        self.y = 0
        self.th = 0
        self.dx = 0
        self.dr = 0
        self.then = rospy.Time.now()

        # Subscriptions
        rospy.Subscriber("lwheel1_ticks_32bit", UInt32, self.lwheel_callback, callback_args=0)
        rospy.Subscriber("lwheel2_ticks_32bit", UInt32, self.lwheel_callback, callback_args=1)
        rospy.Subscriber("lwheel3_ticks_32bit", UInt32, self.lwheel_callback, callback_args=2)
        rospy.Subscriber("rwheel1_ticks_32bit", UInt32, self.rwheel_callback, callback_args=0)
        rospy.Subscriber("rwheel2_ticks_32bit", UInt32, self.rwheel_callback, callback_args=1)
        rospy.Subscriber("rwheel3_ticks_32bit", UInt32, self.rwheel_callback, callback_args=2)
        
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    def update(self):
        now = rospy.Time.now()
        if now > self.t_next:
            elapsed = now - self.then
            self.then = now
            elapsed = elapsed.to_sec()

            if None in self.enc_left or None in self.enc_right:
                d_left = 0
                d_right = 0
                rospy.loginfo("No encoder readings yet")
            else:
                d_left = sum([(self.left[i] - self.enc_left[i]) for i in range(3)]) / (3 * self.ticks_meter)
                d_right = sum([-(self.right[i] - self.enc_right[i]) for i in range(3)]) / (3 * self.ticks_meter)

            self.enc_left = self.left[:]
            self.enc_right = self.right[:]

            d = (d_left + d_right) / 2
            th = -1 * (d_right - d_left) / self.base_width
            self.dx = d / elapsed
            self.dr = th / elapsed

            if d != 0:
                x = cos(th) * d
                y = -sin(th) * d
                self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
                self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
            if th != 0:
                self.th = self.th + th

            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2)
            quaternion.w = cos(self.th / 2)
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                rospy.Time.now(),
                self.base_frame_id,
                self.odom_frame_id
            )

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame_id
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0
            odom.pose.pose.orientation = quaternion
            odom.child_frame_id = self.base_frame_id
            odom.twist.twist.linear.x = self.dx
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = self.dr
            self.odomPub.publish(odom)

    def lwheel_callback(self, msg, index):
        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_lencoder[index] > self.encoder_high_wrap:
            self.lmult[index] += 1
        if enc > self.encoder_high_wrap and self.prev_lencoder[index] < self.encoder_low_wrap:
            self.lmult[index] -= 1
        self.left[index] = enc + self.lmult[index] * (self.encoder_max - self.encoder_min)
        self.prev_lencoder[index] = enc

    def rwheel_callback(self, msg, index):
        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_rencoder[index] > self.encoder_high_wrap:
            self.rmult[index] += 1
        if enc > self.encoder_high_wrap and self.prev_rencoder[index] < self.encoder_low_wrap:
            self.rmult[index] -= 1
        self.right[index] = enc + self.rmult[index] * (self.encoder_max - self.encoder_min)
        self.prev_rencoder[index] = enc

if __name__ == '__main__':
    try:
        diffTf = DiffTf()
        diffTf.spin()
    except rospy.ROSInterruptException:
        pass
