# Copyright (c) 2019, Howard Hughes Medical Institute
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the copyright holder nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from joint_control_msgs.msg import JointTarget

from .lickport import Lickport, LickportInfo

from time import time
import math

class LickportNode(Node):
    def __init__(self):
        super().__init__('lickport')

        self.lickport_info = LickportInfo()
        self.name = 'lickport'
        self.logger = self.get_logger()

        self._joint_state_publisher = self.create_publisher(JointState, 'lickport_joint_state', 10)
        self._joint_target_subscription = self.create_subscription(
            JointTarget,
            'lickport_joint_target',
            self._joint_target_callback,
            10)
        self._joint_target_subscription  # prevent unused variable warning

        self._attached_timer_period = 1
        self._attached_timer = None

        self.lickport = Lickport(self.lickport_info, self.name, self.logger)
        self.lickport.set_stepper_on_change_handlers_to_disabled()
        self.lickport.set_stepper_on_stopped_handlers(self._homed_handler)
        self.lickport.set_on_attach_handler(self._on_attach_handler)
        self.logger.info('opening lickport phidgets...')
        self.lickport.open()

    def _on_attach_handler(self, handle):
        self.lickport._on_attach_handler(handle)
        if self._attached_timer is None:
            self._attached_timer = self.create_timer(self._attached_timer_period, self._attached_timer_callback)

    def _attached_timer_callback(self):
        self._attached_timer.cancel()
        self._attached_timer = None
        if self.lickport.is_attached():
            self.logger.info('lickport is attached!')
            self.lickport.home_stepper_joints()

    def _publish_joint_state_handler(self, handle, value):
        if not self.lickport.all_stepper_joints_homed:
            return
        joint_state = JointState()
        joint_state.header = Header()
        now_frac, now_whole = math.modf(time())
        joint_state.header.stamp.sec = int(now_whole)
        joint_state.header.stamp.nanosec = int(now_frac * 1e9)
        for name, stepper_joint in self.lickport.stepper_joints.items():
            joint_state.name.append(name)
            joint_state.position.append(stepper_joint.stepper.get_position())
            joint_state.velocity.append(stepper_joint.stepper.get_velocity())
        self._joint_state_publisher.publish(joint_state)

    def _homed_handler(self, handle):
        for name, stepper_joint in self.lickport.stepper_joints.items():
            if not stepper_joint.homed:
                return
        self.lickport.set_stepper_on_change_handlers(self._publish_joint_state_handler)
        self.lickport.set_stepper_on_stopped_handlers_to_disabled()
        self.logger.info('lickport is homed!')

    def _joint_target_callback(self, msg):
        if len(msg.name) == len(msg.velocity) == len(msg.position):
            targets = zip(msg.name, msg.velocity, msg.position)
            for name, velocity, position in targets:
                try:
                    self.lickport.stepper_joints[name].stepper.set_velocity_limit(velocity)
                    self.lickport.stepper_joints[name].stepper.set_target_position(position)
                except KeyError:
                    pass
        elif len(msg.name) == len(msg.position):
            targets = zip(msg.name, msg.position)
            for name, position in targets:
                try:
                    self.lickport.stepper_joints[name].stepper.set_target_position(position)
                except KeyError:
                    pass

def main(args=None):
    rclpy.init(args=args)

    lickport_node = LickportNode()

    rclpy.spin(lickport_node)

    lickport_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
