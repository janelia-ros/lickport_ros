# Copyright (c) 2020, Howard Hughes Medical Institute
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

from smart_cage_msgs.msg import LickportState, LickportControl

from .lickport import Lickport, LickportInfo

import time
import datetime
import math

class LickportNode(Node):
    def __init__(self):
        super().__init__('lickport')

        self.lickport_info = LickportInfo()
        self.name = 'lickport'
        self.logger = self.get_logger()

        self._attached_timer_period = 1
        self._attached_timer = None

        self.lickport = Lickport(self.name, self.logger, self.lickport_info)
        self.lickport.set_on_attach_handler(self._on_attach_handler)
        self.logger.info('opening lickport phidgets...')
        self.lickport.open()

        self._lickport_state_publisher = self.create_publisher(LickportState, 'lickport_state')
        self._lickport_control_subscription = self.create_subscription(
            LickportControl,
            'lickport_control',
            self._lickport_control_callback)
        self._lickport_control_subscription  # prevent unused variable warning

    def _on_attach_handler(self, handle):
        self.lickport._on_attach_handler(handle)
        if self._attached_timer is None:
            self._attached_timer = self.create_timer(self._attached_timer_period, self._attached_timer_callback)

    def _attached_timer_callback(self):
        self._attached_timer.cancel()
        self.destroy_timer(self._attached_timer)
        self._attached_timer = None
        if self.lickport.is_attached():
            self.logger.info('lickport is attached!')
            self.lickport.set_stepper_on_change_handlers_to_disabled()
            self.lickport.set_stepper_on_homed_handlers(self._homed_handler)
            self.lickport.home_stepper_joints()

    def _publish_lickport_state_handler(self, handle, value):
        if not self.lickport.all_stepper_joints_homed:
            return
        lickport_state = LickportState()
        lickport_state.datetime = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        now_frac, now_whole = math.modf(time.time())
        lickport_state.nanosec = int(now_frac * 1e9)
        lickport_state.x = self.lickport.stepper_joints['x'].stepper.get_position()
        lickport_state.y = self.lickport.stepper_joints['y'].stepper.get_position()
        lickport_state.z = self.lickport.stepper_joints['z'].stepper.get_position()
        self._lickport_state_publisher.publish(lickport_state)

    def _homed_handler(self, handle):
        for name, stepper_joint in self.lickport.stepper_joints.items():
            if not stepper_joint.homed:
                return
        self.lickport.set_stepper_on_change_handlers(self._publish_lickport_state_handler)
        self.logger.info('lickport is homed!')

    def _lickport_control_callback(self, msg):
        self.lickport.stepper_joints['x'].stepper.set_target_position(msg.x)
        self.lickport.stepper_joints['y'].stepper.set_target_position(msg.y)
        self.lickport.stepper_joints['z'].stepper.set_target_position(msg.z)

def main(args=None):
    rclpy.init(args=args)

    lickport_node = LickportNode()

    try:
        rclpy.spin(lickport_node)
    except KeyboardInterrupt:
        pass

    lickport_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
