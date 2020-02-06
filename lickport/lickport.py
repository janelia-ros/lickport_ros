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

from phidgets_python_api.stepper_joint import StepperJoint, StepperJointInfo

class LickportInfo():
    def __init__(self):
        self.stepper_joints_info = {'x': StepperJointInfo(),
                                    'y': StepperJointInfo(),
                                    'z': StepperJointInfo()}

        self.stepper_joints_info['x'].stepper_info.phidget_info.hub_port = 0
        self.stepper_joints_info['x'].stepper_info.phidget_info.label = 'lickport_0'
        self.stepper_joints_info['x'].stepper_info.current_limit = 0.140
        self.stepper_joints_info['x'].home_switch_info.phidget_info.hub_port = 5
        self.stepper_joints_info['x'].home_switch_info.phidget_info.label = 'lickport_0'
        self.stepper_joints_info['x'].home_switch_info.active_low = False

        self.stepper_joints_info['y'].stepper_info.phidget_info.hub_port = 1
        self.stepper_joints_info['y'].stepper_info.phidget_info.label = 'lickport_0'
        self.stepper_joints_info['y'].stepper_info.current_limit = 0.140
        self.stepper_joints_info['y'].home_switch_info.phidget_info.hub_port = 4
        self.stepper_joints_info['y'].home_switch_info.phidget_info.label = 'lickport_0'
        self.stepper_joints_info['y'].home_switch_info.active_low = False

        self.stepper_joints_info['z'].stepper_info.phidget_info.hub_port = 2
        self.stepper_joints_info['z'].stepper_info.phidget_info.label = 'lickport_0'
        self.stepper_joints_info['z'].stepper_info.current_limit = 0.140
        self.stepper_joints_info['z'].home_switch_info.phidget_info.hub_port = 3
        self.stepper_joints_info['z'].home_switch_info.phidget_info.label = 'lickport_0'
        self.stepper_joints_info['z'].home_switch_info.active_low = False

class Lickport():
    def __init__(self, lickport_info, name, logger):
        self.lickport_info = lickport_info
        self.name = name
        self.logger = logger

        self.stepper_joints = {}
        for name, info in self.lickport_info.stepper_joints_info.items():
            self.stepper_joints[name] = StepperJoint(info, self.name + '_' + name + "_stepper_joint", self.logger)

    def open(self):
        for name, stepper_joint in self.stepper_joints.items():
            stepper_joint.open()

    def close(self):
        for name, stepper_joint in self.stepper_joints.items():
            stepper_joint.close()

    def set_on_attach_handler(self, on_attach_handler):
        for name, stepper_joint in self.stepper_joints.items():
            stepper_joint.set_on_attach_handler(on_attach_handler)

    def _on_attach_handler(self, handle):
        for name, stepper_joint in self.stepper_joints.items():
            if stepper_joint.has_handle(handle):
                stepper_joint._on_attach_handler(handle)

    def is_attached(self):
        for name, stepper_joint in self.stepper_joints.items():
            if not stepper_joint.is_attached():
                return False
        return True

    def set_stepper_on_change_handlers(self, stepper_on_change_handler):
        for name, stepper_joint in self.stepper_joints.items():
            stepper_joint.stepper.set_on_position_change_handler(stepper_on_change_handler)
            stepper_joint.stepper.set_on_velocity_change_handler(stepper_on_change_handler)

    def set_stepper_on_change_handlers_to_disabled(self):
        for name, stepper_joint in self.stepper_joints.items():
            stepper_joint.stepper.set_on_position_change_handler(None)
            stepper_joint.stepper.set_on_velocity_change_handler(None)

    def set_stepper_on_stopped_handlers(self, stepper_on_stopped_handler):
        for name, stepper_joint in self.stepper_joints.items():
            stepper_joint.stepper.set_on_stopped_handler(stepper_on_stopped_handler)

    def set_stepper_on_stopped_handlers_to_disabled(self):
        for name, stepper_joint in self.stepper_joints.items():
            stepper_joint.stepper.set_on_stopped_handler(None)

    def set_stepper_on_homed_handlers(self, on_homed_handler):
        for name, stepper_joint in self.stepper_joints.items():
            stepper_joint.set_on_homed_handler(on_homed_handler)

    def home_stepper_joints(self):
        for name, stepper_joint in self.stepper_joints.items():
            stepper_joint.home()

    def all_stepper_joints_homed(self):
        all_homed = True
        for name, stepper_joint in self.stepper_joints.items():
            if not stepper_joint.homed:
                all_homed = False
                break
        return all_homed

    def any_stepper_joints_moving(self):
        any_moving = False
        for name, stepper_joint in self.stepper_joints.items():
            if stepper_joint.stepper.is_moving():
                any_moving = True
                break
        return any_moving
