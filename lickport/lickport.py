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

from std_msgs.msg import String

from Phidget22.Phidget import *
from Phidget22.PhidgetException import *
from Phidget22.Devices.Stepper import *
from Phidget22.Devices.DigitalInput import *

from .PhidgetHelperFunctions import *


def on_attach_handler(self):
    ph = self
    try:
        channelClassName = ph.getChannelClassName()
        serialNumber = ph.getDeviceSerialNumber()
        channel = ph.getChannel()
        if(ph.getDeviceClass() == DeviceClass.PHIDCLASS_VINT):
            hubPort = ph.getHubPort()
            print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + str(serialNumber) +
                "\n\t-> Hub Port: " + str(hubPort) + "\n\t-> Channel:  " + str(channel) + "\n")
        else:
            print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + str(serialNumber) +
                    "\n\t-> Channel:  " + str(channel) + "\n")

        try:
            ph.setDataInterval(100)
            ph.setEngaged(True)
        except AttributeError:
            pass
        except PhidgetException as e:
            sys.stderr.write("Runtime Error\n\t")
            DisplayError(e)
            return

    except PhidgetException as e:
        print("\nError in Attach Event:")
        DisplayError(e)
        traceback.print_exc()
        return

def on_detach_handler(self):
    ph = self
    try:
        channelClassName = ph.getChannelClassName()
        serialNumber = ph.getDeviceSerialNumber()
        channel = ph.getChannel()
        if(ph.getDeviceClass() == DeviceClass.PHIDCLASS_VINT):
            hubPort = ph.getHubPort()
            print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + str(serialNumber) +
                "\n\t-> Hub Port: " + str(hubPort) + "\n\t-> Channel:  " + str(channel) + "\n")
        else:
            print("\n\t-> Channel Class: " + channelClassName + "\n\t-> Serial Number: " + str(serialNumber) +
                    "\n\t-> Channel:  " + str(channel) + "\n")

    except PhidgetException as e:
        print("\nError in Detach Event:")
        DisplayError(e)
        traceback.print_exc()
        return

def on_error_handler(self, error_code, error_string):

    sys.stderr.write("[Phidget Error Event] -> " + error_string + " (" + str(error_code) + ")\n")

class Joint:
    ACCELERATION = 10000
    VELOCITY_LIMIT = 10000
    HOME_VELOCITY_LIMIT = 1000
    HOME_TARGET_POSITION = -10000
    CURRENT_LIMIT = 0.140
    HOLDING_CURRENT_LIMIT = 0
    ATTACHMENT_TIMEOUT = 5000

    def __init__(self, stepper_channel_info, home_switch_channel_info, joint_name):
        self._joint_name = joint_name
        try:
            self._stepper = Stepper()
            self._home_switch = DigitalInput()
        except PhidgetException as e:
            sys.stderr.write("Runtime Error -> Creating Stepper: \n\t")
            DisplayError(e)
            raise
        except RuntimeError as e:
            sys.stderr.write("Runtime Error -> Creating Stepper: \n\t" + e)
            raise
        self._setup_channel(self._stepper, stepper_channel_info)
        self._setup_channel(self._home_switch, home_switch_channel_info)

    def _setup_channel(self,channel,info):
        channel.setDeviceSerialNumber(info.deviceSerialNumber)
        channel.setHubPort(info.hubPort)
        channel.setIsHubPortDevice(info.isHubPortDevice)
        channel.setChannel(info.channel)

        channel.setOnAttachHandler(on_attach_handler)
        channel.setOnDetachHandler(on_detach_handler)
        channel.setOnErrorHandler(on_error_handler)

    def openWaitForAttachment(self):
        self._stepper.openWaitForAttachment(self.ATTACHMENT_TIMEOUT)
        self._home_switch.openWaitForAttachment(self.ATTACHMENT_TIMEOUT)

    def setup(self):
        self._stepper.setAcceleration(self.ACCELERATION)
        self._stepper.setCurrentLimit(self.CURRENT_LIMIT)
        self._stepper.setVelocityLimit(self.VELOCITY_LIMIT)
        self._stepper.setHoldingCurrentLimit(self.HOLDING_CURRENT_LIMIT)

    def home(self):
        if self._home_switch.getState():
            self._stepper.setVelocityLimit(self.HOME_VELOCITY_LIMIT)
            self._stepper.setTargetPosition(self.HOME_TARGET_POSITION)
            while self._home_switch.getState():
                pass
            print("set velocity to 0")
            self._stepper.setVelocityLimit(0.0)
            self._stepper.addPositionOffset(-self._stepper.getPosition())
            print("position = " + str(self._stepper.getPosition()))
            self._stepper.setTargetPosition(0)
            self._stepper.setVelocityLimit(self.VELOCITY_LIMIT)
        else:
            print('joint already homed!')

    def close(self):
        self._stepper.setOnPositionChangeHandler(None)
        self._stepper.close()
        self._home_switch.close()

    def set_target_position(self, target_position):
        self._stepper.setTargetPosition(target_position)

class Lickport(Node):
    X_JOINT_STEPPER_HUB_PORT = 0
    Y_JOINT_STEPPER_HUB_PORT = 1
    Z_JOINT_STEPPER_HUB_PORT = 2
    X_JOINT_HOME_SWITCH_HUB_PORT = 5
    Y_JOINT_HOME_SWITCH_HUB_PORT = 4
    Z_JOINT_HOME_SWITCH_HUB_PORT = 3
    TARGET_POSITION = 2000

    def __init__(self):
        super().__init__('lickport')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self._setup_joints()

    def _setup_joints(self):
        try:
            stepper_channel_info = ChannelInfo()
            stepper_channel_info.deviceSerialNumber = Phidget.ANY_SERIAL_NUMBER
            stepper_channel_info.isHubPortDevice = False
            stepper_channel_info.channel = 0
            stepper_channel_info.isVint = True
            stepper_channel_info.netInfo.isRemote = False

            home_switch_channel_info = ChannelInfo()
            home_switch_channel_info.deviceSerialNumber = Phidget.ANY_SERIAL_NUMBER
            home_switch_channel_info.isHubPortDevice = True
            home_switch_channel_info.channel = 0
            home_switch_channel_info.isVINT = True
            home_switch_channel_info.netInfo.isRemote = False

            stepper_channel_info.hubPort = self.X_JOINT_STEPPER_HUB_PORT
            home_switch_channel_info.hubPort = self.X_JOINT_HOME_SWITCH_HUB_PORT
            joint_name = 'x'
            self._x_joint = Joint(stepper_channel_info, home_switch_channel_info, joint_name)

            stepper_channel_info.hubPort = self.Y_JOINT_STEPPER_HUB_PORT
            home_switch_channel_info.hubPort = self.Y_JOINT_HOME_SWITCH_HUB_PORT
            joint_name = 'y'
            self._y_joint = Joint(stepper_channel_info, home_switch_channel_info, joint_name)

            stepper_channel_info.hubPort = self.Z_JOINT_STEPPER_HUB_PORT
            home_switch_channel_info.hubPort = self.Z_JOINT_HOME_SWITCH_HUB_PORT
            joint_name = 'z'
            self._z_joint = Joint(stepper_channel_info, home_switch_channel_info, joint_name)

            try:
                self._x_joint.openWaitForAttachment()
                self._y_joint.openWaitForAttachment()
                self._z_joint.openWaitForAttachment()
            except PhidgetException as e:
                PrintOpenErrorMessage(e)
                raise EndProgramSignal("Program Terminated: Open Failed")

            self._x_joint.setup()
            self._y_joint.setup()
            self._z_joint.setup()

        except PhidgetException as e:
            sys.stderr.write("\nExiting with error(s)...")
            DisplayError(e)
            traceback.print_exc()
            print("Cleaning up...")
            self._x_joint.close()
            self._y_joint.close()
            self._z_joint.close()
            return 1
        except EndProgramSignal as e:
            print(e)
            print("Cleaning up...")
            self._x_joint.close()
            self._y_joint.close()
            self._z_joint.close()
            return 1

        def home_all():
            print('Homing y')
            self._y_joint.home()

            print('Homing x')
            self._x_joint.home()

            print('Homing z')
            self._z_joint.home()

        home_all()

        self._x_joint.set_target_position(self.TARGET_POSITION)
        self._y_joint.set_target_position(self.TARGET_POSITION)
        self._z_joint.set_target_position(self.TARGET_POSITION)

    def listener_callback(self, msg):
        self.get_logger().info('Lickport heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    lickport = Lickport()

    rclpy.spin(lickport)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lickport.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
