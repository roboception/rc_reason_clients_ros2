# Copyright 2020 Roboception GmbH
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rclpy

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rc_reason_msgs.srv import HandEyeCalibration
from rc_reason_msgs.srv import HandEyeCalibrationTrigger
from rc_reason_msgs.srv import SetHandEyeCalibration
from rc_reason_msgs.srv import SetHandEyeCalibrationPose

from rc_reason_clients.rest_client import RestClient


class HandEyeCalibClient(RestClient):

    def __init__(self):
        super().__init__('rc_hand_eye_calibration')

        self.srv = self.create_service(HandEyeCalibration, self.get_name() + '/calibrate', self.calibrate_cb)
        self.srv = self.create_service(HandEyeCalibration, self.get_name() + '/get_calibration', self.get_calib_cb)
        self.srv = self.create_service(SetHandEyeCalibration, self.get_name() + '/set_calibration', self.set_calib_cb)
        self.srv = self.create_service(HandEyeCalibrationTrigger, self.get_name() + '/save_calibration', self.save_calib_cb)
        self.srv = self.create_service(HandEyeCalibrationTrigger, self.get_name() + '/delete_calibration', self.delete_calib_cb)
        self.srv = self.create_service(HandEyeCalibrationTrigger, self.get_name() + '/reset_calibration', self.reset_calib_cb)
        self.srv = self.create_service(SetHandEyeCalibrationPose, self.get_name() + '/set_pose', self.set_pose_cb)

    def calibrate_cb(self, request, response):
        self.call_rest_service('calibrate', request, response)
        return response

    def get_calib_cb(self, request, response):
        self.call_rest_service('get_calibration', request, response)
        return response

    def set_calib_cb(self, request, response):
        self.call_rest_service('set_calibration', request, response)
        return response

    def save_calib_cb(self, request, response):
        self.call_rest_service('save_calibration', request, response)
        return response

    def delete_calib_cb(self, request, response):
        self.call_rest_service('delete_calibration', request, response)
        return response

    def reset_calib_cb(self, request, response):
        self.call_rest_service('reset_calibration', request, response)
        return response

    def set_pose_cb(self, request, response):
        self.call_rest_service('set_pose', request, response)
        return response


def main(args=None):
    rclpy.init(args=args)

    client = HandEyeCalibClient()

    host = client.get_parameter('host').value
    if not host:
        client.get_logger().error('host is not set')
        rclpy.shutdown()
        exit(1)

    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
