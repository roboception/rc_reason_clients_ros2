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
from rc_reason_msgs.srv import SilhouetteMatchDetectObject
from rc_reason_msgs.srv import CalibrateBasePlane
from rc_reason_msgs.srv import GetBasePlaneCalibration
from rc_reason_msgs.srv import DeleteBasePlaneCalibration
from rc_reason_msgs.srv import GetRegionsOfInterest2D
from rc_reason_msgs.srv import SetRegionOfInterest2D
from rc_reason_msgs.srv import DeleteRegionsOfInterest2D

from rc_reason_clients.rest_client import RestClient


class SilhouetteMatchClient(RestClient):

    def __init__(self):
        super().__init__('rc_silhouettematch')

        self.srv = self.create_service(SilhouetteMatchDetectObject, 'detect_object', self.detect_cb)
        self.srv = self.create_service(CalibrateBasePlane, 'calibrate_base_plane', self.calibrate_cb)
        self.srv = self.create_service(GetBasePlaneCalibration, 'get_base_plane_calibration', self.get_calib_cb)
        self.srv = self.create_service(DeleteBasePlaneCalibration, 'delete_base_plane_calibration', self.delete_calib_cb)
        self.srv = self.create_service(GetRegionsOfInterest2D, 'get_regions_of_interest_2d', self.get_rois_cb)
        self.srv = self.create_service(SetRegionOfInterest2D, 'set_region_of_interest_2d', self.set_roi_cb)
        self.srv = self.create_service(DeleteRegionsOfInterest2D, 'delete_regions_of_interest_2d', self.delete_rois_cb)

    def detect_cb(self, request, response):
        self.call_rest_service('detect_object', request, response)
        return response

    def calibrate_cb(self, request, response):
        self.call_rest_service('calibrate_base_plane', request, response)
        return response

    def get_calib_cb(self, request, response):
        self.call_rest_service('get_base_plane_calibration', request, response)
        return response

    def delete_calib_cb(self, request, response):
        self.call_rest_service('delete_base_plane_calibration', request, response)
        return response

    def get_rois_cb(self, request, response):
        self.call_rest_service('get_regions_of_interest_2d', request, response)
        return response

    def set_roi_cb(self, request, response):
        self.call_rest_service('set_region_of_interest_2d', request, response)
        return response

    def delete_rois_cb(self, request, response):
        self.call_rest_service('delete_regions_of_interest_2d', request, response)
        return response


def main(args=None):
    rclpy.init(args=args)

    client = SilhouetteMatchClient()

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
