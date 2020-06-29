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
from rc_reason_msgs.srv import SetLoadCarrier, GetLoadCarriers, DeleteLoadCarriers
from rc_reason_msgs.srv import SetRegionOfInterest3D, GetRegionsOfInterest3D, DeleteRegionsOfInterest3D
from rc_reason_msgs.srv import DetectLoadCarriers, DetectFillingLevel
from rc_reason_msgs.srv import ComputeGrasps, DetectItems

from rc_reason_clients.rest_client import RestClient


class PickClient(RestClient):

    def __init__(self, rest_name):
        super().__init__(rest_name)

        self.start()

        self.srv = self.create_service(SetLoadCarrier, self.get_name() + '/set_load_carrier', self.set_lc_cb)
        self.srv = self.create_service(GetLoadCarriers, self.get_name() + '/get_load_carriers', self.get_lcs_cb)
        self.srv = self.create_service(DeleteLoadCarriers, self.get_name() + '/delete_load_carriers', self.delete_lcs_cb)
        self.srv = self.create_service(SetRegionOfInterest3D, self.get_name() + '/set_region_of_interest', self.set_roi_cb)
        self.srv = self.create_service(GetRegionsOfInterest3D, self.get_name() + '/get_regions_of_interst', self.get_rois_cb)
        self.srv = self.create_service(DeleteRegionsOfInterest3D, self.get_name() + '/delete_regions_of_interest', self.delete_rois_cb)
        self.srv = self.create_service(DetectLoadCarriers, self.get_name() + '/detect_load_carriers', self.detect_lcs_cb)
        self.srv = self.create_service(DetectFillingLevel, self.get_name() + '/detect_filling_level', self.detect_filling_level_cb)

    def start(self):
        self.get_logger().info(f"starting {self.rest_name}")
        self.call_rest_service('start')

    def stop(self):
        self.get_logger().info(f"stopping {self.rest_name}")
        self.call_rest_service('stop')

    def set_lc_cb(self, request, response):
        self.call_rest_service('set_load_carrier', request, response)
        return response

    def get_lcs_cb(self, request, response):
        self.call_rest_service('get_load_carriers', request, response)
        return response

    def delete_lcs_cb(self, request, response):
        self.call_rest_service('delete_load_carriers', request, response)
        return response

    def set_roi_cb(self, request, response):
        self.call_rest_service('set_region_of_interest', request, response)
        return response

    def get_rois_cb(self, request, response):
        self.call_rest_service('get_regions_of_interst', request, response)
        return response

    def delete_rois_cb(self, request, response):
        self.call_rest_service('delete_regions_of_interest', request, response)
        return response

    def detect_lcs_cb(self, request, response):
        self.call_rest_service('detect_load_carriers', request, response)
        return response

    def detect_filling_level_cb(self, request, response):
        self.call_rest_service('detect_filling_level', request, response)
        return response


class ItemPickClient(PickClient):

    def __init__(self, rest_name):
        super().__init__(rest_name)
        self.srv = self.create_service(ComputeGrasps, self.get_name() + '/compute_grasps', self.compute_grasps_cb)

    def compute_grasps_cb(self, request, response):
        self.call_rest_service('compute_grasps', request, response)
        return response


class BoxPickClient(PickClient):

    def __init__(self, rest_name):
        super().__init__(rest_name)
        self.srv = self.create_service(ComputeGrasps, self.get_name() + '/compute_grasps', self.compute_grasps_cb)
        self.srv = self.create_service(DetectItems, self.get_name() + '/detect_items', self.detect_items_cb)

    def compute_grasps_cb(self, request, response):
        self.call_rest_service('compute_grasps', request, response)
        return response

    def detect_items_cb(self, request, response):
        self.call_rest_service('detect_items', request, response)
        return response


def main(args=None, rest_node='rc_itempick'):
    rclpy.init(args=args)

    if rest_node == 'rc_itempick':
        client = ItemPickClient(rest_node)
    elif rest_node == 'rc_boxpick':
        client = BoxPickClient(rest_node)
    else:
        client.get_logger().error(f'unknown rest_node {rest_node}')
        rclpy.shutdown()
        exit(1)

    rclpy.get_default_context().on_shutdown(client.stop)

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


def rc_itempick_client(args=None):
    main(rest_node='rc_itempick')


def rc_boxpick_client(args=None):
    main(rest_node='rc_boxpick')


if __name__ == '__main__':
    main()
