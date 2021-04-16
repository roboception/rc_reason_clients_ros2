# Copyright 2021 Roboception GmbH
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

from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from rc_reason_msgs.srv import SetLoadCarrier, GetLoadCarriers, DeleteLoadCarriers
from rc_reason_msgs.srv import DetectLoadCarriers, DetectFillingLevel
from rc_reason_msgs.srv import GetRegionsOfInterest3D, SetRegionOfInterest3D, DeleteRegionsOfInterest3D
from rc_reason_msgs.srv import GetRegionsOfInterest2D, SetRegionOfInterest2D, DeleteRegionsOfInterest2D

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from rc_reason_clients.rest_client import RestClient
from rc_reason_clients.transform_helpers import lc_to_marker, load_carrier_to_tf


class LoadCarrierClient(RestClient):

    def __init__(self):
        super().__init__('rc_load_carrier')

        # client only parameters
        self.declare_parameter(
            "publish_tf",
            True,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Publish detected loadcarriers and items via TF"
            )
        )
        self.declare_parameter(
            "publish_markers",
            True,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Publish detected loadcarriers and grasps as visalization markers"
            )
        )

        self.pub_tf = self.create_publisher(TFMessage, "/tf", QoSProfile(depth=100))
        self.pub_markers = self.create_publisher(MarkerArray, "visualization_marker_array", QoSProfile(depth=10))

        self.lc_markers = []

        self.add_rest_service(DetectLoadCarriers, 'detect_load_carriers', self.lc_cb)
        self.add_rest_service(DetectFillingLevel, 'detect_filling_level', self.lc_cb)
        self.add_rest_service(SetLoadCarrier, 'set_load_carrier', self.generic_cb)
        self.add_rest_service(GetLoadCarriers, 'get_load_carriers', self.generic_cb)
        self.add_rest_service(DeleteLoadCarriers, 'delete_load_carriers', self.generic_cb)
        self.add_rest_service(SetRegionOfInterest3D, 'set_region_of_interest', self.generic_cb)
        self.add_rest_service(GetRegionsOfInterest3D, 'get_regions_of_interest', self.generic_cb)
        self.add_rest_service(DeleteRegionsOfInterest3D, 'delete_regions_of_interest', self.generic_cb)
        self.add_rest_service(GetRegionsOfInterest2D, 'get_regions_of_interest_2d', self.generic_cb)
        self.add_rest_service(SetRegionOfInterest2D, 'set_region_of_interest_2d', self.generic_cb)
        self.add_rest_service(DeleteRegionsOfInterest2D, 'delete_regions_of_interest_2d', self.generic_cb)

    def generic_cb(self, srv_name, request, response):
        self.call_rest_service(srv_name, request, response)
        return response

    def lc_cb(self, srv_name, request, response):
        self.call_rest_service(srv_name, request, response)
        self.publish_lcs(response.load_carriers)
        return response

    def publish_lcs(self, lcs):
        if lcs and self.get_parameter('publish_tf').value:
            transforms = [load_carrier_to_tf(lc, i) for i, lc in enumerate(lcs)]
            self.pub_tf.publish(TFMessage(transforms=transforms))
        if self.get_parameter('publish_markers').value:
            self.publish_lc_markers(lcs)

    def publish_lc_markers(self, lcs):
        new_markers = []
        for i, lc in enumerate(lcs):
            m = lc_to_marker(lc, i, f"{self.rest_name}_lcs")
            if i < len(self.lc_markers):
                self.lc_markers[i] = m
            else:
                self.lc_markers.append(m)
            new_markers.append(m)
        for i in range(len(lcs), len(self.lc_markers)):
            # delete old markers
            self.lc_markers[i].action = Marker.DELETE
        self.pub_markers.publish(MarkerArray(markers=self.lc_markers))
        self.lc_markers = new_markers


def main(args=None):
    rclpy.init(args=args)

    client = LoadCarrierClient()

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
