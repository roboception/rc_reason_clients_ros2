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

from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rc_reason_msgs.srv import SilhouetteMatchDetectObject
from rc_reason_msgs.srv import CalibrateBasePlane
from rc_reason_msgs.srv import GetBasePlaneCalibration
from rc_reason_msgs.srv import DeleteBasePlaneCalibration
from rc_reason_msgs.srv import GetRegionsOfInterest2D
from rc_reason_msgs.srv import SetRegionOfInterest2D
from rc_reason_msgs.srv import DeleteRegionsOfInterest2D

from rc_reason_clients.rest_client import RestClient


def instance_to_tf(instance):
    tf = TransformStamped()
    tf.header.frame_id = instance.pose_frame
    tf.child_frame_id = f"{instance.object_id}_{instance.id}"
    tf.header.stamp = instance.timestamp
    tf.transform.translation.x = instance.pose.position.x
    tf.transform.translation.y = instance.pose.position.y
    tf.transform.translation.z = instance.pose.position.z
    tf.transform.rotation = instance.pose.orientation
    return tf


class SilhouetteMatchClient(RestClient):

    def __init__(self):
        super().__init__('rc_silhouettematch')

        # client only parameters
        self.declare_parameter(
            "publish_tf",
            True,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Publish detected instances via TF"
            )
        )

        self.pub_tf = self.create_publisher(TFMessage, "/tf", QoSProfile(depth=100))

        self.add_rest_service(SilhouetteMatchDetectObject, 'detect_object', self.detect_cb)
        self.add_rest_service(CalibrateBasePlane, 'calibrate_base_plane', self.calibrate_cb)
        self.add_rest_service(GetBasePlaneCalibration, 'get_base_plane_calibration', self.get_calib_cb)
        self.add_rest_service(DeleteBasePlaneCalibration, 'delete_base_plane_calibration', self.delete_calib_cb)
        self.add_rest_service(GetRegionsOfInterest2D, 'get_regions_of_interest_2d', self.get_rois_cb)
        self.add_rest_service(SetRegionOfInterest2D, 'set_region_of_interest_2d', self.set_roi_cb)
        self.add_rest_service(DeleteRegionsOfInterest2D, 'delete_regions_of_interest_2d', self.delete_rois_cb)

    def detect_cb(self, srv_name, request, response):
        self.call_rest_service(srv_name, request, response)
        self.pub_instances(response.instances)
        return response

    def calibrate_cb(self, srv_name, request, response):
        self.call_rest_service(srv_name, request, response)
        return response

    def get_calib_cb(self, srv_name, request, response):
        self.call_rest_service(srv_name, request, response)
        return response

    def delete_calib_cb(self, srv_name, request, response):
        self.call_rest_service(srv_name, request, response)
        return response

    def get_rois_cb(self, srv_name, request, response):
        self.call_rest_service(srv_name, request, response)
        return response

    def set_roi_cb(self, srv_name, request, response):
        self.call_rest_service(srv_name, request, response)
        return response

    def delete_rois_cb(self, srv_name, request, response):
        self.call_rest_service(srv_name, request, response)
        return response

    def pub_instances(self, instances):
        if not instances:
            return
        if not self.get_parameter('publish_tf').value:
            return
        transforms = [instance_to_tf(i) for i in instances]
        self.pub_tf.publish(TFMessage(transforms=transforms))


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
