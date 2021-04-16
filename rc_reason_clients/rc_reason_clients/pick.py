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
from rc_reason_msgs.srv import ComputeGrasps, DetectItems

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from rc_reason_clients.rest_client import RestClient
from rc_reason_clients.transform_helpers import lc_to_marker, load_carrier_to_tf


def grasp_to_tf(grasp, postfix):
    tf = TransformStamped()
    tf.header.frame_id = grasp.pose.header.frame_id
    tf.child_frame_id = f"grasp_{postfix}"
    tf.header.stamp = grasp.pose.header.stamp
    tf.transform.translation.x = grasp.pose.pose.position.x
    tf.transform.translation.y = grasp.pose.pose.position.y
    tf.transform.translation.z = grasp.pose.pose.position.z
    tf.transform.rotation = grasp.pose.pose.orientation
    return tf


def item_to_tf(item, postfix):
    tf = TransformStamped()
    tf.header.frame_id = item.pose.header.frame_id
    tf.child_frame_id = f"boxitem_{postfix}"
    tf.header.stamp = item.pose.header.stamp
    tf.transform.translation.x = item.pose.pose.position.x
    tf.transform.translation.y = item.pose.pose.position.y
    tf.transform.translation.z = item.pose.pose.position.z
    tf.transform.rotation = item.pose.pose.orientation
    return tf


class PickClient(RestClient):

    def __init__(self, rest_name):
        ignored_parameters = ['load_carrier_crop_distance', 'load_carrier_model_tolerance']
        super().__init__(rest_name, ignored_parameters)

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
        self.grasp_markers = []
        self.lc_markers = []

        self.pub_tf = self.create_publisher(TFMessage, "/tf", QoSProfile(depth=100))
        self.pub_markers = self.create_publisher(MarkerArray, "visualization_marker_array", QoSProfile(depth=10))

        self.start()

    def start(self):
        self.get_logger().info(f"starting {self.rest_name}")
        self.call_rest_service('start')

    def stop(self):
        self.get_logger().info(f"stopping {self.rest_name}")
        self.call_rest_service('stop')

    def publish_lcs(self, lcs):
        if lcs and self.get_parameter('publish_tf').value:
            transforms = [load_carrier_to_tf(lc, i) for i, lc in enumerate(lcs)]
            self.pub_tf.publish(TFMessage(transforms=transforms))
        if self.get_parameter('publish_markers').value:
            self.publish_lc_markers(lcs)

    def publish_grasps(self, grasps):
        if grasps and self.get_parameter('publish_tf').value:
            transforms = [grasp_to_tf(grasp, i) for i, grasp in enumerate(grasps)]
            self.pub_tf.publish(TFMessage(transforms=transforms))
        if self.get_parameter('publish_markers').value:
            self.publish_grasps_markers(grasps)

    def publish_grasps_markers(self, grasps):
        def create_marker(grasp, id):
            m = Marker(action=Marker.ADD, type=Marker.SPHERE)
            m.color = ColorRGBA(r=0.8, g=0.2, b=0.0, a=0.8)
            m.scale.x = grasp.max_suction_surface_length
            m.scale.y = grasp.max_suction_surface_width
            m.scale.z = 0.001
            m.header = grasp.pose.header
            m.pose = grasp.pose.pose
            m.id = i
            m.ns = f"{self.rest_name}_grasps"
            return m

        new_markers = []
        for i, grasp in enumerate(grasps):
            m = create_marker(grasp, i)
            if i < len(self.grasp_markers):
                self.grasp_markers[i] = m
            else:
                self.grasp_markers.append(m)
            new_markers.append(m)
        for i in range(len(grasps), len(self.grasp_markers)):
            # delete old markers
            self.grasp_markers[i].action = Marker.DELETE
        self.pub_markers.publish(MarkerArray(markers=self.grasp_markers))
        self.grasp_markers = new_markers

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


class ItemPickClient(PickClient):

    def __init__(self, rest_name):
        super().__init__(rest_name)
        self.add_rest_service(ComputeGrasps, 'compute_grasps', self.compute_grasps_cb)

    def compute_grasps_cb(self, srv_name, request, response):
        self.call_rest_service(srv_name, request, response)
        self.publish_lcs(response.load_carriers)
        self.publish_grasps(response.grasps)
        return response


class BoxPickClient(PickClient):

    def __init__(self, rest_name):
        super().__init__(rest_name)
        self.add_rest_service(ComputeGrasps, 'compute_grasps', self.compute_grasps_cb)
        self.add_rest_service(DetectItems, 'detect_items', self.detect_items_cb)

    def compute_grasps_cb(self, srv_name, request, response):
        self.call_rest_service(srv_name, request, response)
        self.publish_lcs(response.load_carriers)
        self.publish_grasps(response.grasps)
        self.publish_items(response.items)
        return response

    def detect_items_cb(self, srv_name, request, response):
        self.call_rest_service(srv_name, request, response)
        self.publish_lcs(response.load_carriers)
        self.publish_items(response.items)
        return response

    def publish_items(self, items):
        if not items:
            return
        if not self.get_parameter('publish_tf').value:
            return
        transforms = [item_to_tf(item, i) for i, item in enumerate(items)]
        self.pub_tf.publish(TFMessage(transforms=transforms))


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
