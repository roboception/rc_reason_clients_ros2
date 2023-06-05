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

from rclpy.clock import ROSClock
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rc_reason_msgs.srv import HandEyeCalibration
from rc_reason_msgs.srv import HandEyeCalibrationTrigger
from rc_reason_msgs.srv import SetHandEyeCalibration
from rc_reason_msgs.srv import SetHandEyeCalibrationPose

from rc_reason_clients.rest_client import RestClient


class HandEyeCalibClient(RestClient):

    def __init__(self):
        super().__init__('rc_hand_eye_calibration')

        # client only parameters
        self.declare_parameter(
            "camera_frame_id",
            "camera",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Name of the frame on the camera."
            )
        )
        self.declare_parameter(
            "end_effector_frame_id",
            "end_effector",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Name of the frame calibrated to when using a robot_mounted camera."
            )
        )
        self.declare_parameter(
            "base_frame_id",
            "base_link",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Name of the frame calibrated to when using a statically (externally) mounted camera."
            )
        )

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            )
        self.pub_tf = self.create_publisher(TFMessage, "/tf_static", qos)

        self.add_rest_service(HandEyeCalibration, 'calibrate', self.pub_cb)
        self.add_rest_service(HandEyeCalibration, 'get_calibration', self.pub_cb)
        self.add_rest_service(SetHandEyeCalibration, 'set_calibration', self.pub_cb)
        self.add_rest_service(HandEyeCalibrationTrigger, 'save_calibration', self.generic_cb)
        self.add_rest_service(HandEyeCalibrationTrigger, 'delete_calibration', self.generic_cb)
        self.add_rest_service(HandEyeCalibrationTrigger, 'reset_calibration', self.generic_cb)
        self.add_rest_service(SetHandEyeCalibrationPose, 'set_pose', self.generic_cb)

        # get initial calibration from sensor
        self.pub_cb('get_calibration', HandEyeCalibration.Request(), HandEyeCalibration.Response())

    def generic_cb(self, srv_name, request, response):
        self.call_rest_service(srv_name, request, response)
        if not response.success:
            self.get_logger().warn(response.message)
        return response

    def pub_cb(self, srv_name, request, response):
        self.call_rest_service(srv_name, request, response)
        if response.success:
            self.pub_hand_eye(response.pose, response.robot_mounted)
        else:
            self.get_logger().warn(response.message)
        return response

    def pub_hand_eye(self, pose, robot_mounted):
        transform = TransformStamped()
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        transform.header.stamp = ROSClock().now().to_msg()
        if robot_mounted:
            transform.header.frame_id = self.get_parameter('end_effector_frame_id').value
        else:
            transform.header.frame_id = self.get_parameter('base_frame_id').value
        transform.child_frame_id = self.get_parameter('camera_frame_id').value
        self.get_logger().info(f"publishing hand-eye calibration from {transform.header.frame_id} to {transform.child_frame_id}")
        self.pub_tf.publish(TFMessage(transforms=[transform]))


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
