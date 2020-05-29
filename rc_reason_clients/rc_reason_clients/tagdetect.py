
import rclpy

from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult, IntegerRange, FloatingPointRange
from rc_reason_msgs.srv import DetectTags

from rc_reason_clients.rest_client import RestClient


class TagClient(RestClient):

    def __init__(self, rest_name):
        super().__init__(rest_name)

        self.declare_parameter('detect_inverted_tags', False, ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Detect tags with black and white exchanged'))

        self.declare_parameter('forget_after_n_detections', 30, ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Number of detection runs after which to forget about a previous tag during tag re-identification',
                integer_range=[IntegerRange(from_value=1, to_value=30, step=1)]))

        self.declare_parameter('max_corner_distance', 0.005, ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Maximum distance of corresponding tag corners in meters during tag re-identification',
                floating_point_range=[FloatingPointRange(from_value=0.001, to_value=0.01, step=0.001)]))

        self.declare_parameter('quality', 'High', ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Quality/resolution of tag detection (High, Medium or Low)'))

        self.declare_parameter('use_cached_images', False, ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description='Use most recently received image pair instead of waiting for a new pair'))

        self.set_parameters_callback(self.params_callback)

        self.srv = self.create_service(DetectTags, 'detect', self.detect_callback)

    def params_callback(self, parameters):
        for p in parameters:
            if p.name == 'quality':
                if p.value not in ['High', 'Medium', 'Low']:
                    return SetParametersResult(successful=False)
        success = self.set_rest_parameters([{'name': p.name, 'value': p.value} for p in parameters])
        return SetParametersResult(successful=success)

    def detect_callback(self, request, response):
        self.get_logger().info('Incoming detect request')
        self.call_rest_service('detect', request, response)
        return response


def main(args=None, rest_node='rc_april_tag_detect'):
    rclpy.init(args=args)

    client = TagClient(rest_node)

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


def rc_april_tag_detect_client(args=None):
    main(rest_node='rc_april_tag_detect')


def rc_qr_code_detect_client(args=None):
    main(rest_node='rc_qr_code_detect')


if __name__ == '__main__':
    main()
