
import rclpy

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rc_reason_msgs.srv import DetectTags

from rc_reason_clients.rest_client import RestClient


class TagClient(RestClient):

    def __init__(self, name):
        super().__init__(name)

        self.srv = self.create_service(DetectTags, 'detect', self.detect_callback)

    def detect_callback(self, request, response):
        self.get_logger().info('Incoming detect request')
        self.call_rest_service('detect', request, response)
        return response


def main(args=None):
    rclpy.init(args=args)

    client = TagClient('rc_april_tag_detect')

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
