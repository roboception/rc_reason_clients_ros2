from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import json

import requests
from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry

from rc_reason_clients.message_conversion import extract_values, populate_instance


def requests_retry_session(retries=3,
                           backoff_factor=0.3,
                           status_forcelist=[429],
                           session=None):
    """"
    A requests session that will retry on TOO_MANY_REQUESTS.

    E.g. replace requests.get() with requests_retry_session().get()
    """
    session = session or requests.Session()
    retry = Retry(
        total=retries,
        read=retries,
        connect=retries,
        backoff_factor=backoff_factor,
        status_forcelist=status_forcelist,
    )
    adapter = HTTPAdapter(max_retries=retry)
    session.mount('http://', adapter)
    session.mount('https://', adapter)
    return session


class RestClient(Node):

    def __init__(self, rest_name):
        super().__init__(rest_name + '_client')
        self.rest_name = rest_name
        self.declare_parameter('host', '', ParameterDescriptor(type=ParameterType.PARAMETER_STRING, read_only=True))
        self.host = self.get_parameter('host').value

    def call_rest_service(self, service, request, response):
        try:
            args = extract_values(request)
            self.get_logger().info(f'args: {args}')

            url = f'http://{self.host}/api/v1/nodes/{self.rest_name}/services/{service}'
            res = requests_retry_session().put(url, json={'args': args})

            j = res.json()
            self.get_logger().info(f"rest response: {json.dumps(j, indent=2)}")
            populate_instance(j['response'], response)
        except Exception as e:
            self.get_logger().error(str(e))
            response.return_code.value = -1000
            response.return_code.message = str(e)

    def set_rest_parameters(self, parameters):
        try:
            url = f'http://{self.host}/api/v1/nodes/{self.rest_name}/parameters'
            res = requests_retry_session().put(url, json=parameters)
            j = res.json()
            self.get_logger().info(f"rest response: {json.dumps(j, indent=2)}")
            if 'return_code' in j and j['return_code']['value'] != 0:
                self.get_logger().warn(f"Setting parameter failed: {j['return_code']['message']}")
                return False
            if res.status_code != 200:
                self.get_logger().error(f"Unexpected status code: {res.status_code}")
                return False
            return True
        except Exception as e:
            self.get_logger().error(str(e))
            return False
