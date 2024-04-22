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

from functools import partial
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult, IntegerRange, FloatingPointRange

import json

import requests
from requests.adapters import HTTPAdapter
from requests.packages.urllib3.util.retry import Retry

from rc_reason_clients.message_conversion import extract_values, populate_instance, type_map


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


def parameter_descriptor_from_rest(p):
    pd = ParameterDescriptor(description=p['description'])
    if p['type'] in type_map['str']:
        pd.type = ParameterType.PARAMETER_STRING
    elif p['type'] in type_map['bool']:
        pd.type = ParameterType.PARAMETER_BOOL
    elif p['type'] in type_map['int']:
        pd.type = ParameterType.PARAMETER_INTEGER
        pd.integer_range = [IntegerRange(from_value=p['min'], to_value=p['max'])]
    elif p['type'] in type_map['float']:
        pd.type = ParameterType.PARAMETER_DOUBLE
        pd.floating_point_range = [FloatingPointRange(from_value=p['min'], to_value=p['max'])]
    return pd


class RestClient(Node):

    def __init__(self, rest_name, ignored_parameters=[]):
        super().__init__(rest_name + '_client')
        self.rest_name = rest_name
        self.ignored_parameters = ignored_parameters

        self.declare_parameter('host', '', ParameterDescriptor(type=ParameterType.PARAMETER_STRING, read_only=True))
        self.host = self.get_parameter('host').value

        self.declare_parameter('pipeline', 0, ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, read_only=True))
        self.pipeline = self.get_parameter('pipeline').value

        self.api_node_prefix = f"http://{self.host}/api/v2/pipelines/{self.pipeline}/nodes/{self.rest_name}"

        self.rest_param_names = None
        self.declare_rest_parameters()
        self.add_on_set_parameters_callback(self.params_callback)

        self.rest_services = []

    def declare_rest_parameters(self):
        rest_params = [p for p in self._get_rest_parameters() if p['name'] not in self.ignored_parameters]
        self.rest_param_names = [p['name'] for p in rest_params]
        def to_ros_param(p):
            return p['name'], p['value'], parameter_descriptor_from_rest(p)
        parameters = [to_ros_param(p) for p in rest_params]
        self.declare_parameters('', parameters)

    def params_callback(self, parameters):
        new_rest_params = [{'name': p.name, 'value': p.value} for p in parameters if p.name in self.rest_param_names]
        if new_rest_params:
            success = self._set_rest_parameters(new_rest_params)
            return SetParametersResult(successful=success)
        return SetParametersResult(successful=True)

    def call_rest_service(self, service, request=None, response=None):
        try:
            args = {}
            if request is not None:
                # convert ROS request to JSON (with custom API mappings)
                args = extract_values(request)
                self.get_logger().debug(f'calling {service} with args: {args}')

            url = f'{self.api_node_prefix}/services/{service}'
            res = requests_retry_session().put(url, json={'args': args})

            j = res.json()
            self.get_logger().debug(f"{service} rest response: {json.dumps(j, indent=2)}")
            rc = j['response'].get('return_code')
            if rc is not None and rc['value'] < 0:
                self.get_logger().warn(f"service {service} returned an error: [{rc['value']}] {rc['message']}")

            # convert to ROS response
            if response is not None:
                populate_instance(j['response'], response)
        except Exception as e:
            self.get_logger().error(str(e))
            if response is not None and hasattr(response, 'return_code'):
                response.return_code.value = -1000
                response.return_code.message = str(e)

    def _get_rest_parameters(self):
        try:
            url = f'{self.api_node_prefix}/parameters'
            res = requests_retry_session().get(url)
            if res.status_code != 200:
                self.get_logger().error(f"Getting parameters failed with status code: {res.status_code}")
                return []
            return res.json()
        except Exception as e:
            self.get_logger().error(str(e))
            return []

    def _set_rest_parameters(self, parameters):
        try:
            url = f'{self.api_node_prefix}/parameters'
            res = requests_retry_session().put(url, json=parameters)
            j = res.json()
            self.get_logger().debug(f"set parameters response: {json.dumps(j, indent=2)}")
            if 'return_code' in j and j['return_code']['value'] != 0:
                self.get_logger().warn(f"Setting parameter failed: {j['return_code']['message']}")
                return False
            if res.status_code != 200:
                self.get_logger().error(f"Setting parameters failed with status code: {res.status_code}")
                return False
            return True
        except Exception as e:
            self.get_logger().error(str(e))
            return False

    def add_rest_service(self, srv_type, srv_name, callback):
        """create a service and inject the REST-API service name"""
        srv = self.create_service(srv_type, f"{self.get_name()}/{srv_name}", partial(callback, srv_name))
        self.rest_services.append(srv)
