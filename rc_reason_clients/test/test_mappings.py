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

import pytest

from rc_reason_clients.message_conversion import extract_values, populate_instance, type_map
from rc_reason_msgs.msg import DetectedTag


def compare_header(ros_header, timestamp, frame_id):
    assert ros_header.stamp.sec == timestamp['sec']
    assert ros_header.stamp.nanosec == timestamp['nsec']
    assert ros_header.frame_id == frame_id

def compare_pose(ros_pose, rest_pose):
    assert ros_pose.position.x == rest_pose['position']['x']
    assert ros_pose.position.y == rest_pose['position']['y']
    assert ros_pose.position.z == rest_pose['position']['z']
    assert ros_pose.orientation.x == rest_pose['orientation']['x']
    assert ros_pose.orientation.y == rest_pose['orientation']['y']
    assert ros_pose.orientation.z == rest_pose['orientation']['z']
    assert ros_pose.orientation.w == rest_pose['orientation']['w']

def test_detectedtag():
    tag = {
            "id": "36h10_2319",
            "instance_id": "2367151514",
            "pose": {
                "orientation": {
                    "w": 0.661847902940126,
                    "x": -0.14557814643855335,
                    "y": -0.14578886497641433,
                    "z": -0.7207703958280759
                },
                "position": {
                    "x": 0.0419080633254838,
                    "y": -0.02554360431324062,
                    "z": 0.4794844275109502
                }
            },
            "pose_frame": "camera",
            "size": 0.026511077553573122,
            "timestamp": {
                "nsec": 764237750,
                "sec": 1591101453
            }
        }

    ros_tag = DetectedTag()
    populate_instance(tag, ros_tag)
    assert ros_tag.tag.id == tag['id']
    assert ros_tag.tag.size == tag['size']
    assert ros_tag.instance_id == tag['instance_id']
    compare_header(ros_tag.header, tag['timestamp'], tag['pose_frame'])
    compare_pose(ros_tag.pose.pose, tag['pose'])
    assert ros_tag.pose.header == ros_tag.header
