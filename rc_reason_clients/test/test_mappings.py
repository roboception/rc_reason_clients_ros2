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

from rc_reason_clients.message_conversion import (
    extract_values,
    populate_instance,
    type_map,
)
from rc_reason_msgs.msg import DetectedTag, Box
from shape_msgs.msg import Plane, SolidPrimitive
from rc_reason_msgs.srv import CalibrateBasePlane, GetBasePlaneCalibration
from rc_reason_msgs.srv import SetLoadCarrier, GetLoadCarriers, DetectLoadCarriers
from rc_reason_msgs.srv import SetRegionOfInterest3D, GetRegionsOfInterest3D


def compare_timestamp(ros_ts, ts):
    assert ros_ts.sec == ts["sec"]
    assert ros_ts.nanosec == ts["nsec"]


def compare_header(ros_header, timestamp, frame_id):
    compare_timestamp(ros_header.stamp, timestamp)
    assert ros_header.frame_id == frame_id


def compare_pose(ros_pose, rest_pose):
    assert ros_pose.position.x == rest_pose["position"]["x"]
    assert ros_pose.position.y == rest_pose["position"]["y"]
    assert ros_pose.position.z == rest_pose["position"]["z"]
    assert ros_pose.orientation.x == rest_pose["orientation"]["x"]
    assert ros_pose.orientation.y == rest_pose["orientation"]["y"]
    assert ros_pose.orientation.z == rest_pose["orientation"]["z"]
    assert ros_pose.orientation.w == rest_pose["orientation"]["w"]


def compare_plane(ros_plane, rest_plane):
    assert ros_plane.coef[0] == rest_plane["normal"]["x"]
    assert ros_plane.coef[1] == rest_plane["normal"]["y"]
    assert ros_plane.coef[2] == rest_plane["normal"]["z"]
    assert ros_plane.coef[3] == rest_plane["distance"]


def compare_primitives(ros, rest):
    for field_name in ros.get_fields_and_field_types():
        assert rest[field_name] == getattr(ros, field_name)


def test_detectedtag():
    tag = {
        "id": "36h10_2319",
        "instance_id": "2367151514",
        "pose": {
            "orientation": {
                "w": 0.661847902940126,
                "x": -0.14557814643855335,
                "y": -0.14578886497641433,
                "z": -0.7207703958280759,
            },
            "position": {
                "x": 0.0419080633254838,
                "y": -0.02554360431324062,
                "z": 0.4794844275109502,
            },
        },
        "pose_frame": "camera",
        "size": 0.026511077553573122,
        "timestamp": {"nsec": 764237750, "sec": 1591101453},
    }

    ros_tag = DetectedTag()
    populate_instance(tag, ros_tag)
    assert ros_tag.tag.id == tag["id"]
    assert ros_tag.tag.size == tag["size"]
    assert ros_tag.instance_id == tag["instance_id"]
    compare_header(ros_tag.header, tag["timestamp"], tag["pose_frame"])
    compare_pose(ros_tag.pose.pose, tag["pose"])
    assert ros_tag.pose.header == ros_tag.header


def test_plane():
    plane = {"distance": 0.3, "normal": {"y": 0.2, "x": 0.1, "z": 0.3}}
    ros_plane = Plane()
    populate_instance(plane, ros_plane)
    compare_plane(ros_plane, plane)


def test_get_base_plane_calibration_response():
    response = {
        "plane": {
            "distance": -0.7552042203510121,
            "normal": {
                "y": -0.02136428281262259,
                "x": 0.02327135522722706,
                "z": 0.999500881163089,
            },
            "pose_frame": "camera",
        },
        "return_code": {"message": "", "value": 0},
    }
    ros_res = GetBasePlaneCalibration.Response()
    populate_instance(response, ros_res)
    compare_plane(ros_res.plane, response["plane"])
    assert ros_res.pose_frame == response["plane"]["pose_frame"]
    assert ros_res.return_code.value == response["return_code"]["value"]
    assert ros_res.return_code.message == response["return_code"]["message"]


def test_calibrate_base_plane_response():
    response = {
        "timestamp": {"sec": 1593000384, "nsec": 386523224},
        "plane": {
            "distance": -0.7552042203510121,
            "normal": {
                "y": -0.02136428281262259,
                "x": 0.02327135522722706,
                "z": 0.999500881163089,
            },
            "pose_frame": "camera",
        },
        "return_code": {"message": "test", "value": 0},
    }
    ros_res = CalibrateBasePlane.Response()
    populate_instance(response, ros_res)
    compare_plane(ros_res.plane, response["plane"])
    assert ros_res.pose_frame == response["plane"]["pose_frame"]
    assert ros_res.return_code.value == response["return_code"]["value"]
    assert ros_res.return_code.message == response["return_code"]["message"]
    compare_timestamp(ros_res.timestamp, response["timestamp"])


@pytest.mark.parametrize("method", ["STEREO", "APRILTAG", "MANUAL"])
def test_calibrate_base_plane_request(method):
    ros_req = CalibrateBasePlane.Request()
    ros_req.pose_frame = "camera"
    ros_req.offset = 0.123
    ros_req.plane_estimation_method = method
    if method == "STEREO":
        ros_req.stereo_plane_preference = "CLOSEST"
    elif method == "MANUAL":
        ros_req.plane.coef = [0.1, -0.2, 0.3, 0.4]
    api_req = extract_values(ros_req)
    assert ros_req.pose_frame == api_req["pose_frame"]
    assert ros_req.plane_estimation_method == api_req["plane_estimation_method"]
    assert ros_req.offset == api_req["offset"]
    if method == "STEREO":
        assert ros_req.stereo_plane_preference == api_req["stereo"]["plane_preference"]
    elif method == "MANUAL":
        assert ros_req.plane.coef[0] == api_req["plane"]["normal"]["x"]
        assert ros_req.plane.coef[1] == api_req["plane"]["normal"]["y"]
        assert ros_req.plane.coef[2] == api_req["plane"]["normal"]["z"]
        assert ros_req.plane.coef[3] == api_req["plane"]["distance"]


def test_get_lcs():
    ros_req = GetLoadCarriers.Request()
    ros_req.load_carrier_ids = ["foo", "bar"]
    api_req = extract_values(ros_req)
    assert ros_req.load_carrier_ids == api_req["load_carrier_ids"]

    api_res = {
        "load_carriers": [
            {
                "id": "test",
                "inner_dimensions": {"x": 0.8, "y": 0.2, "z": 0.8},
                "outer_dimensions": {"x": 1.0, "y": 0.6, "z": 1.0},
                "pose": {
                    "orientation": {"w": 0.0, "x": 0.0, "y": 0.0, "z": 0.0},
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                },
                "pose_frame": "",
                "rim_thickness": {"x": 0.0, "y": 0.0},
            }
        ],
        "return_code": {"message": "test", "value": 1234},
    }
    ros_res = GetLoadCarriers.Response()
    populate_instance(api_res, ros_res)
    ros_lc = ros_res.load_carriers[0]
    api_lc = api_res["load_carriers"][0]
    compare_pose(ros_lc.pose.pose, api_lc["pose"])
    assert ros_lc.pose.header.frame_id == api_lc["pose_frame"]
    assert ros_lc.id == api_lc["id"]
    compare_primitives(ros_lc.inner_dimensions, api_lc["inner_dimensions"])
    compare_primitives(ros_lc.outer_dimensions, api_lc["outer_dimensions"])
    compare_primitives(ros_lc.rim_thickness, api_lc["rim_thickness"])
    assert ros_res.return_code.value == api_res["return_code"]["value"]
    assert ros_res.return_code.message == api_res["return_code"]["message"]


def test_set_lc():
    ros_req = SetLoadCarrier.Request()
    ros_req.load_carrier.id = "mylc"
    ros_req.load_carrier.inner_dimensions = Box(x=0.8, y=0.2, z=0.8)
    ros_req.load_carrier.outer_dimensions = Box(x=1.0, y=0.6, z=1.0)
    api_req = extract_values(ros_req)
    ros_lc = ros_req.load_carrier
    api_lc = api_req["load_carrier"]
    assert ros_lc.id == api_lc["id"]
    assert ros_lc.pose.header.frame_id == api_lc["pose_frame"]
    compare_pose(ros_lc.pose.pose, api_lc["pose"])
    compare_primitives(ros_lc.inner_dimensions, api_lc["inner_dimensions"])
    compare_primitives(ros_lc.outer_dimensions, api_lc["outer_dimensions"])
    compare_primitives(ros_lc.rim_thickness, api_lc["rim_thickness"])
    # don't send overfilled flag in set_load_carrier
    assert "overfilled" not in api_lc


def test_detect_lcs():
    ros_req = DetectLoadCarriers.Request()
    ros_req.load_carrier_ids = ["test"]
    ros_req.pose_frame = "camera"
    ros_req.region_of_interest_id = "my_roi"
    api_req = extract_values(ros_req)
    assert ros_req.load_carrier_ids == api_req["load_carrier_ids"]
    assert ros_req.pose_frame == api_req["pose_frame"]
    assert ros_req.region_of_interest_id == api_req["region_of_interest_id"]
    assert "robot_pose" not in api_req

    # check that API request contains robot pose if pose_frame is external
    ros_req.pose_frame = "external"
    api_req = extract_values(ros_req)
    assert "robot_pose" in api_req

    api_res = {
        "timestamp": {"sec": 1581614679, "nsec": 917540309},
        "load_carriers": [
            {
                "pose": {
                    "position": {
                        "y": -0.01687562098439693,
                        "x": 0.11888062561732585,
                        "z": 0.8873076959237803,
                    },
                    "orientation": {
                        "y": -0.046329159479736336,
                        "x": 0.9986625754609102,
                        "z": 0.006671112421383355,
                        "w": 0.021958331489780644,
                    },
                },
                "pose_frame": "camera",
                "inner_dimensions": {"y": 0.27, "x": 0.37, "z": 0.14},
                "outer_dimensions": {"y": 0.3, "x": 0.4, "z": 0.2},
                "overfilled": True,
                "rim_thickness": {"y": 0.0, "x": 0.0},
                "id": "test",
            }
        ],
        "return_code": {"message": "foo", "value": 123},
    }
    ros_res = DetectLoadCarriers.Response()
    populate_instance(api_res, ros_res)
    ros_lc = ros_res.load_carriers[0]
    api_lc = api_res["load_carriers"][0]
    compare_pose(ros_lc.pose.pose, api_lc["pose"])
    assert ros_lc.pose.header.frame_id == api_lc["pose_frame"]
    assert ros_lc.id == api_lc["id"]
    assert ros_lc.overfilled == api_lc["overfilled"]
    compare_primitives(ros_lc.inner_dimensions, api_lc["inner_dimensions"])
    compare_primitives(ros_lc.outer_dimensions, api_lc["outer_dimensions"])
    compare_primitives(ros_lc.rim_thickness, api_lc["rim_thickness"])
    assert ros_res.return_code.value == api_res["return_code"]["value"]
    assert ros_res.return_code.message == api_res["return_code"]["message"]
    compare_timestamp(ros_res.timestamp, api_res["timestamp"])


def test_set_roi_box():
    ros_req = SetRegionOfInterest3D.Request()
    ros_roi = ros_req.region_of_interest
    ros_roi.id = "myroi"
    ros_roi.pose.header.frame_id = "camera"
    ros_roi.pose.pose.position.x = 1.0
    ros_roi.pose.pose.orientation.w = 1.0
    ros_roi.primitive = SolidPrimitive(type=SolidPrimitive.BOX)
    ros_roi.primitive.dimensions = [1.0, 2.2, 3.0]

    api_req = extract_values(ros_req)

    api_roi = api_req["region_of_interest"]
    assert ros_roi.id == api_roi["id"]
    assert ros_roi.pose.header.frame_id == api_roi["pose_frame"]
    compare_pose(ros_roi.pose.pose, api_roi["pose"])
    assert api_roi["type"] == "BOX"
    assert ros_roi.primitive.dimensions[0] == api_roi["box"]["x"]
    assert ros_roi.primitive.dimensions[1] == api_roi["box"]["y"]
    assert ros_roi.primitive.dimensions[2] == api_roi["box"]["z"]


def test_set_roi_sphere():
    ros_req = SetRegionOfInterest3D.Request()
    ros_roi = ros_req.region_of_interest
    ros_roi.id = "myroi"
    ros_roi.pose.header.frame_id = "camera"
    ros_roi.pose.pose.position.x = 1.0
    ros_roi.pose.pose.orientation.w = 1.0
    ros_roi.primitive = SolidPrimitive(type=SolidPrimitive.SPHERE)
    ros_roi.primitive.dimensions = [1.1]

    api_req = extract_values(ros_req)

    api_roi = api_req["region_of_interest"]
    assert ros_roi.id == api_roi["id"]
    assert ros_roi.pose.header.frame_id == api_roi["pose_frame"]
    compare_pose(ros_roi.pose.pose, api_roi["pose"])
    assert api_roi["type"] == "SPHERE"
    assert ros_roi.primitive.dimensions[0] == api_roi["sphere"]["radius"]


def test_get_roi_box():
    ros_req = GetRegionsOfInterest3D.Request()
    ros_req.region_of_interest_ids = ["foo", "bar", "baz"]
    api_req = extract_values(ros_req)
    assert ros_req.region_of_interest_ids == api_req["region_of_interest_ids"]

    api_res = {
        "regions_of_interest": [
            {
                "box": {"x": 0.6, "y": 0.37, "z": 0.23},
                "id": "test",
                "pose": {
                    "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
                    "position": {"x": 0.06, "y": 0.0, "z": 0.83},
                },
                "pose_frame": "camera",
                "type": "BOX",
            },
            {
                "sphere": {"radius": 0.8},
                "id": "test_external",
                "pose": {
                    "orientation": {
                        "w": 0.7071067811865476,
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.7071067811865475,
                    },
                    "position": {"x": -0.1, "y": -0.5, "z": 0.5},
                },
                "pose_frame": "external",
                "type": "SPHERE",
            },
        ],
        "return_code": {"message": "", "value": 0},
    }
    ros_res = GetRegionsOfInterest3D.Response()
    populate_instance(api_res, ros_res)
    for i, ros_roi in enumerate(ros_res.regions_of_interest):
        api_roi = api_res["regions_of_interest"][i]
        assert ros_roi.id == api_roi["id"]
        if api_roi["type"] == "BOX":
            assert ros_roi.primitive.type == SolidPrimitive.BOX
            assert len(ros_roi.primitive.dimensions) == 3
            assert ros_roi.primitive.dimensions[0] == api_roi["box"]["x"]
            assert ros_roi.primitive.dimensions[1] == api_roi["box"]["y"]
            assert ros_roi.primitive.dimensions[2] == api_roi["box"]["z"]
        elif api_roi["type"] == "SPHERE":
            assert ros_roi.primitive.type == SolidPrimitive.SPHERE
            assert len(ros_roi.primitive.dimensions) == 1
            assert ros_roi.primitive.dimensions[0] == api_roi["sphere"]["radius"]
