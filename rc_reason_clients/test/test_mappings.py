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
from rc_reason_msgs.msg import DetectedTag, Box, ItemModel
from shape_msgs.msg import Plane, SolidPrimitive
from rc_reason_msgs.srv import CalibrateBasePlane, GetBasePlaneCalibration
from rc_reason_msgs.srv import (
    SetLoadCarrier,
    GetLoadCarriers,
    DetectLoadCarriers,
    DetectFillingLevel,
    DetectTags,
)
from rc_reason_msgs.srv import SetRegionOfInterest3D, GetRegionsOfInterest3D
from rc_reason_msgs.srv import ComputeGrasps, DetectItems


def assert_timestamp(ros_ts, ts):
    assert ros_ts.sec == ts["sec"]
    assert ros_ts.nanosec == ts["nsec"]


def assert_header(ros_header, timestamp, frame_id):
    assert_timestamp(ros_header.stamp, timestamp)
    assert ros_header.frame_id == frame_id


def assert_pose(ros_pose, api_pose):
    assert ros_pose.position.x == api_pose["position"]["x"]
    assert ros_pose.position.y == api_pose["position"]["y"]
    assert ros_pose.position.z == api_pose["position"]["z"]
    assert ros_pose.orientation.x == api_pose["orientation"]["x"]
    assert ros_pose.orientation.y == api_pose["orientation"]["y"]
    assert ros_pose.orientation.z == api_pose["orientation"]["z"]
    assert ros_pose.orientation.w == api_pose["orientation"]["w"]


def assert_plane(ros_plane, api_plane):
    assert ros_plane.coef[0] == api_plane["normal"]["x"]
    assert ros_plane.coef[1] == api_plane["normal"]["y"]
    assert ros_plane.coef[2] == api_plane["normal"]["z"]
    assert ros_plane.coef[3] == api_plane["distance"]


def assert_primitives(ros, api):
    for field_name, field_type in ros.get_fields_and_field_types().items():
        if "/" in field_type:
            assert_primitives(getattr(ros, field_name), api[field_name])
        else:
            assert api[field_name] == getattr(ros, field_name)


def assert_lc(ros_lc, api_lc, timestamp=None, pose_required=True):
    if pose_required:
        assert_pose(ros_lc.pose.pose, api_lc["pose"])
    assert ros_lc.pose.header.frame_id == api_lc["pose_frame"]
    if timestamp is not None:
        assert_timestamp(ros_lc.pose.header.stamp, timestamp)
    assert ros_lc.id == api_lc["id"]
    assert_primitives(ros_lc.inner_dimensions, api_lc["inner_dimensions"])
    assert_primitives(ros_lc.outer_dimensions, api_lc["outer_dimensions"])
    assert_primitives(ros_lc.rim_thickness, api_lc["rim_thickness"])
    if "overfilled" in api_lc:
        assert ros_lc.overfilled == api_lc["overfilled"]


def assert_grasp(ros_grasp, api_grasp):
    assert_pose(ros_grasp.pose.pose, api_grasp["pose"])
    assert_header(
        ros_grasp.pose.header, api_grasp["timestamp"], api_grasp["pose_frame"]
    )
    assert ros_grasp.quality == api_grasp["quality"]
    assert (
        ros_grasp.max_suction_surface_length == api_grasp["max_suction_surface_length"]
    )
    assert ros_grasp.max_suction_surface_width == api_grasp["max_suction_surface_width"]


def assert_item(ros_item, api_item):
    assert_pose(ros_item.pose.pose, api_item["pose"])
    assert_header(ros_item.pose.header, api_item["timestamp"], api_item["pose_frame"])


def test_detect_tags():
    ros_req = DetectTags.Request()

    # robot_pose should only be provided if pose_frame is external
    ros_req.pose_frame = "camera"
    api_req = extract_values(ros_req)
    assert ros_req.pose_frame == api_req["pose_frame"]
    assert "robot_pose" not in api_req

    ros_req.pose_frame = "external"
    ros_req.robot_pose.orientation.x = 1.0
    ros_req.robot_pose.orientation.y = 0.0
    ros_req.robot_pose.orientation.z = 0.0
    ros_req.robot_pose.orientation.w = 0.0
    ros_req.robot_pose.position.x = 1.1
    ros_req.robot_pose.position.y = -1.1
    ros_req.robot_pose.position.z = 0.5
    api_req = extract_values(ros_req)
    assert ros_req.pose_frame == api_req["pose_frame"]
    assert_pose(ros_req.robot_pose, api_req["robot_pose"])

    api_res = {
        "tags": [
            {
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
        ],
        "timestamp": {"nsec": 764237750, "sec": 1591101453},
        "return_code": {"message": "foo", "value": 1234},
    }

    ros_res = DetectTags.Response()
    populate_instance(api_res, ros_res)
    for i, ros_tag in enumerate(ros_res.tags):
        tag = api_res["tags"][i]
        assert ros_tag.tag.id == tag["id"]
        assert ros_tag.tag.size == tag["size"]
        assert ros_tag.instance_id == tag["instance_id"]
        assert_header(ros_tag.header, tag["timestamp"], tag["pose_frame"])
        assert_pose(ros_tag.pose.pose, tag["pose"])
        assert ros_tag.pose.header == ros_tag.header
    assert ros_res.return_code.value == api_res["return_code"]["value"]
    assert ros_res.return_code.message == api_res["return_code"]["message"]
    assert_timestamp(ros_res.timestamp, api_res["timestamp"])


def test_plane():
    plane = {"distance": 0.3, "normal": {"y": 0.2, "x": 0.1, "z": 0.3}}
    ros_plane = Plane()
    populate_instance(plane, ros_plane)
    assert_plane(ros_plane, plane)


def test_get_base_plane_calibration():
    ros_req = GetBasePlaneCalibration.Request()

    # in silhouettematch, the robot_pose can also be used in camera frame!
    ros_req.pose_frame = "camera"
    api_req = extract_values(ros_req)
    assert ros_req.pose_frame == api_req["pose_frame"]
    assert_pose(ros_req.robot_pose, api_req["robot_pose"])

    ros_req.pose_frame = "external"
    api_req = extract_values(ros_req)
    assert ros_req.pose_frame == api_req["pose_frame"]
    assert_pose(ros_req.robot_pose, api_req["robot_pose"])

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
        "return_code": {"message": "foo", "value": 1234},
    }
    ros_res = GetBasePlaneCalibration.Response()
    populate_instance(response, ros_res)
    assert_plane(ros_res.plane, response["plane"])
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
    assert_plane(ros_res.plane, response["plane"])
    assert ros_res.pose_frame == response["plane"]["pose_frame"]
    assert ros_res.return_code.value == response["return_code"]["value"]
    assert ros_res.return_code.message == response["return_code"]["message"]
    assert_timestamp(ros_res.timestamp, response["timestamp"])


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
    # in silhouettematch, the robot_pose can also be used in camera frame!
    assert_pose(ros_req.robot_pose, api_req["robot_pose"])

    ros_req.pose_frame = "external"
    api_req = extract_values(ros_req)
    assert ros_req.pose_frame == api_req["pose_frame"]
    assert_pose(ros_req.robot_pose, api_req["robot_pose"])


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
    assert_lc(ros_res.load_carriers[0], api_res["load_carriers"][0])
    assert ros_res.return_code.value == api_res["return_code"]["value"]
    assert ros_res.return_code.message == api_res["return_code"]["message"]


def test_set_lc():
    ros_req = SetLoadCarrier.Request()
    ros_req.load_carrier.id = "mylc"
    ros_req.load_carrier.inner_dimensions = Box(x=0.8, y=0.2, z=0.8)
    ros_req.load_carrier.outer_dimensions = Box(x=1.0, y=0.6, z=1.0)
    api_req = extract_values(ros_req)
    assert_lc(ros_req.load_carrier, api_req["load_carrier"], pose_required=False)
    # don't send overfilled flag in set_load_carrier
    assert "overfilled" not in api_req["load_carrier"]
    # don't send pose (as prior) if frame_id is not set
    assert "pose" not in api_req["load_carrier"]

    # with prior
    ros_req.load_carrier.pose.header.frame_id = 'camera'
    api_req = extract_values(ros_req)
    assert_pose(ros_req.load_carrier.pose.pose, api_req["load_carrier"]["pose"])


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
    assert_lc(ros_lc, api_lc, api_res["timestamp"])
    assert ros_res.return_code.value == api_res["return_code"]["value"]
    assert ros_res.return_code.message == api_res["return_code"]["message"]
    assert_timestamp(ros_res.timestamp, api_res["timestamp"])


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
    assert_pose(ros_roi.pose.pose, api_roi["pose"])
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
    assert_pose(ros_roi.pose.pose, api_roi["pose"])
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
        assert_pose(ros_roi.pose.pose, api_roi["pose"])
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


def test_filling_level():
    ros_req = DetectFillingLevel.Request()
    ros_req.pose_frame = "camera"
    ros_req.region_of_interest_id = "testroi"
    ros_req.load_carrier_ids = ["mylc"]
    ros_req.filling_level_cell_count.x = 2
    ros_req.filling_level_cell_count.y = 3
    api_req = extract_values(ros_req)
    assert ros_req.pose_frame == api_req["pose_frame"]
    assert ros_req.region_of_interest_id == api_req["region_of_interest_id"]
    assert ros_req.load_carrier_ids == api_req["load_carrier_ids"]
    assert_primitives(
        ros_req.filling_level_cell_count, api_req["filling_level_cell_count"]
    )
    # don't send robot_pose if pose_frame is camera
    assert "robot_pose" not in api_req

    ros_req.pose_frame = "external"
    ros_req.robot_pose.position.y = 1.0
    ros_req.robot_pose.orientation.z = 1.0
    api_req = extract_values(ros_req)
    assert "robot_pose" in api_req
    assert_pose(ros_req.robot_pose, api_req["robot_pose"])

    api_res = {
        "timestamp": {"sec": 1586451776, "nsec": 640984219},
        "load_carriers": [
            {
                "pose": {
                    "position": {
                        "y": -0.016895702313915,
                        "x": 0.11865983503829655,
                        "z": 0.88755382218002,
                    },
                    "orientation": {
                        "y": -0.04670608639905016,
                        "x": 0.99864717766196,
                        "z": 0.006390199364671242,
                        "w": 0.021943839675236384,
                    },
                },
                "filling_level_cell_count": {"y": 1, "x": 1},
                "pose_frame": "camera",
                "inner_dimensions": {"y": 0.27, "x": 0.37, "z": 0.14},
                "outer_dimensions": {"y": 0.3, "x": 0.4, "z": 0.2},
                "overall_filling_level": {
                    "level_free_in_meters": {"max": 0.14, "mean": 0.131, "min": 0.085},
                    "cell_size": {"y": 0.26, "x": 0.36},
                    "coverage": 1.0,
                    "level_in_percent": {"max": 39.3, "mean": 6.4, "min": 0.0},
                    "cell_position": {
                        "y": -0.021338225361242996,
                        "x": 0.11973116380121526,
                        "z": 0.7876582912409178,
                    },
                },
                "overfilled": False,
                "rim_thickness": {"y": 0.0, "x": 0.0},
                "cells_filling_levels": [
                    {
                        "level_free_in_meters": {
                            "max": 0.14,
                            "mean": 0.131,
                            "min": 0.085,
                        },
                        "cell_size": {"y": 0.26, "x": 0.36},
                        "coverage": 1.0,
                        "level_in_percent": {"max": 39.3, "mean": 6.4, "min": 0.0},
                        "cell_position": {
                            "y": -0.021338225361242996,
                            "x": 0.11973116380121526,
                            "z": 0.7876582912409178,
                        },
                    }
                ],
                "id": "auer_40x30",
            }
        ],
        "return_code": {"message": "", "value": 0},
    }
    ros_res = DetectFillingLevel.Response()
    populate_instance(api_res, ros_res)
    ros_lc = ros_res.load_carriers[0]
    api_lc = api_res["load_carriers"][0]
    assert_lc(ros_lc, api_lc)
    assert ros_lc.overfilled == api_lc["overfilled"]
    assert_primitives(ros_lc.overall_filling_level, api_lc["overall_filling_level"])
    for i, ros_l in enumerate(ros_lc.cells_filling_levels):
        assert_primitives(ros_l, api_lc["cells_filling_levels"][i])
    assert ros_res.return_code.value == api_res["return_code"]["value"]
    assert ros_res.return_code.message == api_res["return_code"]["message"]
    assert_timestamp(ros_res.timestamp, api_res["timestamp"])


def test_compute_grasps():
    ros_req = ComputeGrasps.Request()
    ros_req.pose_frame = "camera"
    ros_req.suction_surface_length = 0.05
    ros_req.suction_surface_width = 0.02

    api_req = extract_values(ros_req)
    assert ros_req.pose_frame == api_req["pose_frame"]
    assert ros_req.suction_surface_length == api_req["suction_surface_length"]
    assert ros_req.suction_surface_width == api_req["suction_surface_width"]

    # don't send robot_pose if pose_frame is camera
    assert "robot_pose" not in api_req

    ros_req.pose_frame = "external"
    ros_req.robot_pose.position.y = 1.0
    ros_req.robot_pose.orientation.z = 1.0
    api_req = extract_values(ros_req)
    assert "robot_pose" in api_req
    assert_pose(ros_req.robot_pose, api_req["robot_pose"])

    assert len(api_req["item_models"]) == 0
    assert "region_of_interest_id" not in api_req
    assert "load_carrier_id" not in api_req
    assert "load_carrier_compartment" not in api_req
    assert "collision_detection" not in api_req

    ros_req.region_of_interest_id = "testroi"
    ros_req.load_carrier_id = "mylc"
    api_req = extract_values(ros_req)
    assert ros_req.region_of_interest_id == api_req["region_of_interest_id"]
    assert ros_req.load_carrier_id == api_req["load_carrier_id"]
    assert "load_carrier_compartment" not in api_req
    assert "collision_detection" not in api_req

    # add valid compartment
    ros_req.load_carrier_compartment.box.x = 0.1
    ros_req.load_carrier_compartment.box.y = 0.2
    ros_req.load_carrier_compartment.box.z = 0.3
    api_req = extract_values(ros_req)
    assert_primitives(
        ros_req.load_carrier_compartment, api_req["load_carrier_compartment"]
    )
    assert "collision_detection" not in api_req

    ros_req.collision_detection.gripper_id = "mygripper"
    ros_req.collision_detection.pre_grasp_offset.x = 1.0
    ros_req.collision_detection.pre_grasp_offset.y = 0.1
    ros_req.collision_detection.pre_grasp_offset.z = -0.5
    api_req = extract_values(ros_req)
    assert_primitives(ros_req.collision_detection, api_req["collision_detection"])

    api_res = {
        "timestamp": {"sec": 1587033051, "nsec": 179231838},
        "grasps": [
            {
                "uuid": "589ce564-7fcd-11ea-a789-00142d2cd4ce",
                "quality": 0.6812791396682123,
                "pose_frame": "camera",
                "item_uuid": "",
                "timestamp": {"sec": 1587033051, "nsec": 179231838},
                "pose": {
                    "position": {
                        "y": 0.04363611955261629,
                        "x": 0.07198067450728357,
                        "z": 0.8879013030941614,
                    },
                    "orientation": {
                        "y": -0.18422641602024026,
                        "x": 0.03447462246434434,
                        "z": 0.5485609394110762,
                        "w": 0.8148331263508596,
                    },
                },
                "max_suction_surface_width": 0.051409365319609185,
                "max_suction_surface_length": 0.09779866352968565,
                "type": "SUCTION",
            },
            {
                "uuid": "58a02a80-7fcd-11ea-a789-00142d2cd4ce",
                "quality": 0.7596309125250751,
                "pose_frame": "camera",
                "item_uuid": "",
                "timestamp": {"sec": 1587033051, "nsec": 179231838},
                "pose": {
                    "position": {
                        "y": 0.010809225849663155,
                        "x": 0.13863803337363131,
                        "z": 0.904460429033145,
                    },
                    "orientation": {
                        "y": -0.00608028855302178,
                        "x": -0.02148840363751206,
                        "z": 0.5712238542641026,
                        "w": 0.8204904551058998,
                    },
                },
                "max_suction_surface_width": 0.0546039270399626,
                "max_suction_surface_length": 0.1002807889119014,
                "type": "SUCTION",
            },
        ],
        "load_carriers": [
            {
                "pose": {
                    "position": {
                        "y": -0.0168824336375496,
                        "x": 0.1189406043995812,
                        "z": 0.8875302697155399,
                    },
                    "orientation": {
                        "y": -0.04632486703729271,
                        "x": 0.998664879978751,
                        "z": 0.006342615882086765,
                        "w": 0.02195985184108778,
                    },
                },
                "pose_frame": "camera",
                "inner_dimensions": {"y": 0.27, "x": 0.37, "z": 0.14},
                "outer_dimensions": {"y": 0.3, "x": 0.4, "z": 0.2},
                "overfilled": True,
                "rim_thickness": {"y": 0.0, "x": 0.0},
                "id": "mylc",
            }
        ],
        "return_code": {"message": "", "value": 0},
    }
    ros_res = ComputeGrasps.Response()
    populate_instance(api_res, ros_res)
    ros_lc = ros_res.load_carriers[0]
    api_lc = api_res["load_carriers"][0]
    assert_lc(ros_lc, api_lc, api_res["timestamp"])
    for i, ros_grasp in enumerate(ros_res.grasps):
        api_grasp = api_res["grasps"][i]
        assert_grasp(ros_grasp, api_grasp)


def test_detect_items():
    ros_req = DetectItems.Request()
    ros_req.pose_frame = "camera"
    im = ItemModel(type=ItemModel.RECTANGLE)
    im.rectangle.min_dimensions.x = 0.2
    im.rectangle.min_dimensions.y = 0.3
    im.rectangle.max_dimensions.x = 0.4
    im.rectangle.max_dimensions.y = 0.5
    ros_req.item_models.append(im)

    api_req = extract_values(ros_req)
    assert ros_req.pose_frame == api_req["pose_frame"]
    for i, ros_im in enumerate(ros_req.item_models):
        api_im = api_req["item_models"][i]
        assert_primitives(ros_im, api_im)

    # don't send robot_pose if pose_frame is camera
    assert "robot_pose" not in api_req

    ros_req.pose_frame = "external"
    ros_req.robot_pose.position.y = 1.0
    ros_req.robot_pose.orientation.z = 1.0
    api_req = extract_values(ros_req)
    assert "robot_pose" in api_req
    assert_pose(ros_req.robot_pose, api_req["robot_pose"])

    assert len(api_req["item_models"]) == 1
    assert "region_of_interest_id" not in api_req
    assert "load_carrier_id" not in api_req
    assert "load_carrier_compartment" not in api_req

    ros_req.region_of_interest_id = "testroi"
    ros_req.load_carrier_id = "mylc"
    api_req = extract_values(ros_req)
    assert ros_req.region_of_interest_id == api_req["region_of_interest_id"]
    assert ros_req.load_carrier_id == api_req["load_carrier_id"]
    assert "load_carrier_compartment" not in api_req

    # add valid compartment
    ros_req.load_carrier_compartment.box.x = 0.1
    ros_req.load_carrier_compartment.box.y = 0.2
    ros_req.load_carrier_compartment.box.z = 0.3
    api_req = extract_values(ros_req)
    assert_primitives(
        ros_req.load_carrier_compartment, api_req["load_carrier_compartment"]
    )

    api_res = {
        "timestamp": {"sec": 1587035449, "nsec": 321465164},
        "return_code": {"message": "abcdef", "value": 1234},
        "load_carriers": [
            {
                "pose": {
                    "position": {
                        "y": -0.0168824336375496,
                        "x": 0.1189406043995812,
                        "z": 0.8875302697155399,
                    },
                    "orientation": {
                        "y": -0.04632486703729271,
                        "x": 0.998664879978751,
                        "z": 0.006342615882086765,
                        "w": 0.02195985184108778,
                    },
                },
                "pose_frame": "camera",
                "inner_dimensions": {"y": 0.27, "x": 0.37, "z": 0.14},
                "outer_dimensions": {"y": 0.3, "x": 0.4, "z": 0.2},
                "overfilled": True,
                "rim_thickness": {"y": 0.0, "x": 0.0},
                "id": "xyz",
            }
        ],
        "items": [
            {
                "uuid": "ee0b2bc4-7fd2-11ea-a789-00142d2cd4ce",
                "pose_frame": "camera",
                "timestamp": {"sec": 1587035449, "nsec": 321465164},
                "pose": {
                    "position": {
                        "y": 0.0015526703948114742,
                        "x": 0.018541338820357283,
                        "z": 0.9062130178042246,
                    },
                    "orientation": {
                        "y": -0.29071987884727973,
                        "x": 0.9563493015403196,
                        "z": 0.019845952916433495,
                        "w": 0.02200235531039019,
                    },
                },
                "type": "RECTANGLE",
                "rectangle": {"y": 0.05698329755083764, "x": 0.10302717395647137},
            },
            {
                "uuid": "ee0b2f34-7fd2-11ea-a789-00142d2cd4ce",
                "pose_frame": "camera",
                "timestamp": {"sec": 1587035449, "nsec": 321465164},
                "pose": {
                    "position": {
                        "y": -0.07512277927447901,
                        "x": 0.01837425822827067,
                        "z": 0.9083435428699417,
                    },
                    "orientation": {
                        "y": -0.10244608240444944,
                        "x": 0.9947325521529258,
                        "z": 0.0022848846348791315,
                        "w": 0.0025940681395760523,
                    },
                },
                "type": "RECTANGLE",
                "rectangle": {"y": 0.05739744695533501, "x": 0.10506054260132827},
            },
        ],
    }
    ros_res = DetectItems.Response()
    populate_instance(api_res, ros_res)
    ros_lc = ros_res.load_carriers[0]
    api_lc = api_res["load_carriers"][0]
    assert_lc(ros_lc, api_lc, api_res["timestamp"])
    assert ros_lc.overfilled == api_lc["overfilled"]
    for i, ros_item in enumerate(ros_res.items):
        api_item = api_res["items"][i]
        assert_item(ros_item, api_item)
