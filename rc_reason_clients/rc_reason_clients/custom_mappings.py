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

import copy


def map_api2ros(msg, rostype):
    """ Map an API msg to ROS """
    if rostype == 'rc_reason_msgs/DetectedTag':
        new_msg = {}
        header = {'stamp': msg['timestamp'], 'frame_id': msg['pose_frame']}
        new_msg['header'] = header
        new_msg['tag'] = {'id': msg['id'], 'size': msg['size']}
        new_msg['pose'] = {'pose': msg['pose'], 'header': header}
        new_msg['instance_id'] = msg['instance_id']
        return new_msg
    elif rostype == 'shape_msgs/Plane':
        return {'coef': [msg['normal']['x'], msg['normal']['y'], msg['normal']['z'], msg['distance']]}
    elif rostype in ['rc_reason_msgs/GetBasePlaneCalibration_Response', 'rc_reason_msgs/CalibrateBasePlane_Response']:
        new_msg = copy.deepcopy(msg)
        new_msg['pose_frame'] = msg['plane']['pose_frame']
        del new_msg['plane']['pose_frame']
        return new_msg
    elif rostype == 'rc_reason_msgs/LoadCarrier':
        new_msg = {}
        for key in msg:
            if key not in ['pose', 'pose_frame']:
                new_msg[key] = msg[key]
        header = {'frame_id': msg['pose_frame']}
        new_msg['pose'] = {'pose': msg['pose'], 'header': header}
        return new_msg
    elif rostype == 'rc_reason_msgs/RegionOfInterest3D':
        new_msg = {}
        new_msg['id'] = msg['id']
        header = {'frame_id': msg['pose_frame']}
        new_msg['pose'] = {'pose': msg['pose'], 'header': header}
        if msg['type'] == 'BOX':
            new_msg['primitive'] = {'type': 1, 'dimensions': [msg['box']['x'], msg['box']['y'], msg['box']['z']]}
        elif msg['type'] == 'SPHERE':
            new_msg['primitive'] = {'type': 2, 'dimensions': [msg['sphere']['radius']]}
        return new_msg


    return msg

def map_ros2api(msg, rostype):
    """ Map a ROS msg to API """
    if rostype == 'shape_msgs/Plane':
        c = msg['coef']
        return {'normal': {'x':c[0], 'y': c[1], 'z': c[2]}, 'distance': c[3]}
    elif rostype == 'rc_reason_msgs/CalibrateBasePlane_Request':
        new_msg = copy.deepcopy(msg)
        if msg['plane_estimation_method'] == 'STEREO':
            new_msg['stereo'] = {'plane_preference': msg['stereo_plane_preference']}
            del new_msg['stereo_plane_preference']
        return new_msg
    elif rostype == 'rc_reason_msgs/LoadCarrier':
        new_msg = {}
        # never send overfilled flag
        for key in msg:
            if key not in ['pose', 'overfilled']:
                new_msg[key] = msg[key]
        # map PoseStamped to pose and pose_frame
        new_msg['pose'] = msg['pose']['pose']
        new_msg['pose_frame'] = msg['pose']['header']['frame_id']
        return new_msg
    elif rostype == 'rc_reason_msgs/DetectLoadCarriers_Request':
        new_msg = copy.deepcopy(msg)
        # don't send robot pose if not external
        if msg['pose_frame'] != 'external':
            del new_msg['robot_pose']
        return new_msg
    elif rostype == 'rc_reason_msgs/RegionOfInterest3D':
        new_msg = {}
        new_msg['id'] = msg['id']
        new_msg['pose'] = msg['pose']['pose']
        new_msg['pose_frame'] = msg['pose']['header']['frame_id']
        d = msg['primitive']['dimensions']
        if msg['primitive']['type'] == 1:
            new_msg['type'] = 'BOX'
            new_msg['box'] = {'x': d[0], 'y': d[1], 'z': d[2]}
        elif msg['primitive']['type'] == 2:
            new_msg['type'] = 'SPHERE'
            new_msg['sphere'] = {'radius': d[0]}
        return new_msg

    # no mapping required, return generated one
    return msg
