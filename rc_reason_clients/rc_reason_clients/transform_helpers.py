# Copyright 2021 Roboception GmbH
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

from geometry_msgs.msg import TransformStamped

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA


def load_carrier_to_tf(lc, postfix):
    tf = TransformStamped()
    tf.header.frame_id = lc.pose.header.frame_id
    tf.child_frame_id = "lc_{}".format(postfix)
    tf.header.stamp = lc.pose.header.stamp
    tf.transform.translation.x = lc.pose.pose.position.x
    tf.transform.translation.y = lc.pose.pose.position.y
    tf.transform.translation.z = lc.pose.pose.position.z
    tf.transform.rotation = lc.pose.pose.orientation
    return tf


def lc_to_marker(lc, lc_no, ns):
    m = Marker(action=Marker.ADD, type=Marker.CUBE)
    m.color = ColorRGBA(r=0.0, g=0.2, b=0.8, a=0.3)
    m.header = lc.pose.header
    m.ns = ns

    # FIXME: calculate actual bottom and sides
    m.id = lc_no
    m.pose = lc.pose.pose
    m.scale.x = lc.outer_dimensions.x
    m.scale.y = lc.outer_dimensions.y
    m.scale.z = lc.outer_dimensions.z

    return m

def match_to_tf(match):
    tf = TransformStamped()
    tf.header.frame_id = match.pose.header.frame_id
    tf.child_frame_id = "{}_{}".format(match.template_id, match.uuid)
    tf.header.stamp = match.pose.header.stamp
    tf.transform.translation.x = match.pose.pose.position.x
    tf.transform.translation.y = match.pose.pose.position.y
    tf.transform.translation.z = match.pose.pose.position.z
    tf.transform.rotation = match.pose.pose.orientation
    return tf
