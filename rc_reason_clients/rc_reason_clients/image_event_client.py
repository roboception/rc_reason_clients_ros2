# Copyright 2026 Roboception GmbH
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

import time
import threading
import grpc
import rclpy
from rclpy.node import Node
from rc_reason_msgs.msg import ImageEvent
from std_msgs.msg import Header
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from rc_reason_clients.generated import image_interface_pb2_grpc, image_interface_pb2


def uint64_to_ros_time(timestamp):
    seconds = timestamp // 1_000_000_000
    nanos = timestamp % 1_000_000_000
    return int(seconds), int(nanos)


class ImageEventClient(Node):
    def __init__(self):
        super().__init__("rc_image_event_client")

        # Parameters
        self.declare_parameter(
            "host",
            "",
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING, read_only=True),
        )
        self.declare_parameter(
            "pipeline",
            0,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, read_only=True),
        )

        # Determine default port based on pipeline
        pipeline = self.get_parameter("pipeline").value
        default_port = 50051 + pipeline

        self.declare_parameter(
            "port",
            default_port,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, read_only=True),
        )
        self.declare_parameter(
            "reconnect_interval",
            2.0,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE),
        )

        # Event enabling parameters
        self.declare_parameter(
            "depth_acquisition_done_enabled",
            True,
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL),
        )

        self.host = self.get_parameter("host").value
        self.port = self.get_parameter("port").value
        self.reconnect_interval = self.get_parameter("reconnect_interval").value

        # Publishers
        self.pub_depth_acquisition_done = self.create_publisher(
            ImageEvent, "~/depth_acquisition_done", 10
        )

        self.running = True
        self.thread = None

    def start(self):
        self.get_logger().info(
            f"Starting ImageEventClient connecting to {self.host}:{self.port}"
        )
        self.thread = threading.Thread(target=self.run_loop, daemon=True)
        self.thread.start()

    def run_loop(self):
        target = f"{self.host}:{self.port}"

        while rclpy.ok() and self.running:
            channel = grpc.insecure_channel(target)
            stub = image_interface_pb2_grpc.ImageEventsInterfaceStub(channel)

            # Prepare request with enabled events
            depth_enabled = self.get_parameter("depth_acquisition_done_enabled").value
            req = image_interface_pb2.ImageEventsRequest(
                depth_acquisition_done_enabled=depth_enabled
            )

            try:
                self.get_logger().info(f"Connecting to {target}...")
                # Wait for the channel to be ready to provide better feedback
                grpc.channel_ready_future(channel).result(timeout=10.0)
                self.get_logger().info(f"Connected to {target}")

                stream = stub.StreamImageEvents(req)

                for event in stream:
                    if not rclpy.ok() or not self.running:
                        break

                    if event.HasField("depth_acquisition_done"):
                        da_event = event.depth_acquisition_done

                        msg = ImageEvent()

                        # Header timestamp from event
                        header = Header()
                        sec, nanosec = uint64_to_ros_time(da_event.timestamp)
                        header.stamp.sec = sec
                        header.stamp.nanosec = nanosec
                        header.frame_id = "rc_visard"
                        msg.header = header

                        # Imageset timestamp
                        sec, nanosec = uint64_to_ros_time(event.imageset_timestamp)
                        msg.imageset_stamp.sec = sec
                        msg.imageset_stamp.nanosec = nanosec

                        self.pub_depth_acquisition_done.publish(msg)

            except grpc.RpcError as e:
                if self.running:
                    self.get_logger().warn(
                        f"gRPC connection error: {e.code()} {e.details()}"
                    )
                    time.sleep(self.reconnect_interval)
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"Unexpected error: {e}")
                    time.sleep(self.reconnect_interval)
            finally:
                channel.close()

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageEventClient()

    host = node.get_parameter("host").value
    if not host:
        node.get_logger().error("host is not set")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
