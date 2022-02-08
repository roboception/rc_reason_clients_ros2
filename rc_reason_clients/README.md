rc_reason_clients for ROS2
==========================

This package provides ROS2 client nodes that interface with Roboception devices like the [rc_visard](https://roboception.com/rc_visard) 3D sensor and [rc_cube](https://roboception.com/rc_cube).

Please consult the manuals for detailed descriptions of parameters and services:
* https://doc.rc-visard.com
* https://doc.rc-cube.com

These rc_reason client nodes communicate with the device via REST-API and make the functionality available in a ROS2 native way:

* automatically provide all parameters as ROS2 parameters
* provide ROS2 services

Building
--------

Create a ROS2 workspace, clone the repository and build with `colcon build --symlink-install`.

Running
--------

The parameters and services of each client have the same names as in the REST-API (see [documentation](https://doc.rc-visard.com)).

Additionally every client has a `host` parameter which needs to be set to the IP address or hostname of the device (e.g. rc_visard).

Example:

```
ros2 run rc_reason_clients rc_april_tag_detect_client --ros-args --param host:=10.0.2.40
```

rc_april_tag_detect_client and rc_qr_code_detect_client
-------------------------------------------------------

Clients to interface with TagDetect (AprilTag and QRCode detection) running on the device.
See the [TagDetect documentation](https://doc.rc-visard.com/latest/en/tagdetect.html) for details.

[TagDetect parameters](https://doc.rc-visard.com/latest/en/tagdetect.html#parameters)

The clients have additional parameters to enable publishing of detected tags on `/tf` or as markers.
The child_frame_id is set to `<tagId>_<instanceId>`.

* `publish_tf`: Publish detected tags on tf (default: True)
* `publish_markers`: Publish detected tags as visualization markers (default: True)

[TagDetect services](https://doc.rc-visard.com/latest/en/tagdetect.html#services)

For AprilTag detection:

`ros2 run rc_reason_clients rc_april_tag_detect_client --ros-args --param host:=10.0.2.40`

For QRCode detection:

`ros2 run rc_reason_clients rc_qr_code_detect_client --ros-args --param host:=10.0.2.40`

rc_silhouettematch_client
-------------------------

Client to interface with SilhouetteMatch running on the device.
See the [SilhouetteMatch documentation](https://doc.rc-visard.com/latest/en/silhouettematch.html) for details.

[SilhouetteMatch parameters](https://doc.rc-visard.com/latest/en/silhouettematch.html#parameters)

The client has a additional parameters to enable publishing of detected instances on `/tf` or the base plane as marker.
The child_frame_id is set to `<templateId>_<instanceId>`.

* `publish_tf`: Publish detected instances on tf (default: True)
* `publish_markers`: Publish base plane as visualization marker (default: True)

[SilhouetteMatch Services](https://doc.rc-visard.com/latest/en/silhouettematch.html#services)

To run the client:
`ros2 run rc_reason_clients rc_silhouettematch_client --ros-args --param host:=10.0.2.40`

rc_itempick_client
------------------

Client to interface with ItemPick running on the device.
See the [ItemPick documentation](https://doc.rc-visard.com/latest/en/itempick.html) for details.

[ItemPick parameters](https://doc.rc-visard.com/latest/en/itempick.html#parameters)

The client has an additional parameters to enable publishing of detected load carriers and grasps on `/tf` or as markers:

* `publish_tf`: Publish detected instances on tf (default: True)
* `publish_markers`: Publish detected instances as visualization markers (default: True)

[ItemPick services](https://doc.rc-visard.com/latest/en/itempick.html#services)

To run the client:
`ros2 run rc_reason_clients rc_itempick_client --ros-args --param host:=10.0.2.40`

rc_boxpick_client
-----------------

Client to interface with BoxPick running on the device.
See the [BoxPick documentation](https://doc.rc-visard.com/latest/en/itempick.html) for details.

[BoxPick parameters](https://doc.rc-visard.com/latest/en/itempick.html#parameters)

The client has an additional parameters to enable publishing of detected load carriers, grasps and items on `/tf` or as markers:

* `publish_tf`: Publish detected instances on tf (default: True)
* `publish_markers`: Publish detected instances as visualization markers (default: True)

[BoxPick services](https://doc.rc-visard.com/latest/en/itempick.html#services)

To run the client:
`ros2 run rc_reason_clients rc_boxpick_client --ros-args --param host:=10.0.2.40`

rc_hand_eye_calibration_client
------------------------------

Client to interface with HandEyeCalibration running on the device.
The hand-eye calibration is published via TF2 (on `/tf_static`) at startup and when a new calibration is performed or requested.

See the [HandEyeCalibration documentation](https://doc.rc-visard.com/latest/en/handeye_calibration.html) for details.

[HandEyeCalibration parameters](https://doc.rc-visard.com/latest/en/handeye_calibration.html#parameters)

The client has the additional parameters for publishing via TF2:

* `camera_frame_id`: Name of the frame on the camera (default: "camera")
* `end_effector_frame_id`: Name of the frame calibrated to when using a robot_mounted camera (default: "end_effector")
* `base_frame_id`: Name of the frame calibrated to when using a statically (externally) mounted camera (default: "base_link")

[HandEyeCalibration services](https://doc.rc-visard.com/latest/en/handeye_calibration.html#services)

To run the client:
`ros2 run rc_reason_clients rc_hand_eye_calibration_client --ros-args --param host:=10.0.2.40`

rc_load_carrier_client
----------------------

Client to interface with LoadCarrier detection and RegionOfInterest configuration running on the device.

See the [LoadCarrier documentation](https://doc.rc-visard.com/latest/en/loadcarrier.html) for details.

[LoadCarrier parameters](https://doc.rc-visard.com/latest/en/loadcarrier.html#parameters)

The client has an additional parameters to enable publishing of detected load carriers on `/tf` or as markers:

* `publish_tf`: Publish detected load carriers on tf (default: True)
* `publish_markers`: Publish detected load_carriers as visualization markers (default: True)

[LoadCarrier services](https://doc.rc-visard.com/latest/en/loadcarrier.html#services)

To run the client:
`ros2 run rc_reason_clients rc_load_carrier_client --ros-args --param host:=10.0.2.40`

rc_cadmatch_client
------------------

CADMatch is only available for the rc_cube.

Client to interface with CADMatch on the rc_cube.

See the [CADMatch documentation](https://doc.rc-cube.com/latest/en/cadmatch.html) for details.

[CADMatch parameters](https://doc.rc-cube.com/latest/en/cadmatch.html#parameters)

The client has an additional parameters to enable publishing of detected load carriers on `/tf` or as markers:

* `publish_tf`: Publish detected load carriers on tf (default: True)
* `publish_markers`: Publish detected load_carriers as visualization markers (default: True)

[CADMatch services](https://doc.rc-cube.com/latest/en/cadmatch.html#services)

To run the client:
`ros2 run rc_reason_clients rc_cadmatch_client --ros-args --param host:=10.0.2.40`
