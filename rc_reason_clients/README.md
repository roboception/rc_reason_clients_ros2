rc_reason_clients for ROS2
==========================

This package provides ROS2 client nodes that interface with Roboception devices like the [rc_visard](https://roboception.com/rc_visard) 3D sensor.

Please consult the manual for detailed descriptions of parameters and services: https://doc.rc-visard.com).

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

Example for eloquent:

```
ros2 run rc_reason_clients rc_april_tag_detect_client --ros-args --param host:=10.0.2.40
```

rc_april_tag_detect_client and rc_qr_code_detect_client
-------------------------------------------------------

Clients to interface with TagDetect (AprilTag and QRCode detection) running on the device.
See the [TagDetect documentation](https://doc.rc-visard.com/latest/en/tagdetect.html) for details.

[TagDetect parameters](https://doc.rc-visard.com/latest/en/tagdetect.html#parameters)

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

[SilhouetteMatch Services](https://doc.rc-visard.com/latest/en/silhouettematch.html#services)

To run the client:
`ros2 run rc_reason_clients rc_silhouettematch_client --ros-args --param host:=10.0.2.40`

rc_itempick_client
------------------

Client to interface with ItemPick running on the device.
See the [ItemPick documentation](https://doc.rc-visard.com/latest/en/itempick.html) for details.

[ItemPick parameters](https://doc.rc-visard.com/latest/en/itempick.html#parameters)

[ItemPick services](https://doc.rc-visard.com/latest/en/itempick.html#services)

To run the client:
`ros2 run rc_reason_clients rc_silhouettematch_client --ros-args --param host:=10.0.2.40`

rc_boxpick_client
-----------------

Client to interface with BoxPick running on the device.
See the [BoxPick documentation](https://doc.rc-visard.com/latest/en/itempick.html) for details.

[BoxPick parameters](https://doc.rc-visard.com/latest/en/itempick.html#parameters)

[BoxPick services](https://doc.rc-visard.com/latest/en/itempick.html#services)

To run the client:
`ros2 run rc_reason_clients rc_silhouettematch_client --ros-args --param host:=10.0.2.40`
