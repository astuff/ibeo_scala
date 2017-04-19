Ibeo ScaLa
==========

The Ibeo ScaLa is a multi-plane laser scanner with embedded object tracking and classification.  See this page for more information: http://www.autonomoustuff.com/product/ibeo-scala/

This repository contains a low-level driver that can connect to a ScaLa via a TCP connection and deserialize data into data structures and a ROS nodelet that uses the library to publish that information.

Packages
--------

`ibeo_scala`
- The ROS node.  It uses the network_interface library to connect to a ScaLa or a ScaLa ECU and parses the messages using the core_ibeo_scala code. It then publishes received data as ROS messages.

`ibeo_scala_msgs`
- ROS messages capable of representing all of the data received from a ScaLa.

Usage
-----

`ibeo_scala`
- Create an instance of the `ibeo_scala` node.  See [ibeo_scala.launch](/ibeo_scala/launch/ibeo_scala.launch) for an example.

Node Params
-----------
`frame_id`
- The ID of the TF frame that will be placed in the `sensor_msgs/PointCloud2` header.  Default: `scala`

`host`
- The hostname or IP address of the ScaLa device or ECU.  Default: `192.168.0.100`

`port`
- The TCP port of the ScaLa device or ECU.  Default: `12002`

`publish_raw_data`
- If this is `True`, the node will publish `network_interface/TCPFrame` messages containing the raw data that it sends to and receives from the ScaLa.  This will increase CPU usage noticeably.  Default: `False`

Publications
------------

`eth_tx` : `network_interface/TCPFrame`
- Raw ethernet data transmitted from the device.  Only published if the `publish_raw_data` param is `True`.

`eth_rx_echo` : `network_interface/TCPFrame`
- Raw ethernet data transmitted to the device.  Only published if the `publish_raw_data` param is `True`.

`parsed_tx/camera_image` : `ibeo_scala_msgs/CameraImage`
- Camera images received from the ECU.

`parsed_tx/device_status` : `ibeo_scala_msgs/DeviceStatus`
- DeviceStatus messages received from the ECU.

`parsed_tx/ecu_object_data_2225` : `ibeo_scala_msgs/EcuObjectData2225`

`parsed_tx/ecu_object_data_2280` : `ibeo_scala_msgs/EcuObjectData2280`
- Objects tracked by an ECU.

`parsed_tx/ecu_scan_data` : `ibeo_scala_msgs/EcuScanData`
- Points scanned by an ECU.

`parsed_tx/object_data_2270` : `ibeo_scala_msgs/ObjectData2270`

`parsed_tx/object_data_2271` : `ibeo_scala_msgs/ObjectData2271`
- Objects tracked by a ScaLa device.

`parsed_tx/scan_data_2202` : `ibeo_scala_msgs/ScanData2202`

`parsed_tx/scan_data_2205` : `ibeo_scala_msgs/ScanData2205`

`parsed_tx/scan_data_2208` : `ibeo_scala_msgs/ScanData2208`
- Points scanned by a ScaLa device.

`parsed_tx/vehicle_state_2805` : `ibeo_scala_msgs/HostVehicleState2805`

`parsed_tx/vehicle_state_2806` : `ibeo_scala_msgs/HostVehicleState2806`

`parsed_tx/vehicle_state_2807` : `ibeo_scala_msgs/HostVehicleState2807`
- Vehicle state produced by an ECU.

`as_tx/point_cloud` : `sensor_msgs/PointCloud2`
- Points received via `EcuScanData`, `ScanData2202`, `ScanData2205` and `ScanData2208` messages from a ScaLa or a ScaLa ECU.

Subscriptions
-------------

`eth_rx` : `network_interface/TCPFrame`
- Any raw ethernet data published to this topic will be forwarded to the ScaLa or ScaLa ECU.
