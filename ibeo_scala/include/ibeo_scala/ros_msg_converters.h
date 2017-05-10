#ifndef ROS_MSG_CONVERTERS_H
#define ROS_MSG_CONVERTERS_H

#include <ibeo_scala_core.h>
#include <ibeo_scala_msgs/ScanData2202.h>
#include <ibeo_scala_msgs/ScanData2205.h>
#include <ibeo_scala_msgs/ScanData2208.h>
#include <ibeo_scala_msgs/ObjectData2225.h>
#include <ibeo_scala_msgs/ObjectData2270.h>
#include <ibeo_scala_msgs/ObjectData2271.h>
#include <ibeo_scala_msgs/ObjectData2280.h>
#include <ibeo_scala_msgs/CameraImage.h>
#include <ibeo_scala_msgs/HostsVehicleState2805.h>
#include <ibeo_scala_msgs/HostsVehicleState2806.h>
#include <ibeo_scala_msgs/HostsVehicleState2807.h>
#include <ibeo_scala_msgs/DeviceStatus.h>

namespace AS
{
namespace Drivers
{
namespace IbeoScala
{

ibeo_scala_msgs::ScanData2202 encode_2202(IbeoTxMessage* msg_class);
ibeo_scala_msgs::ScanData2205 encode_2205(IbeoTxMessage* msg_class);
ibeo_scala_msgs::ScanData2208 encode_2208(IbeoTxMessage* msg_class);
ibeo_scala_msgs::ObjectData2225 encode_2225(IbeoTxMessage* msg_class);
ibeo_scala_msgs::ObjectData2270 encode_2270(IbeoTxMessage* msg_class);
ibeo_scala_msgs::ObjectData2271 encode_2271(IbeoTxMessage* msg_class);
ibeo_scala_msgs::ObjectData2280 encode_2280(IbeoTxMessage* msg_class);
ibeo_scala_msgs::CameraImage encode_2403(IbeoTxMessage* msg_class);
ibeo_scala_msgs::HostsVehicleState2805 encode_2805(IbeoTxMessage* msg_class);
ibeo_scala_msgs::HostsVehicleState2806 encode_2806(IbeoTxMessage* msg_class);
ibeo_scala_msgs::HostsVehicleState2807 encode_2807(IbeoTxMessage* msg_class);
ibeo_scala_msgs::DeviceStatus encode_6301(IbeoTxMessage* msg_class);

} //namespace IbeoScala
} //namespace Drivers
} //namespace AS

#endif
