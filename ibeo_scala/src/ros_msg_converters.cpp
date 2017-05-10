#include <ros_msg_converters.h>

namespace AS
{
namespace Drivers
{
namespace IbeoScala
{

ibeo_scala_msgs::ScanData2202 encode_2202(IbeoTxMessage* msg_class)
{
  ibeo_scala_msgs::ScanData2202 new_msg;
  auto derived_msg_class = dynamic_cast<ScanData2202*>(msg_class);

  return new_msg;
}

ibeo_scala_msgs::ScanData2205 encode_2205(IbeoTxMessage* msg_class)
{
  ibeo_scala_msgs::ScanData2205 new_msg;
  return new_msg;
}

ibeo_scala_msgs::ScanData2208 encode_2208(IbeoTxMessage* msg_class)
{
  ibeo_scala_msgs::ScanData2208 new_msg;
  return new_msg;
}

ibeo_scala_msgs::ObjectData2225 encode_2225(IbeoTxMessage* msg_class)
{
  ibeo_scala_msgs::ObjectData2225 new_msg;
  return new_msg;
}

ibeo_scala_msgs::ObjectData2270 encode_2270(IbeoTxMessage* msg_class)
{
  ibeo_scala_msgs::ObjectData2270 new_msg;
  return new_msg;
}

ibeo_scala_msgs::ObjectData2271 encode_2271(IbeoTxMessage* msg_class)
{
  ibeo_scala_msgs::ObjectData2271 new_msg;
  return new_msg;
}

ibeo_scala_msgs::ObjectData2280 encode_2280(IbeoTxMessage* msg_class)
{
  ibeo_scala_msgs::ObjectData2280 new_msg;
  return new_msg;
}

ibeo_scala_msgs::CameraImage encode_2403(IbeoTxMessage* msg_class)
{
  ibeo_scala_msgs::CameraImage new_msg;
  return new_msg;
}

ibeo_scala_msgs::HostsVehicleState2805 encode_2805(IbeoTxMessage* msg_class)
{
  ibeo_scala_msgs::HostsVehicleState2805 new_msg;
  return new_msg;
}

ibeo_scala_msgs::HostsVehicleState2806 encode_2806(IbeoTxMessage* msg_class)
{
  ibeo_scala_msgs::HostsVehicleState2806 new_msg;
  return new_msg;
}

ibeo_scala_msgs::HostsVehicleState2807 encode_2807(IbeoTxMessage* msg_class)
{
  ibeo_scala_msgs::HostsVehicleState2807 new_msg;
  return new_msg;
}

ibeo_scala_msgs::DeviceStatus encode_6301(IbeoTxMessage* msg_class)
{
  ibeo_scala_msgs::DeviceStatus new_msg;
  return new_msg;
}

}
}
}
