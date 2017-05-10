#include <ibeo_ros_msg_handler.h>

IbeoRosMsgHandler::IbeoRosMsgHandler(unsigned short type_id, ros::Publisher &pub) :
  type_id(type_id),
  pub(pub)
{
}

void IbeoRosMsgHandler::encode_and_publish(IbeoTxMessage* parser_class)
{
  switch (type_id)
  {
    case 0x2202:
    {
      ibeo_scala_msgs::ScanData2202 new_msg;
      encode_2202((ScanData2202*)parser_class, new_msg);
      pub.publish(new_msg);
    } break;
    case 0x2205:
    {
      ibeo_scala_msgs::ScanData2205 new_msg;
      encode_2205((ScanData2205*)parser_class, new_msg);
      pub.publish(new_msg);
    } break;
    case 0x2208:
    {
      ibeo_scala_msgs::ScanData2208 new_msg;
      encode_2208((ScanData2208*)parser_class, new_msg);
      pub.publish(new_msg);
    } break;
    case 0x2225:
    {
      ibeo_scala_msgs::ObjectData2225 new_msg;
      encode_2225((ObjectData2225*)parser_class, new_msg);
      pub.publish(new_msg);
    } break;
    case 0x2270:
    {
      ibeo_scala_msgs::ObjectData2270 new_msg;
      encode_2270((ObjectData2270*)parser_class, new_msg);
      pub.publish(new_msg);
    } break;
    case 0x2271:
    {
      ibeo_scala_msgs::ObjectData2271 new_msg;
      encode_2271((ObjectData2271*)parser_class, new_msg);
      pub.publish(new_msg);
    } break;
    case 0x2280:
    {
      ibeo_scala_msgs::ObjectData2280 new_msg;
      encode_2280((ObjectData2280*)parser_class, new_msg);
      pub.publish(new_msg);
    } break;
    case 0x2403:
    {
      ibeo_scala_msgs::CameraImage new_msg;
      encode_2403((CameraImage*)parser_class, new_msg);
      pub.publish(new_msg);
    } break;
    case 0x2805:
    {
      ibeo_scala_msgs::HostsVehicleState2805 new_msg;
      encode_2805((HostsVehicleState2805*)parser_class, new_msg);
      pub.publish(new_msg);
    } break;
    case 0x2806:
    {
      ibeo_scala_msgs::HostsVehicleState2806 new_msg;
      encode_2806((HostsVehicleState2806*)parser_class, new_msg);
      pub.publish(new_msg);
    } break;
    case 0x2807:
    {
      ibeo_scala_msgs::HostsVehicleState2807 new_msg;
      encode_2807((HostsVehicleState2807*)parser_class, new_msg);
      pub.publish(new_msg);
    } break;
    case 0x6301:
    {
      ibeo_scala_msgs::DeviceStatus new_msg;
      encode_6301((DeviceStatus*)parser_class, new_msg);
      pub.publish(new_msg);
    } break;
  }
}

ros::Time IbeoRosMsgHandler::ntp_to_ros_time(NTPTime time)
{
  return ros::Time(((time & 0xFFFF0000) >> 32), (time & 0x0000FFFF));
}

void IbeoRosMsgHandler::encode_ibeo_header(IbeoDataHeader &class_header, ibeo_scala_msgs::IbeoDataHeader &msg_header)
{
  msg_header.previous_message_size = class_header.previous_message_size;
  msg_header.message_size = class_header.message_size;
  msg_header.device_id = class_header.device_id;
  msg_header.data_type_id = class_header.data_type_id;
  msg_header.stamp = ntp_to_ros_time(class_header.time);
}

void IbeoRosMsgHandler::encode_2202(ScanData2202* parser_class, ibeo_scala_msgs::ScanData2202 &new_msg)
{
  encode_ibeo_header(parser_class->ibeo_header, new_msg.ibeo_header);
}

void IbeoRosMsgHandler::encode_2205(ScanData2205* parser_class, ibeo_scala_msgs::ScanData2205 &new_msg)
{
  encode_ibeo_header(parser_class->ibeo_header, new_msg.ibeo_header);
}

void IbeoRosMsgHandler::encode_2208(ScanData2208* parser_class, ibeo_scala_msgs::ScanData2208 &new_msg)
{
  encode_ibeo_header(parser_class->ibeo_header, new_msg.ibeo_header);
}

void IbeoRosMsgHandler::encode_2225(ObjectData2225* parser_class, ibeo_scala_msgs::ObjectData2225 &new_msg)
{
  encode_ibeo_header(parser_class->ibeo_header, new_msg.ibeo_header);
}

void IbeoRosMsgHandler::encode_2270(ObjectData2270* parser_class, ibeo_scala_msgs::ObjectData2270 &new_msg)
{
  encode_ibeo_header(parser_class->ibeo_header, new_msg.ibeo_header);
}

void IbeoRosMsgHandler::encode_2271(ObjectData2271* parser_class, ibeo_scala_msgs::ObjectData2271 &new_msg)
{
  encode_ibeo_header(parser_class->ibeo_header, new_msg.ibeo_header);
}

void IbeoRosMsgHandler::encode_2280(ObjectData2280* parser_class, ibeo_scala_msgs::ObjectData2280 &new_msg)
{
  encode_ibeo_header(parser_class->ibeo_header, new_msg.ibeo_header);
}

void IbeoRosMsgHandler::encode_2403(CameraImage* parser_class, ibeo_scala_msgs::CameraImage &new_msg)
{
  encode_ibeo_header(parser_class->ibeo_header, new_msg.ibeo_header);
}

void IbeoRosMsgHandler::encode_2805(HostsVehicleState2805* parser_class, ibeo_scala_msgs::HostsVehicleState2805 &new_msg)
{
  encode_ibeo_header(parser_class->ibeo_header, new_msg.ibeo_header);
}

void IbeoRosMsgHandler::encode_2806(HostsVehicleState2806* parser_class, ibeo_scala_msgs::HostsVehicleState2806 &new_msg)
{
  encode_ibeo_header(parser_class->ibeo_header, new_msg.ibeo_header);
}

void IbeoRosMsgHandler::encode_2807(HostsVehicleState2807* parser_class, ibeo_scala_msgs::HostsVehicleState2807 &new_msg)
{
  encode_ibeo_header(parser_class->ibeo_header, new_msg.ibeo_header);
}

void IbeoRosMsgHandler::encode_6301(DeviceStatus* parser_class, ibeo_scala_msgs::DeviceStatus &new_msg)
{
  encode_ibeo_header(parser_class->ibeo_header, new_msg.ibeo_header);
}
