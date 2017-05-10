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

  new_msg.scan_number = parser_class->scan_number;
  new_msg.scanner_status = parser_class->scanner_status;
  new_msg.sync_phase_offset = parser_class->sync_phase_offset;
  new_msg.scan_start_time = ntp_to_ros_time(parser_class->scan_start_time);
  new_msg.scan_end_time = ntp_to_ros_time(parser_class->scan_end_time);
  new_msg.angle_ticks_per_rotation = parser_class->angle_ticks_per_rotation;
  new_msg.start_angle_ticks = parser_class->start_angle_ticks;
  new_msg.end_angle_ticks = parser_class->end_angle_ticks;
  new_msg.mounting_yaw_angle_ticks = parser_class->mounting_yaw_angle_ticks;
  new_msg.mounting_pitch_angle_ticks = parser_class->mounting_pitch_angle_ticks;
  new_msg.mounting_roll_angle_ticks = parser_class->mounting_roll_angle_ticks;
  new_msg.mounting_position_x = parser_class->mounting_position_x;
  new_msg.mounting_position_y = parser_class->mounting_position_y;
  new_msg.mounting_position_z = parser_class->mounting_position_z;

  for (auto scan_point : parser_class->scan_point_list)
  {
    ibeo_scala_msgs::ScanPoint2202 scan_point_msg;

    scan_point_msg.layer = scan_point.layer;
    scan_point_msg.echo = scan_point.echo;
    scan_point_msg.transparent_point = scan_point.transparent_point;
    scan_point_msg.clutter_atmospheric = scan_point.clutter_atmospheric;
    scan_point_msg.ground = scan_point.ground;
    scan_point_msg.dirt = scan_point.dirt;
    scan_point_msg.horizontal_angle = scan_point.horizontal_angle;
    scan_point_msg.radial_distance = scan_point.radial_distance;
    scan_point_msg.echo_pulse_width = scan_point.echo_pulse_width;

    new_msg.scan_point_list.push_back(scan_point_msg);
  }
}

void IbeoRosMsgHandler::encode_2205(ScanData2205* parser_class, ibeo_scala_msgs::ScanData2205 &new_msg)
{
  encode_ibeo_header(parser_class->ibeo_header, new_msg.ibeo_header);

  new_msg.scan_start_time = ntp_to_ros_time(parser_class->scan_start_time);
  new_msg.scan_end_time_offset = parser_class->scan_end_time_offset;
  new_msg.fused_scan = parser_class->fused_scan;

  if (parser_class->mirror_side == FRONT)
    new_msg.mirror_side = ibeo_scala_msgs::ScanData2205::FRONT;
  else
    new_msg.mirror_side = ibeo_scala_msgs::ScanData2205::REAR;

  if (parser_class->coordinate_system == SCANNER)
    new_msg.coordinate_system = ibeo_scala_msgs::ScanData2205::SCANNER;
  else
    new_msg.coordinate_system = ibeo_scala_msgs::ScanData2205::VEHICLE;

  new_msg.scan_number = parser_class->scan_number;
  new_msg.scan_points = parser_class->scan_points;
  new_msg.number_of_scanner_infos = parser_class->number_of_scanner_infos;

  for (auto scanner_info : parser_class->scanner_info_list)
  {
    ibeo_scala_msgs::ScannerInfo scanner_info_msg;

    scanner_info_msg.device_id = scanner_info.device_id;
    scanner_info_msg.scanner_type = scanner_info.scanner_type;
    scanner_info_msg.scan_number = scanner_info.scan_number;
    scanner_info_msg.start_angle = scanner_info.start_angle;
    scanner_info_msg.end_angle = scanner_info.end_angle;
    scanner_info_msg.scan_start_time = ntp_to_ros_time(scanner_info.scan_start_time);
    scanner_info_msg.scan_end_time = ntp_to_ros_time(scanner_info.scan_end_time);
    scanner_info_msg.scan_start_time_from_device = ntp_to_ros_time(scanner_info.scan_start_time_from_device);
    scanner_info_msg.scan_end_time_from_device = ntp_to_ros_time(scanner_info.scan_end_time_from_device);
    scanner_info_msg.scan_frequency = scanner_info.scan_frequency;
    scanner_info_msg.beam_tilt = scanner_info.beam_tilt;
    scanner_info_msg.scan_flags = scanner_info.scan_flags;

    scanner_info_msg.mounting_position.yaw_angle = scanner_info.mounting_position.yaw_angle;
    scanner_info_msg.mounting_position.pitch_angle = scanner_info.mounting_position.pitch_angle;
    scanner_info_msg.mounting_position.roll_angle = scanner_info.mounting_position.roll_angle;
    scanner_info_msg.mounting_position.x_position = scanner_info.mounting_position.x_position;
    scanner_info_msg.mounting_position.y_position = scanner_info.mounting_position.y_position;
    scanner_info_msg.mounting_position.z_position = scanner_info.mounting_position.z_position;

    for (int i = 0; i < 8; i++)
    {
      scanner_info_msg.resolutions[i].resolution_start_angle = scanner_info.resolutions[i].resolution_start_angle;
      scanner_info_msg.resolutions[i].resolution = scanner_info.resolutions[i].resolution;
    }

    new_msg.scanner_info_list.push_back(scanner_info_msg);
  }

  for (auto scan_point : parser_class->scan_point_list)
  {
    ibeo_scala_msgs::ScanPoint2205 scan_point_msg;

    scan_point_msg.x_position = scan_point.x_position;
    scan_point_msg.y_position = scan_point.y_position;
    scan_point_msg.z_position = scan_point.z_position;
    scan_point_msg.echo_width = scan_point.echo_width;
    scan_point_msg.device_id = scan_point.device_id;
    scan_point_msg.layer = scan_point.layer;
    scan_point_msg.echo = scan_point.echo;
    scan_point_msg.time_offset = scan_point.time_offset;
    scan_point_msg.ground = scan_point.ground;
    scan_point_msg.dirt = scan_point.dirt;
    scan_point_msg.precipitation = scan_point.precipitation;
    scan_point_msg.transparent = scan_point.transparent;

    new_msg.scan_point_list.push_back(scan_point_msg);
  }
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
