#include <ros_msg_handler.h>

RosMsgHandler::RosMsgHandler(unsigned short type_id, ros::Publisher* pub) :
  type_id(type_id),
  pub(pub)
{
}

void RosMsgHandler::encode_and_publish(IbeoTxMessage* parser_class)
{
  switch (type_id)
  {
    case 0x2202:
    {
      ibeo_scala_msgs::ScanData2202 new_msg;
      encode_2202((ScanData2202*)parser_class, &new_msg);
      pub->publish(new_msg);
    } break;
    case 0x2205:
    {
      ibeo_scala_msgs::ScanData2205 new_msg;
      encode_2205((ScanData2205*)parser_class, &new_msg);
      pub->publish(new_msg);
    } break;
    case 0x2208:
    {
      ibeo_scala_msgs::ScanData2208 new_msg;
      encode_2208((ScanData2208*)parser_class, &new_msg);
      pub->publish(new_msg);
    } break;
  }
}

void RosMsgHandler::encode_2202(ScanData2202* parser_class, ibeo_scala_msgs::ScanData2202* new_msg)
{
  //Do some stuff here.
}

void RosMsgHandler::encode_2205(ScanData2205* parser_class, ibeo_scala_msgs::ScanData2205* new_msg)
{
  //Do some stuff here.
}

void RosMsgHandler::encode_2208(ScanData2208* parser_class, ibeo_scala_msgs::ScanData2208* new_msg)
{
  //Do some stuff here.
}
