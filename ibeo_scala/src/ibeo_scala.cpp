/*
 *   ibeo_scala.cpp - ROS implementation of the Ibeo ScaLa driver.
 *   Copyright (C) 2017 AutonomouStuff, Co.
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
 *   USA
 */

#include <ibeo_scala_core.h>
#include <ros_msg_converters.h>
#include <network_interface/network_interface.h>

//C++ Includes
#include <unordered_map>

//ROS includes
#include <ros/ros.h>

//PCL Includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

//Messages
#include <network_interface/TCPFrame.h>
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
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

using namespace AS;
using namespace AS::Network;
using namespace AS::Drivers::IbeoScala;

template<typename T> T encode_ros_msg(unsigned short &data_type, IbeoTxMessage &msg_class);

TCPInterface tcp_interface;

int main(int argc, char **argv)
{
  std::string ip_address = "192.168.1.52";
	int port = 12002;
	std::string frame_id = "ibeo_scala";
	bool is_fusion = false;
  unsigned char *msg_buf;
  int buf_size = IBEO_PAYLOAD_SIZE;
  std::vector<unsigned char> partial_msg;
  std::vector<std::vector<unsigned char>> messages;

	// ROS initialization
	ros::init(argc, argv, "ibeo_scala");
	ros::NodeHandle n;
	ros::NodeHandle priv("~");
	ros::Rate loop_rate(50.0);
	bool exit = false;

  //Wait for time to be valid.
  while (ros::Time::now().nsec == 0);

	if (priv.getParam("ip_address", ip_address))
	{ 
		ROS_INFO("Got ip_address: %s", ip_address.c_str());
		if (ip_address == "" )
		{
		 ROS_ERROR("IP Address Invalid");
		 exit = true;
		}
	}

	if (priv.getParam("port", port))
	{ 
		ROS_INFO("Got port: %d", port);
		if (port < 0)
		{
		 ROS_ERROR("Port Invalid");
		 exit = true;
		}
	}
  
	if (priv.getParam("is_fusion", is_fusion))
	{ 
		ROS_INFO("is Fusion ECU: %s", (is_fusion)? "true" : "false");
	}

	if (priv.getParam("sensor_frame_id", frame_id))
	{
		ROS_INFO("Got sensor frame ID: %s", frame_id.c_str());
	}

	if(exit)
    return 0;

  return_statuses status = tcp_interface.open(ip_address.c_str(), port);

  if (status == ok)
  {
    ros::Publisher eth_tx_pub = n.advertise<network_interface::TCPFrame>("tcp_tx", 10);
    ros::Publisher pointcloud_pub = n.advertise<pcl::PointCloud <pcl::PointXYZ> >("as_tx/point_cloud", 1);
    ros::Publisher object_markers_pub = n.advertise<visualization_msgs::MarkerArray>("as_tx/objects", 1);
    ros::Publisher object_contour_points_pub = n.advertise<visualization_msgs::Marker>("as_tx/object_contour_points", 1);

    ros::Publisher scan_2202_pub, scan_2205_pub, scan_2208_pub,
                   object_2225_pub, object_2270_pub, object_2271_pub, object_2280_pub,
                   camera_image_pub, vehicle_state_2805_pub, vehicle_state_2806_pub,
                   vehicle_state_2807_pub, device_status_pub;

    std::unordered_map<unsigned short, ros::Publisher*> publist;

    if (is_fusion)
    {
      scan_2205_pub = n.advertise<ibeo_scala_msgs::ScanData2205>("parsed_tx/scan_data_2205", 1);
      object_2225_pub = n.advertise<ibeo_scala_msgs::ObjectData2225>("parsed_tx/object_data_2225", 1);
      object_2280_pub = n.advertise<ibeo_scala_msgs::ObjectData2280>("parsed_tx/object_data_2280", 1);
      camera_image_pub = n.advertise<ibeo_scala_msgs::CameraImage>("parsed_tx/camera_image", 1);
      vehicle_state_2806_pub = n.advertise<ibeo_scala_msgs::HostsVehicleState2806>("parsed_tx/hosts_vehicle_state_2806", 1);
      vehicle_state_2807_pub = n.advertise<ibeo_scala_msgs::HostsVehicleState2807>("parsed_tx/hosts_vehicle_state_2807", 1);

      publist.insert(std::make_pair(0x2205, &scan_2205_pub));
      publist.insert(std::make_pair(0x2225, &object_2225_pub));
      publist.insert(std::make_pair(0x2280, &object_2280_pub));
      publist.insert(std::make_pair(0x2403, &camera_image_pub));
      publist.insert(std::make_pair(0x2806, &vehicle_state_2806_pub));
      publist.insert(std::make_pair(0x2807, &vehicle_state_2807_pub));
    }
    else
    {
      scan_2202_pub = n.advertise<ibeo_scala_msgs::ScanData2202>("parsed_tx/scan_data_2202", 1);
      scan_2208_pub = n.advertise<ibeo_scala_msgs::ScanData2208>("parsed_tx/scan_data_2208", 1);
      object_2270_pub = n.advertise<ibeo_scala_msgs::ObjectData2270>("parsed_tx/object_data_2270", 1);
      object_2271_pub = n.advertise<ibeo_scala_msgs::ObjectData2271>("parsed_tx/object_data_2271", 1);
      vehicle_state_2805_pub = n.advertise<ibeo_scala_msgs::HostsVehicleState2805>("parsed_tx/hosts_vehicle_state_2805", 1);

      publist.insert(std::make_pair(0x2202, &scan_2202_pub));
      publist.insert(std::make_pair(0x2208, &scan_2208_pub));
      publist.insert(std::make_pair(0x2270, &object_2270_pub));
      publist.insert(std::make_pair(0x2271, &object_2271_pub));
      publist.insert(std::make_pair(0x2805, &vehicle_state_2805_pub));
    }

    device_status_pub = n.advertise<ibeo_scala_msgs::DeviceStatus>("parsed_tx/device_status", 1);
    publist.insert(std::make_pair(0x6301, &device_status_pub));

    while (ros::ok())
    {
      buf_size = IBEO_PAYLOAD_SIZE;
      msg_buf = (unsigned char*) malloc(buf_size); //New allocation.

      tcp_interface.read_some(msg_buf, buf_size); //Read a (big) chunk.

      int first_mw = find_magic_word(msg_buf, buf_size);

      if (first_mw > -1)
      {
        if (first_mw > 0 && !partial_msg.empty())
        {
          //We have leftover data from last read and we found
          //a new magic word past the beginning of this read.
          //Assume that the leftover from last read and the beginning
          //of this read make a new message.

          std::vector<unsigned char> new_part_msg(partial_msg.begin(), partial_msg.end());
          new_part_msg.insert(new_part_msg.end(), msg_buf, msg_buf + first_mw);
          messages.push_back(new_part_msg);

          partial_msg.clear();
        }
          
        msg_buf = msg_buf + first_mw; //Point to the beginning of the first message in this chunk.
        buf_size -= first_mw;

        int mw_offset;
        while ((mw_offset = find_magic_word(msg_buf, buf_size)) > -1)
        {
          //Found another message in this chunk.
          std::vector<unsigned char> last_message(msg_buf, msg_buf + mw_offset);
          messages.push_back(last_message);

          msg_buf = msg_buf + mw_offset; //Point to the beginning of the next message.
          buf_size -= mw_offset; //Reduce the size of the array.
        }

        if (!messages.empty())
        {
          //Found at least one message, let's parse them.
          for(unsigned int i = 0; i < messages.size(); i++)
          {
            IbeoDataHeader ibeo_header;
            ibeo_header.parse(&(messages[i][0]));

            auto class_parser = IbeoTxMessage::make_message(ibeo_header.data_type_id); //Instantiate a class of the correct type.
            class_parser->parse(&(messages[i][0])); //Parse the raw data into the class.
            ros::Publisher* ros_publisher = publist.at(ibeo_header.data_type_id); //Get publisher.

            switch (ibeo_header.data_type_id)
            {
              case 0x2202:
              {
                auto msg = encode_2202(class_parser);
                ros_publisher->publish(msg);
              } break;
              case 0x2205:
              {
                auto msg = encode_2205(class_parser);
                ros_publisher->publish(msg);
              } break;
            }
          }

          messages.clear();
        }

        if (buf_size > 0)
        {
          //We still have data left over. Add it to the next loop.
          partial_msg.insert(partial_msg.end(), msg_buf, msg_buf + buf_size);
        }
      }

      free(msg_buf); //FREE THE BITS
    }
  }
  else
  {
    ROS_ERROR("Connection to ScaLa could not be opened");
  }

  tcp_interface.close();
  return 0;
}
