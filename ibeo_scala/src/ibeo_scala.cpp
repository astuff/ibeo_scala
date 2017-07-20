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

#include <ibeo_ros_msg_handler.h>
#include <network_interface/network_interface.h>

//C++ Includes
#include <unordered_map>

//PCL Includes
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace AS;
using namespace AS::Network;
using namespace AS::Drivers::IbeoScala;

TCPInterface tcp_interface;

int main(int argc, char **argv)
{
  std::string ip_address = "192.168.1.52";
	int port = 12002;
	std::string frame_id = "ibeo_scala";
	bool is_fusion = false;
  bool publish_raw = false;
  unsigned char *msg_buf;
  unsigned char *orig_msg_buf; //Used for deallocation.
  size_t bytes_read;
  int buf_size = IBEO_PAYLOAD_SIZE;
  std::vector<unsigned char> partial_msg;
  std::vector<unsigned char> grand_buffer;
  std::vector<std::vector<unsigned char>> messages;

	// ROS initialization
	ros::init(argc, argv, "ibeo_scala");
	ros::NodeHandle n;
	ros::NodeHandle priv("~");
	ros::Rate loop_rate(1000.0);
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

  if (priv.getParam("publish_raw_data", publish_raw))
  {
    ROS_INFO("Publish raw data: %s", publish_raw ? "true" : "false");
  }

	if(exit)
    return 0;

  return_statuses status = tcp_interface.open(ip_address.c_str(), port);

  if (status == OK)
  {
    ros::Publisher eth_tx_pub = n.advertise<network_interface::TCPFrame>("tcp_tx", 10);
    ros::Publisher pointcloud_pub = n.advertise<pcl::PointCloud <pcl::PointXYZ> >("as_tx/point_cloud", 1);
    ros::Publisher object_markers_pub = n.advertise<visualization_msgs::MarkerArray>("as_tx/objects", 1);
    ros::Publisher object_contour_points_pub = n.advertise<visualization_msgs::Marker>("as_tx/object_contour_points", 1);

    ros::Publisher scan_2202_pub, scan_2205_pub, scan_2208_pub,
                   object_2225_pub, object_2270_pub, object_2271_pub, object_2280_pub,
                   camera_image_pub, vehicle_state_2805_pub, vehicle_state_2806_pub,
                   vehicle_state_2807_pub, device_status_pub;

    std::unordered_map<unsigned short, IbeoRosMsgHandler> handler_list;

    if (is_fusion)
    {

      scan_2205_pub = n.advertise<ibeo_scala_msgs::ScanData2205>("parsed_tx/scan_data_2205", 1);
      object_2225_pub = n.advertise<ibeo_scala_msgs::ObjectData2225>("parsed_tx/object_data_2225", 1);
      object_2280_pub = n.advertise<ibeo_scala_msgs::ObjectData2280>("parsed_tx/object_data_2280", 1);
      camera_image_pub = n.advertise<ibeo_scala_msgs::CameraImage>("parsed_tx/camera_image", 1);
      vehicle_state_2806_pub = n.advertise<ibeo_scala_msgs::HostsVehicleState2806>("parsed_tx/hosts_vehicle_state_2806", 1);
      vehicle_state_2807_pub = n.advertise<ibeo_scala_msgs::HostsVehicleState2807>("parsed_tx/hosts_vehicle_state_2807", 1);

      IbeoRosMsgHandler handler_2205(0x2205, scan_2205_pub);
      IbeoRosMsgHandler handler_2225(0x2225, object_2225_pub);
      IbeoRosMsgHandler handler_2280(0x2280, object_2280_pub);
      IbeoRosMsgHandler handler_2403(0x2403, camera_image_pub);
      IbeoRosMsgHandler handler_2806(0x2806, vehicle_state_2806_pub);
      IbeoRosMsgHandler handler_2807(0x2807, vehicle_state_2807_pub);

      handler_list.insert(std::make_pair(0x2205, handler_2205));
      handler_list.insert(std::make_pair(0x2225, handler_2225));
      handler_list.insert(std::make_pair(0x2280, handler_2280));
      handler_list.insert(std::make_pair(0x2403, handler_2403));
      handler_list.insert(std::make_pair(0x2806, handler_2806));
      handler_list.insert(std::make_pair(0x2807, handler_2807));
    }
    else
    {

      scan_2202_pub = n.advertise<ibeo_scala_msgs::ScanData2202>("parsed_tx/scan_data_2202", 1);
      scan_2208_pub = n.advertise<ibeo_scala_msgs::ScanData2208>("parsed_tx/scan_data_2208", 1);
      object_2270_pub = n.advertise<ibeo_scala_msgs::ObjectData2270>("parsed_tx/object_data_2270", 1);
      object_2271_pub = n.advertise<ibeo_scala_msgs::ObjectData2271>("parsed_tx/object_data_2271", 1);
      vehicle_state_2805_pub = n.advertise<ibeo_scala_msgs::HostsVehicleState2805>("parsed_tx/hosts_vehicle_state_2805", 1);

      IbeoRosMsgHandler handler_2202(0x2202, scan_2202_pub);
      IbeoRosMsgHandler handler_2208(0x2208, scan_2208_pub);
      IbeoRosMsgHandler handler_2270(0x2270, object_2270_pub);
      IbeoRosMsgHandler handler_2271(0x2271, object_2271_pub);
      IbeoRosMsgHandler handler_2805(0x2805, vehicle_state_2805_pub);

      handler_list.insert(std::make_pair(0x2202, handler_2202));
      handler_list.insert(std::make_pair(0x2208, handler_2208));
      handler_list.insert(std::make_pair(0x2270, handler_2270));
      handler_list.insert(std::make_pair(0x2271, handler_2271));
      handler_list.insert(std::make_pair(0x2805, handler_2805));
    }

    device_status_pub = n.advertise<ibeo_scala_msgs::DeviceStatus>("parsed_tx/device_status", 1);
    IbeoRosMsgHandler handler_6301(0x6301, device_status_pub);
    handler_list.insert(std::make_pair(0x6301, handler_6301));


    while (ros::ok())
    {
      buf_size = IBEO_PAYLOAD_SIZE;
      orig_msg_buf = (unsigned char*) calloc(sizeof(unsigned char), buf_size); //New allocation.
      msg_buf = orig_msg_buf;

      status = tcp_interface.read_some(msg_buf, buf_size, bytes_read); //Read a (big) chunk.
      buf_size = bytes_read;
      grand_buffer.insert( grand_buffer.end() , msg_buf , msg_buf + bytes_read);
      //printf("packet debug: finished reading %d bytes. Grand buffer is now %d bytes.\n",bytes_read, grand_buffer.size());


      int first_mw = 0;
      // if (!partial_msg.empty())
      // {
      //   //printf("packet debug: prepending %d remaining bytes from last read.\n", partial_msg.size());
      //   //We have leftover data from last read and we found
      //   //a new magic word.
      //   //Assume that the leftover from last read and anything
      //   //before the new magic word of this read make a new message.

      //   std::vector<unsigned char> new_part_msg(partial_msg.begin(), partial_msg.end());
      //   //new_part_msg.insert(new_part_msg.end(), msg_buf, buf_size); 
      //   //messages.push_back(new_part_msg);
      //   first_mw = find_magic_word(new_part_msg.data(), new_part_msg.size());

      //   partial_msg.clear();
      // }


      int j = 1;
      while( true )
      {
        first_mw = find_magic_word((uint8_t*) grand_buffer.data() + 1, grand_buffer.size() );
        if( first_mw == -1 ) 
        {
          //printf("packet debug: no magic word found in %u 8-bit bytes.\n", grand_buffer.size());
          break;
        }
        // else if( first_mw == 0 )
        // {

        //   grand_buffer.erase(grand_buffer.begin(), grand_buffer.begin() + (sizeof(MAGIC_WORD) / sizeof( unsigned char)));
        //   //printf("Magic word is at the front. Removing. Grand buffer is now %d bytes long.\n", grand_buffer.size());

        // }
        else
        {
          std::vector<unsigned char> msg;
          msg.insert(msg.end(),grand_buffer.begin(), grand_buffer.begin() + first_mw + 1);
          // //printf("packet debug: msg: ");
          // for( unsigned char c : msg )
          // {
          //   //printf("%02x ", c);
          // }
          messages.push_back(msg);
          grand_buffer.erase(grand_buffer.begin(), grand_buffer.begin() + first_mw + 1);
          //printf("\npacket debug: found magic word #%d at %d. Grand buffer is now %u bytes long. There are %u messages to be parsed.\n", j++, first_mw, grand_buffer.size(), messages.size());
          //printf("packet debug: msg size: %u, messages size: %u\n", msg.size(), messages.size());
        }
      }


      //if (first_mw > -1)
      //{
        ////printf("packet debug: found magic word #1 at %d\n", first_mw);
          
        // msg_buf += first_mw; //Point to the byte at the beginning of the first message in this chunk.
        // buf_size -= first_mw;

        // int mw_offset;
        // bool more_magic = true;

        // int i = 1;
        // while (more_magic)
        // {
        //   unsigned char * new_buf = msg_buf + 1;
        //   mw_offset = find_magic_word(new_buf, buf_size - 1);
          

        //   if (mw_offset > -1)
        //   {
        //     //Found another message in this chunk.
        //     std::vector<unsigned char> last_message(msg_buf, msg_buf + mw_offset);
        //     messages.push_back(last_message);
        //     msg_buf = msg_buf + mw_offset + 1; //Point to the beginning of the next message.
        //     buf_size -= mw_offset; //Reduce the size of the array.
        //     //printf("packet debug: found magic word #%d at offset %d\n", ++i, mw_offset);
        //   }
        //   else
        //   {
        //     more_magic = false;
        //   }
        // }

        if (!messages.empty())
        {
          //Found at least one message, let's parse them.
          for(unsigned int i = 0; i < messages.size(); i++)
          {
            ROS_INFO("Parsing message %u of %d.", i, messages.size());
            //printf("packet debug: Parsing message %u of %d.\n", i, messages.size());
            // for(unsigned char c : messages[i])
            // {
            //   printf("%02x ",c);
            // }
            if (publish_raw)
            {
              network_interface::TCPFrame raw_frame;
              raw_frame.address = ip_address;
              raw_frame.port = port;
              raw_frame.size = messages[i].size();
              raw_frame.data.insert(raw_frame.data.begin(), messages[i].begin(), messages[i].end());
              raw_frame.header.frame_id = frame_id;
              raw_frame.header.stamp = ros::Time::now();

              eth_tx_pub.publish(raw_frame);
            }

            ROS_INFO("Size of message: %lu.", messages[i].size());

            IbeoDataHeader ibeo_header;
            ibeo_header.parse(messages[i].data());

            ROS_INFO("Got message type: 0x%X", ibeo_header.data_type_id);

            auto class_parser = IbeoTxMessage::make_message(ibeo_header.data_type_id); //Instantiate a parser class of the correct type.
            ROS_INFO("Created class parser.");

            if (class_parser != NULL)
            {
              //Only parse message types we know how to handle.
              class_parser->parse(messages[i].data()); //Parse the raw data into the class.
              ROS_INFO("Parsed data.");
              auto msg_handler = handler_list.at(ibeo_header.data_type_id); //Get a message handler that was created with the correct parameters.
              ROS_INFO("Created message handler.");
              msg_handler.encode_and_publish(class_parser, frame_id); //Create a new message of the correct type and publish it.
              ROS_INFO("Encoded ROS message.");

              //TODO: Figure out what to do with points and objects.
              if (class_parser->has_scan_points)
              {
                pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
                pcl_cloud.header.frame_id = frame_id;
                pcl_cloud.header.stamp = ibeo_header.time;
                std::vector<Point3D> scan_points = class_parser->get_scan_points();
                msg_handler.encode_pointcloud(scan_points, pcl_cloud);
                pointcloud_pub.publish(pcl_cloud);
              }

              if (class_parser->has_contour_points)
              {
                visualization_msgs::Marker marker;
                marker.header.frame_id = frame_id;
                marker.header.stamp = ros::Time::now();
                std::vector<Point3D> contour_points = class_parser->get_contour_points();
                if( contour_points.size() > 0 )
                {
                  //printf("ready to encode %d contour points.\n ", (int) contour_points.size() );
                  msg_handler.encode_contour_points(contour_points, marker);
                  //printf("marker array ready to publish with %d contour points. ",marker.points.size());
                  object_contour_points_pub.publish(marker);
                  //printf(" DONE.\n");
                }
                
              }

              if (class_parser->has_objects)
              {
                std::vector<IbeoObject> objects = class_parser->get_objects();
                visualization_msgs::MarkerArray marker_array;
                msg_handler.encode_marker_array(objects, marker_array);
                for( visualization_msgs::Marker m : marker_array.markers )
                {
                  //printf("setting marker %d frame_id to %s.\n",m.id, frame_id.c_str());
                  m.header.frame_id = frame_id;
                }

                object_markers_pub.publish(marker_array);
              }

            }
            else
            {
              printf("class parser for %u byte message of type 0x%04x is NULL\n", messages[i].size(), ibeo_header.data_type_id);
            }
          }
          //printf("packet debug: Clearing %u messages.\n", messages.size());
          messages.clear();

        }

      

        // if (buf_size > 0)
        // {
        //   //We still have data left over. Add it to the next loop.
        //   partial_msg.insert(partial_msg.end(), msg_buf, msg_buf + buf_size);
        // }
      //}
      // else
      // {
      //   //printf("packet debug: no magic word found in this packet. ");
      //   if( !partial_msg.empty() ) //printf("%d byte partial message remaining. ", partial_msg.size());
      //   //printf("\n");
      //   partial_msg.insert(partial_msg.end(), msg_buf, msg_buf + buf_size);
      // }

      free(orig_msg_buf); //FREE THE BITS
      loop_rate.sleep();
      //ros::spinOnce(); // No callbacks yet - no reason to spin.
    }
  }
  else
  {
    ROS_ERROR("Connection to ScaLa could not be opened. Error no %i", status);
  }

  status = tcp_interface.close();

  if (status != OK)
    ROS_ERROR("Connection to ScaLa was not properly closed. Error no %i", status);

  return 0;
}
