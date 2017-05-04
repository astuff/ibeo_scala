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
#include <network_interface/network_interface.h>

//ROS includes
#include <ros/ros.h>
//Messages

using namespace AS;
using namespace AS::Network;
using namespace AS::Drivers::IbeoScala;

TCPInterface tcp_interface;

int main(int argc, char **argv)
{
  //int c;
  std::string ip_address = "192.168.1.52";
	int port = 12002;
	std::string frame_id = "ibeo_scala";
	bool is_fusion = false;

	// ROS initialization
	ros::init(argc, argv, "ibeo_scala");
	ros::NodeHandle n;
	ros::NodeHandle priv("~");
	ros::Rate loop_rate(1.0/0.01);
	bool exit = false;

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
  }
  else
  {
    ROS_ERROR("Connection to ScaLa could not be opened");
  }

  tcp_interface.close();
  return 0;
}
