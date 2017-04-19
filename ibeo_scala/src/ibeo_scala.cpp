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
