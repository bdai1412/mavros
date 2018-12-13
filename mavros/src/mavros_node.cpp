/**
 * @brief MAVROS Node
 * @file mavros_node.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2013,2014,2015 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros.h>
#include <signal.h>


static void sigintHandle(int sig)
{
  ros::Rate rate_wait(1.0);
	bool is_offboard_node_alive = false;
	ros::param::get("/is_offboard_node_alive", is_offboard_node_alive);
	while(is_offboard_node_alive)
	{
		ros::param::get("/is_offboard_node_alive", is_offboard_node_alive);
		ROS_WARN("Waiting for offboard shutting down! Please kill offboard first!");
		rate_wait.sleep();
	}
	ROS_WARN("Mavros Exiting!");
  ros::shutdown();
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "mavros", ros::init_options::NoSigintHandler);

  signal(SIGINT, sigintHandle);

	mavros::MavRos mavros;
	mavros.spin();

	return 0;
}

