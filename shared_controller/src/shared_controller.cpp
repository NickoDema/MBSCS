/* shared_controller.cpp
 *
 *  Created on: 09.05.2017
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Department of Computer Science and Control Systems
 */

#include <shared_controller.h>

Controller::Controller(std::string node_name) : nh_("~")
{
	//cmd_vel_pub = nh_.advertise<nav_msgs::OccupancyGrid>("/cmd_vel", 1, true);
    //map_pub = nh_.advertise<nav_msgs::OccupancyGrid>("/sec_map", 1, true);
    cmd_vel_sub = nh_.subscribe("/cmd_vel", 1, &Controller::cmd_vel_cb, this);
    //map_sub = nh_.subscribe("/map", 1, &Mapper::odom_cb, this);

	marker_pub = nh_.advertise<visualization_msgs::Marker>("/visualization", 1,true);

	std::cout << "1" << std::endl;	//++++++++++++++++++++++

	base[0].x = X_SIZE; base[0].y = Y_SIZE;		//= Frame(X_SIZE, Y_SIZE);
	base[1].x = X_SIZE; base[1].y = 0; 		//= Frame(X_SIZE, 0);
	base[2].x = X_SIZE; base[2].y = -Y_SIZE; 		//= Frame(X_SIZE, -Y_SIZE);
	base[3].x = 0; 		base[3].y = -Y_SIZE; 		//= Frame(0, -Y_SIZE);
	base[4].x = -X_SIZE; base[4].y = -Y_SIZE; 		//= Frame(-X_SIZE, -Y_SIZE);
	base[5].x = -X_SIZE; base[5].y = 0; 		//= Frame(-X_SIZE, 0);
	base[6].x = -X_SIZE; base[6].y = Y_SIZE; 		//= Frame(-X_SIZE, Y_SIZE);
	base[7].x = 0; 		 base[7].y = Y_SIZE; 		//= Frame(0, Y_SIZE);

	std::cout << "2" << std::endl;	//++++++++++++++++++++++
}

void Controller::cmd_vel_cb(const geometry_msgs::Twist &cmd_vel_msg)
{
	std::cout << "3" << std::endl;	//++++++++++++++++++++++
	visualization_msgs::Marker points;
	set_marker(points);

	float R;
	float dX = CELL_H, dPHI;
	float cmd_x = 0;
	float cmd_z = 0;
	if (abs(cmd_vel_msg.linear.x) > MAX_VEL_LIN)
	{
		cmd_x = MAX_VEL_LIN;
		if (cmd_vel_msg.linear.x < 0) {
			cmd_x *= (-1);
		}
	}
	else
	{
		if (abs(cmd_vel_msg.linear.x) > MIN_VEL_LIN) {
			cmd_x = cmd_vel_msg.linear.x;
		}
	}

	if (abs(cmd_vel_msg.angular.z) > MAX_VEL_ANG)
	{
		cmd_z = MAX_VEL_ANG;
		if (cmd_vel_msg.angular.z < 0) {
			cmd_z *= (-1);
		}
	}
	else
	{
		if (abs(cmd_vel_msg.angular.z) > MIN_VEL_ANG) {
			cmd_z = cmd_vel_msg.angular.z;
		}
	}


	std::cout << "cmd_x " << cmd_x << std::endl;	//++++++++++++++++++++++
	std::cout << "cmd_z " << cmd_z << std::endl;	//++++++++++++++++++++++

	if ((abs(cmd_z) > MIN_VEL_ANG) || (abs(cmd_x) > MIN_VEL_LIN))
	{
		std::cout << "3" << std::endl;	//++++++++++++++++++++++
		if (abs(cmd_z) < MIN_VEL_ANG) 
		{
			int i = 0;
			if (cmd_x < 0)
				{
					i = 4;
					dX *= (-1);
				}
			for (; i < 8; i+=2)
			{	
				std::cout << "6 ";	//++++++++++++++++++++++
				//------------------
				pnt.x = base[i].x;
				pnt.y = base[i].y;
				points.points.push_back(pnt);

				for(int j = 1; j < 8; j++)
				{
					pnt.x = pnt.x + dX;
					points.points.push_back(pnt);
				}
				if (dX > 0 && i == 2) break;
			}
			std::cout << "7" << std::endl;	//++++++++++++++++++++++
		}
		else
		{
			float y_R = cmd_x/cmd_z;		//*180/(3.1415*cmd_z);
			std::cout << "y_R = " << y_R << std::endl;	//++++++++++++++++++++++

			for (int i = 0; i < 8; i+=2)
			{
				dPHI = 1;
				if (cmd_z < 0) dPHI  *= (-1);

				std::cout << "8" << std::endl;	//++++++++++++++++++++++
				std::cout << "i = " << i << std::endl;	//++++++++++++++++++++++
				pnt.x = base[i].x;
				pnt.y = base[i].y;
				std::cout << "x y = " << pnt.x << " " << pnt.y << std::endl;	//++++++++++++++++++++++
				points.points.push_back(pnt);
				//переход в новую систему координат для y
				pnt.y = base[i].y - y_R;
				R = sqrt(pow(base[i].x,2) + pow(pnt.y,2));
				std::cout << "R = " << R << std::endl;	//++++++++++++++++++++++
				dPHI *= CELL_H/R; //acos(CELL_H*2/R);
				std::cout << "dPHI = " << dPHI << std::endl;	//++++++++++++++++++++++
				
				for(int j = 1; j < 8; j++)
				{
					std::cout << "9 ";	//++++++++++++++++++++++
					double x_nxt = pnt.x*cos(dPHI) - pnt.y*sin(dPHI);
					double y_nxt = pnt.y*cos(dPHI) + pnt.x*sin(dPHI);
					pnt.x = x_nxt;
					pnt.y = y_nxt + y_R;
					std::cout << "x y = " << pnt.x << " " << pnt.y << std::endl;	//++++++++++++++++++++++
					points.points.push_back(pnt);
					pnt.y = y_nxt;
				}
				std::cout << std::endl;	//++++++++++++++++++++++
			}
			//dPHI  =*(-1);
		}
	marker_pub.publish(points);
	}
	//std::cout << "10" << std::endl;	//++++++++++++++++++++++
	 
}

void Controller::spin()
{
    ros::Rate R(40);
	//odom_init();
   	while(nh_.ok())
    {
		ros::spinOnce();
        R.sleep();
    }
}

Controller::~Controller()
{
    //map_pub.shutdown();
	//cmd_vel_pub.shutdown();
	//map_sub.shutdown();
	cmd_vel_sub.shutdown();
	marker_pub.shutdown();
}

bool Controller::set_marker (visualization_msgs::Marker &marker)
{
		marker.header.frame_id = "/base_link";
		marker.header.stamp = ros::Time::now();
		marker.ns = "markers";
		marker.id = 2;
		marker.type = visualization_msgs::Marker::POINTS;
		marker.action = visualization_msgs::Marker::ADD;

		marker.pose.orientation.w = 1.0; 
		marker.scale.x = 0.03;
		marker.scale.y = 0.03;
		marker.scale.z = 0.03;
		marker.color.r = 0.7f;
		marker.color.g = 0.5f;
		marker.color.b = 1.0f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration(0.5);
    
    return true;
}