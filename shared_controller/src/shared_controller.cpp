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
	cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
    map_pub = nh_.advertise<nav_msgs::OccupancyGrid>("/map_sec", 1, true);
    cmd_vel_sub = nh_.subscribe("/cmd_vel_joy", 1, &Controller::cmd_vel_cb, this);
    //map_sub = nh_.subscribe("/map", 1, &Mapper::odom_cb, this);
	marker_pub = nh_.advertise<visualization_msgs::Marker>("/visualization", 1,true);

	base[0].x = X_SIZE; base[0].y = Y_SIZE;		//= Frame(X_SIZE, Y_SIZE);
	base[1].x = X_SIZE; base[1].y = 0; 		//= Frame(X_SIZE, 0);
	base[2].x = X_SIZE; base[2].y = -Y_SIZE; 		//= Frame(X_SIZE, -Y_SIZE);
	base[3].x = 0; 		base[3].y = -Y_SIZE; 		//= Frame(0, -Y_SIZE);
	base[4].x = -X_SIZE; base[4].y = -Y_SIZE; 		//= Frame(-X_SIZE, -Y_SIZE);
	base[5].x = -X_SIZE; base[5].y = 0; 		//= Frame(-X_SIZE, 0);
	base[6].x = -X_SIZE; base[6].y = Y_SIZE; 		//= Frame(-X_SIZE, Y_SIZE);
	base[7].x = 0; 		 base[7].y = Y_SIZE; 		//= Frame(0, Y_SIZE);
}

void Controller::cmd_vel_cb(const geometry_msgs::Twist &cmd_vel_msg)
{
	nav_msgs::OccupancyGridConstPtr map_msg = 
	ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");

	tf::TransformListener listener;
	tf::StampedTransform transform;
	try{
	    ros::Time now = ros::Time(0);
	    listener.waitForTransform("/odom", "/base_link", now, ros::Duration(10));
	    listener.lookupTransform("/odom", "/base_link", now, transform);
	}
	catch (tf::TransformException &ex)
	{
	    ROS_ERROR("%s",ex.what());
	}

	tf::Quaternion q = transform.getRotation();
    double yaw = tf::getYaw(q);
	//yaw *= (-1);
	std::cout << "yaw " << yaw << std::endl;	//++++++++++++

	float R, dPHI, PHI;
	float dX = CELL;
	float endX = X_SIZE + BORD;
	float cmd_x = 0;
	float cmd_z = 0;

	visualization_msgs::Marker points;
	geometry_msgs::Twist cmd_vel_sec_msg;
	nav_msgs::OccupancyGrid map_sec_msg;
	geometry_msgs::Point pnt;

	//map message initialization
	map_sec_msg.header.frame_id = map_msg->header.frame_id;
	map_sec_msg.header.stamp = ros::Time::now();
	map_sec_msg.info = map_msg->info;
	map_sec_msg.info.map_load_time = ros::Time::now();
	//map_sec_msg.data(map_msg->data);
	for(int w = 0; w < map_msg->data.size(); w++) {
			map_sec_msg.data.push_back(map_msg->data[w]);
		}

	pnt.z = 0.01;
	set_marker(points);

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

std::cout << "cmd x z in : " << cmd_x << " " << cmd_z << std::endl;	//++++++++++++++++++++++
	if ((abs(cmd_z) > MIN_VEL_ANG) || (abs(cmd_x) > MIN_VEL_LIN))
	{
		float min_for_x = MIN_X_RNG;
		float min_for_z = MIN_Z_RNG;
		float min_for_z_sect[8];
		for (int k = 0; k < 8; k++) {
			min_for_z_sect[k] = MIN_Z_RNG;
		}

		//only linear movement
		if (abs(cmd_z) < MIN_VEL_ANG) 
		{
			//выбор начальной точки
			int i = 0;
			if (cmd_x < 0)
			{
				i = 4;
				dX *= (-1);
			}

			//определение границ интервала
			float begin,end;
			if (base[i].y > base[i+2].y) 
			{
				begin = base[i+2].y;
				end = base[i].y;
			}
			else
			{
				begin = base[i].y;
				end = base[i+2].y;
			}
			std::cout << "b e" << begin << " " << end << std::endl << std::endl;	//++++++++++++++++++++++

			int k_y_max, p;
			k_y_max = (end - begin)/CELL;

			for (float k_y = begin, p = 0; p <= k_y_max; p++, k_y += CELL)
			{	
				//std::cout << "k_y circle " << k_y << std::endl << std::endl;	//++++++++++++++++++++++
				//draw a prediction lines
				if ( (p == 0) || (p == k_y_max) )
				{	
					//std::cout << "if begin/end" << std::endl;	//++++++++++++++++++++++
					pnt.x = base[i].x;
					pnt.y = k_y;
					points.points.push_back(pnt);
					for(int j = 1; j < 9; j++)
					{
						pnt.x = pnt.x + dX;
						points.points.push_back(pnt);
					}
				}

				//get all the point from the sector
				for (float k_x = base[i].x; (k_x <= endX) && (k_x >= (-endX)); k_x += dX)
				{
					
					double x_map = k_x*cos(yaw) - k_y*sin(yaw);
					double y_map = k_y*cos(yaw) + k_x*sin(yaw);
					
					int x_cell = (int)(x_map/CELL) + R_POSE;
					int y_cell = (int)(y_map/CELL) + R_POSE;
					
					if (map_sec_msg.data[y_cell*CELL_N+x_cell] == 100)
					{
						float range_cur = abs(k_x) - abs(base[i].x);
						if ( range_cur < min_for_x ) {
							min_for_x = range_cur;
						}
						if (range_cur < BORD - 0.1) {
							map_sec_msg.data[y_cell*CELL_N+x_cell] = 
							(int)(-128)*(1 - range_cur/(BORD + 0.02));
						}
					}
				}
			}
		cmd_vel_sec_msg.linear.x = cmd_x*min_for_x/MIN_X_RNG;
		}



		else
		{

			float y_R = cmd_x/cmd_z;		
			std::cout << "y_R = " << y_R << std::endl;	//+++++++++++++yaw+++++++++

			for (int i = 0; i < 8; i++)
			{
				dPHI = 1;
				if (cmd_z < 0) {
					dPHI *= (-1);
				}
				std::cout << "i = " << i << std::endl;	//++++++++++++

				//определение границ интервала для каждого случая
				float begin,end, border;
				bool vert_edge;
				if (i == 0 || i == 1 || i == 4 || i == 5)
				{	
					vert_edge = false;
					border = base[i].x;
					if (base[i].y - y_R > base[i+1].y - y_R) 
					{
						begin = base[i+1].y - y_R;
						end = base[i].y - y_R;
					}
					else
					{
						begin = base[i].y - y_R;
						end = base[i+1].y - y_R;
					}
				}
				else if (i == 7)
				{
					vert_edge = true;
					border = base[i].y - y_R;
					if (base[i].x > base[0].x) 
					{
						begin = base[0].x;
						end = base[i].x;
					}
					else
					{
						begin = base[i].x;
						end = base[0].x;
					}
				}
				else
				{	
					vert_edge = true;
					border = base[i].y - y_R;
					if (base[i].x > base[i+1].x) 
					{
						begin = base[i+1].x;
						end = base[i].x;
					}
					else
					{
						begin = base[i].x;
						end = base[i+1].x;
					}
				}
				std::cout << "begin | end = " << begin << " | " << end << std::endl;	//+++++++++++++++
				std::cout << "vert_edge = " << vert_edge << std::endl;	//+++++++++++++++
				std::cout << "border = " << border << std::endl;	//+++++++++++++++
				
				//external circle
				int j_ext_max = (end - begin)/CELL;
				float p_ext = begin;
				for (int j_ext = 0; j_ext < j_ext_max; j_ext++, p_ext += CELL)
				{
					R = sqrt(pow(p_ext,2) + pow(border,2));
					dPHI *= CELL/R;
					int j_int_max = BORD/CELL;

					float x_var;
					float y_var;
					if (vert_edge)
					{
						x_var = p_ext;
						y_var = border;
					}
					else
					{
						x_var = border;
						y_var = p_ext;	
					}

					//internal circle
					for (int j_int = 0; j_int < j_int_max; j_int++)
					{
						

						//draw a pridiction lines
						if ((j_ext == 0 && (i == 0 || i == 2 || i == 4 || i == 6)))
						{
							//x_var err
							pnt.x = base[i].x;
							pnt.y = base[i].y;
							points.points.push_back(pnt);
							//pnt.y -= y_R;
							for(int j = 1; j < 8; j++)
							{
								pnt.y -= y_R;
								float x_nxt = pnt.x*cos(dPHI) - pnt.y*sin(dPHI);
								float y_nxt = pnt.y*cos(dPHI) + pnt.x*sin(dPHI);
								pnt.x = x_nxt;
								pnt.y = y_nxt + y_R;
								if ((pnt.x >= X_SIZE || pnt.x <= -X_SIZE) || 
								((pnt.y >= Y_SIZE) || (pnt.y <= -Y_SIZE)))
								{
									points.points.push_back(pnt);
									//pnt.y = y_nxt;
								}
							}
						}

						float x_nxt = x_var*cos(dPHI) - y_var*sin(dPHI);
						float y_nxt = y_var*cos(dPHI) + x_var*sin(dPHI);
						if ((x_nxt >= X_SIZE || x_nxt <= -X_SIZE) || 
								((y_nxt >= Y_SIZE) || (y_nxt <= -Y_SIZE)))
						{
							x_var = x_nxt;
							y_var = y_nxt;
							y_nxt += y_R;

							float x_map = x_nxt*cos(yaw) - y_nxt*sin(yaw);
							float y_map = y_nxt*cos(yaw) + x_nxt*sin(yaw);

							int x_cell = (int)(x_map/CELL) + R_POSE;
							int y_cell = (int)(y_map/CELL) + R_POSE;

							if (map_sec_msg.data[y_cell*CELL_N+x_cell] == 100)
							{
								float range_cur = j_int*CELL;
								if ( range_cur < min_for_z_sect[i] ) {
									min_for_z_sect[i] = range_cur;
								}
								if (range_cur < BORD - 0.05) {
									map_sec_msg.data[y_cell*CELL_N+x_cell] = 
									(int)(-128)*(1 - range_cur/(BORD + 0.02));
								}
							}
						}
					}
				}
			std::cout << "min_sect " << min_for_z_sect[i] << std::endl;	//+++++++++++++++				
			}
			//choose minimal velocity
			for (int k = 0; k < 8; k++)
			{
				if (min_for_z_sect[k] < min_for_z)
					{
						min_for_z = min_for_z_sect[k];
						if (k == 0 || k == 1 || k == 4 || k == 5) {
							if (min_for_z_sect[k] < min_for_x) {
								min_for_x = min_for_z_sect[k];
							}
						}
					}
			}

		}

		cmd_vel_sec_msg.linear.x = cmd_x*min_for_x/MIN_X_RNG;
		cmd_vel_sec_msg.angular.z = cmd_z*min_for_z/MIN_Z_RNG;


	if (abs(cmd_vel_sec_msg.linear.x)  < MIN_VEL_LIN) cmd_vel_sec_msg.linear.x = 0;
	if (abs(cmd_vel_sec_msg.angular.z) < MIN_VEL_ANG) cmd_vel_sec_msg.angular.z = 0;

	std::cout << "cmd x z out: " << cmd_vel_sec_msg.linear.x << " " 
			  << cmd_vel_sec_msg.angular.z << std::endl;	//++++++++++++++++++++++

	cmd_vel_pub.publish(cmd_vel_sec_msg);
	map_pub.publish(map_sec_msg);
	marker_pub.publish(points);
	}	 
}

void Controller::spin()
{
    ros::Rate R(80);
	//odom_init();
   	while(nh_.ok())
    {
		ros::spinOnce();
        R.sleep();
    }
}

Controller::~Controller()
{
    map_pub.shutdown();
	cmd_vel_pub.shutdown();
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
		marker.scale.x = 0.01;
		marker.scale.y = 0.01;
		marker.scale.z = 0.01;
		marker.color.r = 0.7f;
		marker.color.g = 0.5f;
		marker.color.b = 1.0f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration(0.5);
    
    return true;
}