/* mapper.cpp
 *
 *  Created on: 19.04.2017
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Department of Computer Science and Control Systems
 */

#include <mapper.h>

Mapper::Mapper(std::string node_name) : nh_("~"), map_keeper(CELL_N)
{
	map_pub = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
	odom_sub = nh_.subscribe("/odom", 1, &Mapper::odom_cb, this);

	last_pose.position.x = 0;
	last_pose.position.y = 0;
	//----------------
}

void Mapper::odom_cb(const nav_msgs::Odometry& odom_msg)
{
	//смещение
	double xy_cells_n;
	double err;
	struct ch {
		double x = 0;
		double y = 0;
	};
	static ch odom_ch;
	std::cout << "1" << std::endl;	//------------------------------------
	//wait for single message from distance sensors
	sensor_msgs::PointCloudConstPtr distances_msg = 
	ros::topic::waitForMessage<sensor_msgs::PointCloud>("/distance_sensors");
	
	odom_ch.x = odom_msg.pose.pose.position.x - last_pose.position.x;
	odom_ch.y = odom_msg.pose.pose.position.y - last_pose.position.y;
	last_pose.position.x = odom_msg.pose.pose.position.x;
	last_pose.position.y = odom_msg.pose.pose.position.y;

	std::cout << odom_ch.x << " " << odom_ch.y << " odom_ch" << std::endl;	//--
	std::cout << last_pose.position.x << " " << last_pose.position.y << " last_pose" << std::endl;	//--

	std::cout << "2" << std::endl;	//--------------------------------------

	//update non-rotating frame for the map
	tf::TransformListener listener;
	tf::StampedTransform transform;
	static tf::TransformBroadcaster br;
	try{
	    ros::Time now = ros::Time(0);
	    listener.waitForTransform("/odom", "/base_link", now, ros::Duration(10));
	    listener.lookupTransform("/odom", "/base_link", now, transform);
	}
	catch (tf::TransformException &ex)
	{
	    ROS_ERROR("%s",ex.what());
	}
	//change transform quaternion to (0, 0, 0, 1)
	transform.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "/base_map"));



	std::cout << "3" << std::endl;	//------------------------------------
	//Определение смещения робота в клетках карты для оси Х
	//АТЭНШН, при малых перемещениях (скорости) смещений происходить не будет
	err = abs(modf((odom_ch.x + map_keeper.x_error)/CELL_H, &xy_cells_n));
	std::cout << xy_cells_n << " xy_cells_n1" << std::endl;	//--------
	xy_cells_n = abs(xy_cells_n)/2;
	std::cout << xy_cells_n << " xy_cells_n2" << std::endl;	//--------

	if ((xy_cells_n - (long)xy_cells_n) > 0.5) {
		xy_cells_n = (long)xy_cells_n + 1;
	}
	else {
		xy_cells_n = (long)xy_cells_n;
	}
	std::cout << xy_cells_n << "xy_cels X" << std::endl;	//--

	//Смещение карты в соответствии с одометрией по оси Х
	//здесь нужно менять x_error, а не odom_ch!!!!!!!!!!!!!!!!!!
	if (odom_ch.x > 0)
	{
		if (odom_ch.x > CELL_H - map_keeper.x_error)
		{
			std::cout << "call move with f" << std::endl;	//--------
			map_keeper.move((int)xy_cells_n, 'f');
			map_keeper.x_error = odom_ch.x - (xy_cells_n*CELL - map_keeper.x_error);
		}
		else
		{
			map_keeper.x_error = map_keeper.x_error + odom_ch.x;
		}
	}
	else
	{
		if (abs(odom_ch.x) > CELL_H + map_keeper.x_error)
		{
			std::cout << "call move with b" << std::endl;	//--------
			map_keeper.move((int)xy_cells_n, 'b');
			map_keeper.x_error = (xy_cells_n*CELL + map_keeper.x_error) - abs(odom_ch.x);
		}
		else
		{
			map_keeper.x_error = map_keeper.x_error + odom_ch.x;
		}
	}



	//Определение смещения робота в клетках карты для оси Y
	err = abs(modf((odom_ch.y + map_keeper.y_error)/CELL_H, &xy_cells_n));
	std::cout << xy_cells_n << " xy_cells_n1" << std::endl;	//--------
	xy_cells_n = abs(xy_cells_n)/2;
	std::cout << xy_cells_n << " xy_cells_n2" << std::endl;	//--------

	if ((xy_cells_n - (long)xy_cells_n) > 0.5) {
		xy_cells_n = (long)xy_cells_n + 1;
	}
	else {
		xy_cells_n = (long)xy_cells_n;
	}
	std::cout << xy_cells_n << "xy_cels Y" << std::endl;	//--

	//Смещение карты в соответствии с одометрией по оси Х
	//здесь нужно менять x_error, а не odom_ch!!!!!!!!!!!!!!!!!!
	if (odom_ch.y > 0)
	{
		if (odom_ch.y > CELL_H - map_keeper.y_error)
		{
			std::cout << "call move with l" << std::endl;	//--------
			map_keeper.move((int)xy_cells_n, 'l');
			map_keeper.y_error = odom_ch.y - (xy_cells_n*CELL - map_keeper.y_error);
		}
		else
		{
			map_keeper.y_error = map_keeper.y_error + odom_ch.y;
		}
	}
	else
	{
		if (abs(odom_ch.y) > CELL_H + map_keeper.y_error)
		{
			std::cout << "call move with r" << std::endl;	//--------
			map_keeper.move((int)xy_cells_n, 'r');
			map_keeper.y_error = (xy_cells_n*CELL + map_keeper.y_error) - abs(odom_ch.y);
		}
		else
		{
			map_keeper.y_error = map_keeper.y_error + odom_ch.y;
		}
	}


	//get yaw from odom
	//Нужно помнить стартовый угол из одома и от него плясать
	tf::Pose pose_fyaw;
	tf::poseMsgToTF(odom_msg.pose.pose, pose_fyaw);
	double yaw_angle = tf::getYaw(pose_fyaw.getRotation());

	std::cout << distances_msg->points.size() << " points" << std::endl;	//--------
	std::cout << yaw_angle << " yaw" << std::endl;	//--

	for (int i=0;i<distances_msg->points.size(); i++)
	{
		Map_builder builder(distances_msg->points[i].x, distances_msg->points[i].y);
		std::cout << "Create bulder" << std::endl;	//--
		builder.div_by_two();
		std::cout << "div" << std::endl;	//--
		builder.to_map(yaw_angle, map_keeper.x_error, map_keeper.y_error, map_keeper.map_);
		std::cout << "to map" << std::endl;	//--




/*		//АТЭНШН, если ошибка слишком велика, то значения улетают за пределы массива
		if (sqrt(pow(distances_msg->points[i].x, 2) + pow(distances_msg->points[i].y, 2)) > 0.57) continue;
		///////////////////////////////////////////////////////////
		

		///////////////////////////////////////////////////////////

		double x = distances_msg->points[i].x + map_keeper.x_error;
		double y = distances_msg->points[i].y + map_keeper.y_error;
		//std::cout << x << " " << y << " point wit err" << std::endl;	//---------

		double xR = x*cos(yaw_angle) - y*sin(yaw_angle);
		double yR = y*cos(yaw_angle) + x*sin(yaw_angle);

		//std::cout << xR << " " << yR << " point0 after rotate" << std::endl;	//------

		if (xR > -0.02 && xR < 0.02) xR = R_POSE;
		else xR = R_POSE+(xR+CELL_H)/CELL;
		if (yR > -0.02 && yR < 0.02) yR = R_POSE;
		else yR = R_POSE+(yR+CELL_H)/CELL;

		if (xR < 0) xR = 0;
		if (yR < 0) yR = 0;

		//seg fault here if .. (отрицательные значения еблан)
		map_keeper.map_[(int)yR][(int)xR] = 100;*/
	}

	nav_msgs::OccupancyGrid map_msg;
	map_msg.header.frame_id = "/base_map";
	map_msg.header.stamp = ros::Time::now();
	map_msg.info.map_load_time = ros::Time::now();
	map_msg.info.resolution = 0.04;
	map_msg.info.width = CELL_N;
	map_msg.info.height = CELL_N;
	map_msg.info.origin.position.x = -1.02;
	map_msg.info.origin.position.y = -1.02;
	map_msg.info.origin.position.z = 0;
	map_msg.info.origin.orientation.x = 0.0;
	map_msg.info.origin.orientation.y = 0.0;
	map_msg.info.origin.orientation.z = 0.0;
	map_msg.info.origin.orientation.w = 1.0;

	for(int i = 0; i < CELL_N; i++) { 
		for(int j = 0; j < CELL_N; j++) {
			map_msg.data.push_back(map_keeper.map_[i][j]);
		}
	}
	map_pub.publish(map_msg);
}

void Mapper::spin()
{
    ros::Rate R(5);
	odom_init();
   	while(nh_.ok())
    {
		ros::spinOnce();
        R.sleep();
    }
}

void Mapper::odom_init()
{
	ros::Rate R(1);
	while ((odom_sub.getNumPublishers() < 1) && nh_.ok()) 
	    {
	        ROS_WARN_ONCE("Wait for /odom publisher.");
	        R.sleep();
	    }
	nav_msgs::OdometryConstPtr odom_msg = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom");
	last_pose = odom_msg->pose.pose;
	std::cout << last_pose.position.x << "odom init" << std::endl;	//--

	//create non-rotating frame for the map
	tf::TransformListener listener;
	tf::StampedTransform transform;
	tf::TransformBroadcaster br;
	try{
	    ros::Time now = ros::Time(0);
	    listener.waitForTransform("/odom", "/base_link", now, ros::Duration(20));
	    listener.lookupTransform("/odom", "/base_link", now, transform);
	}
	catch (tf::TransformException &ex)
	{
	    ROS_ERROR("%s",ex.what());
	    return;
	}
	//change transform quaternion to (0, 0, 0, 1)
	transform.setRotation(tf::createQuaternionFromRPY(0, 0, 0));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "/base_map"));
	std::cout << "set base_map init" << std::endl;	//--

}

Mapper::~Mapper()
{
    map_pub.shutdown();
	odom_sub.shutdown();
}