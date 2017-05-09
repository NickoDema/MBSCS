/*  sd_filter.cpp
 *
 *  Created on: 04.05.2017
 *      Author: Nicko_Dema@protonmail.com
 *		ITMO University
 *		Department of Computer Science and Control Systems
 */

#include <sd_filter.h>

sdFilter::sdFilter(): nh("~")
{
    dist_sens_sub = nh.subscribe( "/distance_sensors", 1, &sdFilter::filter_cb, this );
    dist_sens_pub = nh.advertise<sensor_msgs::PointCloud>( "/distance_sensors_filtered", 1, true );
}

void sdFilter::spin()
{
    ros::Rate rate(30);
    while( nh.ok() )
    {
	ros::spinOnce();
	rate.sleep();
    }
}

void sdFilter::filter_cb( const sensor_msgs::PointCloudConstPtr& msg )
{   
   /**
    */
	static ros::Time stamp = msg->header.stamp;
	ros::Duration dt_dur = msg->header.stamp - stamp;
	double dt = dt_dur.toSec();

	static std::vector<geometry_msgs::Point32> points;

	//Первый вызов функции должен прерваться
	stamp = msg->header.stamp;
	sensor_msgs::PointCloud msg_fltred;
	msg_fltred.header.frame_id = "/base_link";

	if (points.size() != msg->points.size()) 
	{
		points.resize(msg->points.size());
		for (int i=0; i < msg->points.size(); i++)
		{	
			points[i].x = msg->points[i].x;
			points[i].y = msg->points[i].y;
			msg_fltred.points.push_back(points[i]); 
		}

		msg_fltred.header.stamp = ros::Time::now();
		dist_sens_pub.publish(msg_fltred);
		return;
	}

	//calculate alpha here
	double alpha = 1 - exp(-dt/TAO);
	for (int i = 0; i < msg->points.size(); i++)
	{	
		points[i].x += alpha*(msg->points[i].x - points[i].x);
		points[i].y += alpha*(msg->points[i].y - points[i].y);
		msg_fltred.points.push_back(points[i]); 
	}

	msg_fltred.header.stamp = ros::Time::now();
	dist_sens_pub.publish(msg_fltred);


}

sdFilter::~sdFilter()
{
    dist_sens_pub.shutdown();
    dist_sens_sub.shutdown();
}
