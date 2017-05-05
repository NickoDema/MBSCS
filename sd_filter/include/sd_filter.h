/*  sd_filter.h
 *
 *  Created on: 04.05.2017
 *      Author: Nicko_Dema@protonmail.com
 *		ITMO University
 *		Department of Computer Science and Control Systems
 */

#ifndef SDFILTER_H_
#define SDFILTER_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include <vector>
#include <cmath>

#define TAO 0.4 //sec

class sdFilter
{
	public:
	sdFilter();
	~sdFilter();

	void spin();

	private:
		ros::NodeHandle nh;

	  	ros::Publisher  dist_sens_pub;
	  	ros::Subscriber dist_sens_sub;

	  	void filter_cb( const sensor_msgs::PointCloudConstPtr&);

};

#endif /* SDFILTER_H_ */
