/*  sd_filter_node.cpp
 *
 *  Created on: 04.05.2017
 *      Author: Nicko_Dema@protonmail.com
 *		ITMO University
 *		Department of Computer Science and Control Systems
 */

#include "sd_filter.h"

int main( int argc, char **argv)
{
    ros::init( argc, argv, "sensor_data_filter" );
    sdFilter robotinoF;
    robotinoF.spin();

    return 0;
}
