/* visualizer.h
 *
 *  Created on: 16.04.2017
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Department of Computer Science and Control Systems
 */

#ifndef NAV_STEP_
#define NAV_STEP_

#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <cmath>

class Visualizer
{
    public:
        Visualizer(std::string);
        ~Visualizer();
        void spin();
    protected:
        ros::NodeHandle nh_;

        ros::Publisher marker_pub;

        geometry_msgs::Point pnt;

        bool set_marker (visualization_msgs::Marker &);
};

#endif  /*NAV_STEP_*/