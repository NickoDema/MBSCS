/* visualizer.h
 *
 *  Created on: 16.04.2017
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Department of Computer Science and Control Systems
 */

#ifndef VIZ_
#define VIZ_

#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <cmath>

#define X_SIZE 0.21
#define Y_SIZE 0.20
#define BORD 0.05
#define BORD_2 0.1

class Visualizer
{
    public:
        Visualizer(std::string);
        ~Visualizer();
        void spin();
    protected:
        ros::NodeHandle nh_;

        enum m_type
        {
            point  = 0x01,
            line_s = 0x02,      //line strips
            lise_l = 0x04,      //line list
            // ++++
        };

        ros::Publisher marker_pub;

        geometry_msgs::Point pnt;

        bool set_marker (visualization_msgs::Marker &,m_type);
};

#endif  /*VIZ_*/