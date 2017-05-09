/* mapper.h
 *
 *  Created on: 19.04.2017
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Department of Computer Science and Control Systems
 */

#ifndef MAPPER_
#define MAPPER_

#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <cmath>

#define CELL 0.02       //Cell boundary size
#define CELL_H 0.01
#define CELL_N 51       //Maps width and height
#define R_POSE 25       //Robot pose on map_


class Mapper
{
    public:
        Mapper(std::string);
        ~Mapper();
        void spin();
    protected:
        ros::NodeHandle nh_;
        ros::Publisher map_pub;
        ros::Subscriber odom_sub;

        geometry_msgs::Pose last_pose;      //??? static

        void odom_cb(const nav_msgs::Odometry&);
        void odom_init();

        //int8_t map[CELL_N][CELL_N];     //MAP
        class Map_keeper 
        {
            public:
            double x_error;
	        double y_error;
            Map_keeper(int);
            int8_t map_[CELL_N][CELL_N];     //MAP
            void move(int, char);
        } map_keeper;
        class Map_builder
        {
            protected:
            struct Point       
 		    {
     		    double x, y;
     		    Point *Next,*Prev;
 		    };
            Point *Head, *Tail;
            double max_dist;
            public:
            Map_builder(double, double);
            ~Map_builder();
            void add(double, double);
            void div_by_two();
            void to_map(double,double,double,int8_t [CELL_N][CELL_N]);
            double get_dist(Point *, Point *);
        };


};

#endif  /*MAPPER_*/