/* shared_controller.h
 *
 *  Created on: 09.05.2017
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Department of Computer Science and Control Systems
 */

#ifndef SHARED_CONTROLLER_
#define SHARED_CONTROLLER_

#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

#define MAX_VEL_LIN 0.2
#define MAX_VEL_ANG 0.5
#define MIN_VEL_ANG 0.02 //just for algorithm
#define MIN_VEL_LIN 0.02

//for obstacle detector
#define MIN_X_RNG 0.25  //as BORD
#define MIN_Z_ANG 0.8

#define X_SIZE 0.27     //0.22
#define Y_SIZE 0.22    //0.185 
#define BORD 0.25
#define CELL 0.02
#define CELL_H 0.01
#define R_POSE 33
#define CELL_N 67
#define PI_SH 3.142

class Controller
{
    public:
        Controller(std::string);
        ~Controller();
        void spin();
    protected:
        ros::NodeHandle nh_;
        ros::Publisher cmd_vel_pub;
        ros::Publisher map_pub;
        ros::Subscriber cmd_vel_sub;
        //ros::Subscriber map_sub;
        ros::Publisher marker_pub;

        void cmd_vel_cb(const geometry_msgs::Twist &);
        //void map_cb(const nav_msgs::OccupancyGrid &);

        struct Frame {
            float x;
            float y;
            //Frame(float par_x, float par_y) : x(par_x), y(par_y) {}
        };

        //граница робота с учетом безопасной зоны в 5 см.
        Frame base[8];  //clock-wise

        bool set_marker (visualization_msgs::Marker &);
        //int8_t map[CELL_N][CELL_N];     //MAP
/*        class Map_keeper 
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
        };*/


};

#endif  /*SHARED_CONTROLLER_*/