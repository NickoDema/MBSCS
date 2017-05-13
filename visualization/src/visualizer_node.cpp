/* vizualizer_node.cpp
 *
 *  Created on: 16.04.2017
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Department of Computer Science and Control Systems
 */

 #include <visualizer.h>

 int main(int argc, char** argv)
 {
     ros::init(argc, argv, "visualizer");
     Visualizer robotino_viz(ros::this_node::getName());
     robotino_viz.spin();

     return 0;
 }
