/* shared_controller_node.cpp
 *
 *  Created on: 09.05.2017
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Department of Computer Science and Control Systems
 */

 #include <shared_controller.h>

 int main(int argc, char** argv)
 {
     ros::init(argc, argv, "shared_controller");
     Controller shared_controller(ros::this_node::getName());
     shared_controller.spin();

     return 0;
 }
