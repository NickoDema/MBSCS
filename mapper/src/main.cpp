/* main.cpp
 *
 *  Created on: 19.04.2017
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Department of Computer Science and Control Systems
 */

 #include <mapper.h>

 int main(int argc, char** argv)
 {
     ros::init(argc, argv, "mapper");
     Mapper robotino_mapp(ros::this_node::getName());
     robotino_mapp.spin();

     return 0;
 }
