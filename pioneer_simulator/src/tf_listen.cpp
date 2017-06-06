#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalID.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

#include <tf/transform_listener.h>

#include <iostream>
#include "std_msgs/Bool.h"



int main(int argc, char **argv){

 ros::init(argc, argv, "tf_listen");
    
      ros::NodeHandle node;
   
tf::TransformListener listener;
	tf::StampedTransform transform;
	
	ros::Rate loop_rate(100);
	
	while(ros::ok()){
		
		
		
		
		try{
         listener.lookupTransform("Alfa/map", "Alfa/base_link_true", ros::Time(0), transform);
       }
       catch (tf::TransformException &ex) {
         ROS_ERROR("%s",ex.what());
         ros::Duration(1.0).sleep();
         continue;
       }


std::cout<<transform.getOrigin().y()<<std::endl;
std::cout<<transform.getRotation().z()<<std::endl;

ros::spinOnce();
	loop_rate.sleep();
	
	
}

}
