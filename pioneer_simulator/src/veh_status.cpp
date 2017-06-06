#include "ros/ros.h"
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Bool.h"



int goalStatusFlag=0;
bool vehMoving=0;

void subStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& status){
	
	if (!status->status_list.empty()){
	
	goalStatusFlag = status->status_list[0].status;
}

	else goalStatusFlag=1;
	
	
	
}
	
	
void subCmdVelCallback(geometry_msgs::Twist msg){

 if (msg.linear.x==0 && msg.linear.y==0 && msg.linear.z==0 && msg.angular.x==0 && msg.angular.y==0 && msg.angular.z==0) vehMoving=0;
 else vehMoving=1;
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "veh_status");
	
	ros::NodeHandle n;
	
	ros::Subscriber subCmdVel=n.subscribe("Alfa/cmd_vel",1, subCmdVelCallback);
	ros::Subscriber subStatus=n.subscribe("move_base/status", 1, subStatusCallback);
	
	ros::Publisher pubStatus=n.advertise<std_msgs::Bool>("Alfa/vehMissionStatus",1);
	
	ros::Rate loop_rate(100);
	
	std_msgs::Bool msg;
	
	while(ros::ok()){
	
	
	
	if ( (goalStatusFlag==3 || goalStatusFlag==8 || goalStatusFlag==1 || goalStatusFlag==2) && vehMoving==0 ) {
		msg.data=1;}
		
	else { 
		msg.data=0;}
		
	pubStatus.publish(msg);
	std::cout<<msg<<std::endl;
	
	ros::spinOnce();
	loop_rate.sleep();
	
	}
} 
	
