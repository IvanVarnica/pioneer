#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalID.h>
#include <math.h>
#include <geometry_msgs/PoseStamped.h>

#include <iostream>

float gate_x1=40.794, gate_x2=44.844, gate_y1=20.534, gate_y2=20.595, r=2;
float goal1, goal2;

void readerCallback(sensor_msgs::LaserScan scaner){
	
	std::cout<<"ok    ";
	float d, startAngle;
	
	float leftRange, rightRange, leftAngle, rightAngle, gateRx,gateLx,gateRy,gateLy;
	
	
	d=sqrt(pow(gate_x2-gate_x1,2)+pow(gate_y2-gate_y1,2));
	
	startAngle=acos((d/2)/(r*r+pow(d/2,2)))+3.14*0.1;
	
	int i0=ceil(startAngle/2/3.14*1440);
	
	
	int laserCounts=0;
	int truckLaser[1440]={0};
	float diagonal=sqrt(r*r+pow(d/2,2)) ;
	int i;
	std::cout<< diagonal <<"      "<< d<<"    "<<std::endl;
	for(i=i0; i<=scaner.ranges.size()-i0;i++){
		//std::cout<< diagonal <<"      "<< i<<"    "<<scaner.ranges[i]<<std::endl;
		if (scaner.ranges[i]<diagonal && scaner.ranges[i]>=r){
			
				truckLaser[laserCounts++]=i;
				std::cout<<truckLaser[laserCounts-1]<<std::endl;
				}
			}
			
			//std::cout<<truckLaser;
			
			
	int start=truckLaser[0];
	int j=0;
	i=0;
	
	//for(i=0; i<laserCounts;i++){
		std::cout<<"ok   3 ";
		
		
		//kreni iz sredine prema rubovima
		while((start+j)==truckLaser[i]) {
			j++;
			i++;};
		
		
		 rightRange= (scaner.ranges[start]+scaner.ranges[i-1])/2;
		 rightAngle= (start+truckLaser[i-1])*3.14/1440;
		
		
		std::cout<<" R start was   "<<start<<"  R end was  "<<truckLaser[i-1]<<"        "<<rightRange<<"        "<<rightAngle<< 	std::endl;
		
		i--;
		j=0;
		start=truckLaser[i];
		
		
		while((start+j)==truckLaser[i]) {
			j++;
			i++;};
		
		 leftRange= (scaner.ranges[start]+scaner.ranges[i-1])/2;
		 leftAngle= (start+truckLaser[i-1])/3.14/1440;
	//}
	std::cout<<"ok    ";

		 gateRx=goal1+rightRange*cos(rightAngle);		
		 gateRy=goal2+rightRange*cos(rightAngle);	
		
		 gateLx=goal1+leftRange*cos(leftAngle);	
		 gateLy=goal2+leftRange*cos(leftAngle);
		
		std::cout<<rightAngle*180/3.14<<"        "<<leftAngle*180/3.14<<std::endl;
		
		std::cout<<gateRx<<"        "<<gateRy<<"        "<<gateLx<<"        "<<gateLy<<"        "<<std::endl;
				
   
}


void statusCallback(actionlib_msgs::GoalStatusArray state){
	
	//actionlib_msgs::GoalStatus status=state.status_list[0].status;
	//std::cout<<status<<std::endl;
	
	//std::cout<<typeid(state.status_list[0].status).name()<<std::endl;
	}
	
void coordCalc(float *x, float *y, float *phii){
	float x1, x2, y1, y2, x0, y0, k1, k2, r, phi;
	
	x2=44.844;
	x1=40.794;
	y2=20.534;
	y1=20.595;
	r=2;
	
	x0=(x1+x2)/2;
	y0=(y1+y2)/2;
	
	k1=(y2-y1)/(x2-x1);
	
	k2=-1/k1;
	
	phi=atan(k2);
	*x=x0+r*cos(phi);
	*y=y0+r*sin(phi);
	*phii=phi+3.14;	
}



int main(int argc, char **argv){
	
	ros::init(argc, argv, "reader");
	
	ros::NodeHandle n;
	
	ros::Subscriber reader=n.subscribe("/Alfa/scan", 1, readerCallback);
	ros::Subscriber status=n.subscribe("/move_base/status", 1, statusCallback);
	ros::Publisher goToGate=n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 2);
	
	float goal_x, goal_y, goal_phi;
	
	geometry_msgs::PoseStamped goal;
	
	ros::Rate loop_rate(1000);
	
	while(ros::ok()){
	coordCalc(&goal_x, &goal_y, &goal_phi);
	
	goal.header.frame_id="Alfa/map";
	goal.header.stamp = ros::Time::now();
	
	goal1=goal_x;
	goal2=goal_y;
	
	goal.pose.position.x=goal_x;
	goal.pose.position.y=goal_y;
	goal.pose.position.z=0;
	
	goal.pose.orientation.w=goal_phi;
	
	goToGate.publish(goal);
	
	
	//std::cout<<goal<<std::endl;
	
	
	ros::spinOnce();
	loop_rate.sleep();
	
}
}
	
