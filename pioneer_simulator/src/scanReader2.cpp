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


ros::Publisher goalPublish;

float gate_x1=40.794, gate_x2=44.844, gate_y1=20.534, gate_y2=20.595, r=2;
float goal1, goal2;

int state=1;
int goalStatusFlag;
bool scanON=0;
bool permision=0;

float gateRx=0;
float gateLx=0;
float gateRy=0;
float gateLy=0;



//tf::StampedTransform transform;


// Ideja je da na osnovu trenutnoga polozaja i danih koordinata gatea gdje kamion staje izracunati gdje da vozilo dođe da skenira prostor da pronađe polozaj kamiona te da dođe pred kamion
// 1. izracunaj polozaj pred gateom na koji treba doci
// 2. dodi na polozaj
// 3. pronadji kamion
// 4. izracunaj polozaj pred kamionom na koji treba doci
// 5. dodi na polozaj
// 6. turn off node

void readerCallback(sensor_msgs::LaserScan scaner){
	
	if (scanON){
	//std::cout<<"ok    ";
	float d, startAngle;
	
	float leftRange, rightRange, leftAngle, rightAngle;
	
	
	d=sqrt(pow(gate_x2-gate_x1,2)+pow(gate_y2-gate_y1,2));
	
	startAngle=acos((d/2)/(r*r+pow(d/2,2)))+3.14*0.1;
	
	int i0=ceil(startAngle/2/3.14*1440);
	
	
	int laserCounts=0;
	int truckLaser[1440]={0};
	float diagonal=sqrt(r*r+pow(d/2,2)) ;
	int i;
	int startLeft=0, startRight=0, nbrLeft, nbrRight;
	
	for(i=i0; i<=scaner.ranges.size()-i0;i++){
		
		if (scaner.ranges[i]<diagonal && scaner.ranges[i]>=r){
			
				truckLaser[laserCounts++]=i;
				
				if ( i > scaner.ranges.size()/2 && startLeft==0){
					startLeft = i;
					startRight = truckLaser[laserCounts-2];
					nbrLeft=laserCounts-1;
					nbrRight=laserCounts-2;
					}
				}
				}
			
				
	int j=0;
	i=0;
	

		
		//kreni iz sredine prema rubovima
		while((startRight-j)==truckLaser[nbrRight]) {
			
			
			j++;
			nbrRight--;
			std::cout<<scaner.ranges[truckLaser[nbrRight]]<<std::endl;
			};
		
		
		
		rightRange= scaner.ranges[truckLaser[nbrRight+2]];
		 rightAngle=truckLaser[nbrRight+2]*3.14/1440*2;
		
		std::cout<<"Right winner is "<<rightRange<<"at angle of " <<rightAngle*180/3.14-180<<std::endl;
		
		j=0;
		
		while((startLeft+j)==truckLaser[nbrLeft]) {
			j++;
			nbrLeft++;
			
			
			};
		
		 
		 leftRange= scaner.ranges[truckLaser[nbrLeft-2]];
		 
		 leftAngle= truckLaser[nbrLeft-2]*3.14/1440*2;
		 std::cout<<"Left winner is "<<leftRange<<"at angle of " <<leftAngle*180/3.14-180<<std::endl;
		 
	

		 gateRx=goal1+rightRange*cos(rightAngle-3.14-3.14/2);		//set yaw in place of -3.14/2
		 gateRy=goal2+rightRange*sin(rightAngle-3.14-3.14/2);	
		
		 gateLx=goal1+leftRange*cos(leftAngle-3.14-3.14/2);	
		 gateLy=goal2+leftRange*sin(leftAngle-3.14-3.14/2);
		//add transforms
		
		//gateRx= gateRx + transform.getOrigin().x()*cos( transform.getRotation().getAngle() );
		//gateRy= gateRy - transform.getOrigin().x()*sin( transform.getRotation().getAngle() );
		
		//gateLx= gateRx + transform.getOrigin().x()*cos( transform.getRotation().getAngle() );
		//gateLy= gateLy - transform.getOrigin().x()*sin( transform.getRotation().getAngle() );
		
		//std::cout<<rightAngle*180/3.14<<"        "<<leftAngle*180/3.14<<std::endl;
		
		std::cout<<gateRx<<"        "<<gateRy<<"        "<<gateLx<<"        "<<gateLy<<"        "<<std::endl;
		
		
   }
}


void subStatusCallback(std_msgs::Bool msg){
	
	if (msg.data==0) permision=1;
	else permision=0;
	
	}
	
void gateCoordCalc(float x1, float y1, float x2, float y2, float r, float *x, float *y, float *phii){
	float x0, y0, k1, k2, phi;
	
	//x2=44.844;
	//x1=40.794;
	//y2=20.534;
	//y1=20.595;
	//r=2;
	
	x0=(x1+x2)/2;
	y0=(y1+y2)/2;
	
	k1=(y2-y1)/(x2-x1);
	
	k2=-1/k1;
	
	phi=atan(k2);
	*x=x0+r*cos(phi);
	*y=y0+r*sin(phi);
	*phii=phi+3.14;	
}

void publishGoal (geometry_msgs::PoseStamped goal){
	
	//goalPublish.publish(goal);
	
}

int main(int argc, char **argv){
	
	ros::init(argc, argv, "reader");
	
	ros::NodeHandle n;
	
	ros::Subscriber reader=n.subscribe("/Alfa/scan", 1, readerCallback);
	ros::Subscriber subStatus=n.subscribe("Alfa/vehMissionStatus", 1, subStatusCallback);
	
	ros::Publisher goalPublish=n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 2);
	
	float goal_x, goal_y, goal_yaw;
	
	geometry_msgs::PoseStamped goal;
	
	ros::Rate loop_rate(1000);
	
	while(ros::ok()){
		
		//tf::TransformListener listener;
		
		
		//try{
         //listener.lookupTransform("Alfa/base_link_true","Alfa/laser", ros::Time(0), transform);
       //}
       //catch (tf::TransformException &ex) {
         //ROS_ERROR("%s",ex.what());
         //ros::Duration(1.0).sleep();
         //continue;
       //}
		
		//std::cout<< transform<<std::endl;
		////////////////////////////////////////////////////////////////////////
		//state=4;
		
		
		
		scanON=0;
		switch (state){
			
			case 1 :
			{
				gateCoordCalc(44.844,20.534, 40.794, 20.534,  2, &goal_x, &goal_y, &goal_yaw);
				goal.header.frame_id="Alfa/map";
				goal.header.stamp = ros::Time::now();
		
				goal1=goal_x;
				goal2=goal_y;
		
				goal.pose.position.x=goal_x;
				goal.pose.position.y=goal_y;
				goal.pose.position.z=0;
				
				tf::Quaternion goal_quat;
		
				goal_quat.setRPY(0, 0, goal_yaw);
		
				goal.pose.orientation.x = goal_quat.getX();
				goal.pose.orientation.y = goal_quat.getY();
				goal.pose.orientation.z = goal_quat.getZ();
				goal.pose.orientation.w = goal_quat.getW();
				
				//publishGoal(goal);
				goalPublish.publish(goal);
				
				std::cout<<goal<<std::endl;
				if (permision) break;
				
					std::cout<<"Gate goal has been published!"<<std::endl;
					state = 2;
					
				break;
			}
			
			case 2 :{
				if(!permision) state=3;
				break;}
				
				
				//skeniranje i trazenje koordinata kamiona
			case 3 :{
				scanON=1;
				
				if (gateLx!=0 && gateLy!=0 && gateRx!=0 && gateRy!=0) state=4;
				
				break;}
				
				
			case 4 :{
				
				gateCoordCalc(gateRx, gateRy, gateLx, gateLy, 0.5, &goal_x, &goal_y, &goal_yaw);
				goal.header.frame_id="Alfa/map";
				goal.header.stamp = ros::Time::now();
		
				goal1=goal_x;
				goal2=goal_y;
		
				goal.pose.position.x=goal_x;
				goal.pose.position.y=goal_y;
				goal.pose.position.z=0;
				
				tf::Quaternion goal_quat;
		
				goal_quat.setRPY(0, 0, goal_yaw);
		
				goal.pose.orientation.x = goal_quat.getX();
				goal.pose.orientation.y = goal_quat.getY();
				goal.pose.orientation.z = goal_quat.getZ();
				goal.pose.orientation.w = goal_quat.getW();
				
				//goalPublish.publish(goal);
				std::cout<<goal<<std::endl;
				if (permision) break;
				
					std::cout<<"Truck gate goal has been published!"<<std::endl;
					state = 6;
					
				break;}
				
				
				case 6 :{
				if(!permision) state=7;
				break;}
				
			
				
				
				case 7:{
					std::cout<<"Job is done"<<std::cout;
					break;}
				
		}
		
	
	
	
	
	
	
	

	
	
	//goToGate.publish(goal);
	
	
	//std::cout<<goal<<std::endl;
	
	
	ros::spinOnce();
	loop_rate.sleep();
	
}
}
	
