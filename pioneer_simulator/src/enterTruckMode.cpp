#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib_msgs/GoalID.h>
#include <math.h>
//#include <geometry_msgs/Twist>
#include <tf/tf.h>

#include <tf/transform_listener.h>

#include <iostream>

float maxRampDist=1; //maximum distance to downed ramp thats is higher that robots position


bool scanON=0;
sensor_msgs::LaserScan scanResult;
bool scanRecieved=0;
float r=1;

ros::Publisher cmdVelPub;


float lastDist=0;
float lastDistDiff=0;
float totalDist=0;

float expDist=2; //distance to cross util you are sure its type 3 truck


struct truckProperties {
int type;

float LGx;
float LGy;

float RGx;
float RGy;

float inSpeed;
float outSpeed;

float length;
float width;
};

struct truckProperties truck;


void truckPropInit(){
	truck.type=-1; //undefined
	truck.inSpeed=0;
	truck.outSpeed=0;
	}
	
	
void adjustTruckProp(){
	
	 if(truck.type==4){
		 truck.inSpeed = 1;
		 truck.outSpeed = 0.1;
		 }
		 
	 if(truck.type==-2){
		 truck.inSpeed = 0.5;
		 truck.outSpeed = 0.5;
		 }
		 
	if(truck.type==2){
		 truck.inSpeed = 0.1;
		 truck.outSpeed = 1;
		 }
		 
	if(truck.type==3){
		 truck.inSpeed = 0.5;
		 truck.outSpeed = 0.5;
		 }
		 
	  }
	  
	  
void readerCallback(sensor_msgs::LaserScan scaner){
	
	if (scanON) {scanResult=scaner;
		scanRecieved=1;
	}
}


	

void checkScan(){
	float midDist =scanResult.ranges[1440/2]; //get distance from the middle of the truck (ramp or end of truck)
	
	if(truck.type==-1 && midDist<(r+maxRampDist)) truck.type=4;
	else truck.type=-2;  //it is type 2 or 3 
	
	std::cout<<"Checking scan"<<std::endl;
	}
	

void goInTruck(ros::Publisher *pub){
	
	geometry_msgs::Twist msg;
	
	msg.linear.x=truck.inSpeed;
	std::cout<<scanResult.ranges[1440/2+360*1/18]<<std::endl;
	std::cout<<scanResult.ranges[1440/2-360*1/18]<<std::endl;
	
	if (scanResult.ranges[1440/2+360*1/18] > scanResult.ranges[1440/2-360*1/18]) msg.angular.z=-0.05;
	else if (scanResult.ranges[1440/2+360*1/18] < scanResult.ranges[1440/2-360*1/18]) msg.angular.z=0.05	;
	else msg.angular.z=0;
	
	pub->publish(msg);
	
	std::cout<<"Go in the truck"<<std::endl;
	// idi do r ili do sredine rampe 
	
//na osnovu lokacije doking postaje može se ići prema njenoj sredini!

//ili sami završeta prethodne akcije da se odredi položaj pred sredinom kamiona što jedino ne bi odgovaralo slučaju 4 >> time bi se moglo prvo checkScan a zatim kopirati iz prošlog programa
//dio za scan da se dobiju koordinate trucka
}

void stop(ros::Publisher *pub){
	geometry_msgs::Twist msg;
	
	msg.linear.x=0;
	msg.angular.z=0;
	
	pub->publish(msg);	
	
	std::cout<<"stop"<<std::endl;
}

bool checkDist(){
	
	std::cout<<"Checking distance"<<std::endl;
	float midDist =scanResult.ranges[1440/2];
	float distDiff;
	
	
	if(lastDist!=0){
		
		
		distDiff = (lastDist-midDist);
		totalDist=totalDist+distDiff;
		
		if (distDiff > 1.2*lastDistDiff) {
			truck.type=2;
			
			return 1;
		}
		
		if (totalDist>expDist){
			truck.type=3;
			
			return 1;
			}
		
		
		
		if (-distDiff > 1.2*lastDistDiff) {
			truck.type=4;
			
			return 1;
		}
		
	}
	
	
	
	
	lastDist=midDist;
	lastDistDiff=distDiff;
	return 0;
}


int main(int argc, char **argv){
	
	ros::init(argc, argv, "enterTruckMode");
	
	ros::NodeHandle n;
	
	ros::Subscriber reader=n.subscribe("/Alfa/scan", 1, readerCallback);
	
	ros::Publisher cmdVelPub=n.advertise<geometry_msgs::Twist>("Alfa/cmd_vel",1);
	
	ros::Rate loop_rate(100);
	
	
	
	tf::TransformListener listener;
	tf::StampedTransform transform;
	
	
	
	truckPropInit();
	scanON=1;
	
	while(ros::ok()){
	std::cout<<truck.type<<std::endl;	
		
		
		
		//try{
        //listener.lookupTransform("Alfa/map", "Alfa/base_link_true", ros::Time(0), transform);
       //}
       //catch (tf::TransformException &ex) {
         //ROS_ERROR("%s",ex.what());
         //ros::Duration(1.0).sleep();
         //continue;
       //}
       
       //std::cout<<transform.getOrigin().y()<<std::endl;
	   //std::cout<<transform.getRotation().z()<<std::endl;

      
	
	//start going in truck
	
	if (scanRecieved) {
		checkScan();
		adjustTruckProp();
	
	}
	
	
	
	if (truck.type==-2){
	goInTruck(&cmdVelPub);
	
	if(checkDist()) {
		adjustTruckProp();
		stop(&cmdVelPub);
	}}
	
	
	if (truck.type>1 && scanResult.ranges[1440/2]>1){
		goInTruck(&cmdVelPub);
	
	}
	else stop(&cmdVelPub);
	
	
	ros::spinOnce();
	loop_rate.sleep();
	}
	
	}
