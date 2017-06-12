#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <std_msgs/String.h>
#include <sstream>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <math.h>

sensor_msgs::LaserScan scanResult;
bool startMappingBool = 0;

int mapWidth = 500;
int mapLength = 2000;
double localMap[500][2000];

int rBeam[500];
float angle[3];

int sensorRange=5;
float x[5*100*3], y[5*100*3]; //sensorrange *100 * 3
int sizex=sensorRange*100*3;

//for inverseSensorModel
float Rmax=2500; //maksimalni domet sonara
float rov = 500; // radijus vidljivosti sonara
float th3db = 0.0218; //kut pola sirine zrake
float pE = 0.4; // donja granica vjerojatnosti nezauzetosti
float pO = 0.6; // gornja granica vjerojatnosti zauzetosti

float deltark = 1;//podrucje u kojem mjerenje sonara poprima srednju vrijednost



void readerCallback(sensor_msgs::LaserScan scaner){
	std::cout<<"scan je stigao"<<std::endl;
	 scanResult=scaner;
	 if (startMappingBool == 0) startMappingBool=1;
		
}

void startMappingCallback(std_msgs::Bool msg){
	
	if (msg.data==1) startMappingBool = 1; //remove prom readercallback
	
}


void printaj(){
	
	
for(int j=0;j<mapLength;j++){
	for(int i=0;i<mapWidth;i++){
		
			
			std::cout<<localMap[i][j]<<"\t";
		}
	}
		
}

void initData(){
int i;
	for(i=0;i<mapWidth;i++){
		for(int j=0;j<mapLength;j++){
			
			localMap[i][j]=floor(255*0.5);
		}
		
	}
	
	for(i=0; i<sensorRange*100; i++) rBeam[i]=i+1; //5m range
		
	angle[0] = -1/8 *3.14/180;
	angle[1] = 0;
	angle[2] = 1/8 *3.14/180;
	
	
	
	for (int i=0; i<100*sensorRange; i++){
		
		x[i] = cos(angle[0]) * rBeam[i] + 15; //+tf sen-rob
		y[i] = sin(angle[0]) * rBeam[i];
		
		x[i+sensorRange*100] = cos(angle[1]) * rBeam[i] + 15; //+tf sen-rob
		y[i+sensorRange*100] = sin(angle[1]) * rBeam[i];
		
		x[i+sensorRange*100] = cos(angle[2]) * rBeam[i] + 15; //+tf sen-rob
		y[i+sensorRange*100] = sin(angle[2]) * rBeam[i];
		
	}
	
		
	for (i=0; i<100*sensorRange;i++){
		  std::cout<<x[i]<<"   ";
	  }
std::cout<<std::endl;
for (i=0; i<100*sensorRange;i++){
		  std::cout<<y[i]<<"   ";
	  }
	  	
}


void writePGM()
{
    FILE *pgmFile;
    int i, j;
    int hi, lo;
 
    pgmFile = fopen("localMap.pgm", "wb");
    if (pgmFile == NULL) {
        perror("cannot open file to write");
        exit(EXIT_FAILURE);
    }
 
    fprintf(pgmFile, "P5 ");
    fprintf(pgmFile, "%d %d ", mapWidth, mapLength);
    fprintf(pgmFile, "%d ", 255);

    
        for (i = 0; i < mapWidth; ++i)
            for (j = 0; j < mapLength; ++j) {
				
                lo = floor(localMap[i][j]);
                fputc(lo, pgmFile);
            }
    
 
    fclose(pgmFile);
}

float inverseSensorModel(float r, float ro, float th)
{
	r=r*100;
	float dro = 1-(1+tanh(2*(ro-rov)))/2;
	float alpha;
	float P;
	
	if (abs(th)>=0 && abs(th)<=th3db) alpha = 1-pow((th/th3db),2);
	else alpha = 0; 
	
	//std::cout<<ro<<"           "<<r<<"           "<<deltark<<"           "<<dro<<"           "<<std::endl;
	if (ro < r-2*deltark)
		P = 0.5 + (pE-0.5)*alpha*dro; 
	else if ((ro >= r-2*deltark) && (ro < r-deltark))
		P = 0.5 + (pE-0.5)*alpha*dro*(1-pow((2+(ro-r)/deltark),2)); 
	else if ((ro >= r-deltark) && (ro < r+deltark))
		P = 0.5 + (pO-0.5)*alpha*dro*(1-pow(((ro-r)/deltark),2)); 
	else
		P = 0.5; 
		
	return P;
}

void updateMap(){
	
	//std::cout<<"updatemap"<<std::endl;
	//lok u glob???
	
	float xcell[sizex], ycell[sizex];
	
	 int i;
	 //std::cout<<"updatemap2"<<std::endl;
	 //std::cout<<"updatemap2"<<std::endl;
	 
	
	 for (i=0; i<sizex;i++){
		 
		// std::cout<<i<<"       "<<std::endl;
		 
	 xcell[i] = ceil(y[i]); 
	 ycell[i] = ceil(x[i]); 
	  
	 if(xcell[i]<1)   xcell[i]=1;
	 if(xcell[i]>500)  xcell[i]=500;
     
    
     if(ycell[i]<1)   ycell[i]=1;
	 if(ycell[i]>2000)  ycell[i]=2000;
	 }
	 
	 //std::cout<<"updatemap3"<<std::endl;
	 ///////////////////////////////////
	  //for (i=0; i<sizex;i++){
		  //std::cout<<ycell[i]<<"   ";
	  //}
	  
	  
	  ///////////////////////
	 int j=0,k=0;
	 int a,b, senNum;
	  
	 for (senNum=360; senNum<1080 ; senNum++){
	 int flags[mapWidth][mapLength]={{0}};
	 //std::cout<<senNum<<"     senzor  "<<std::endl;
	 
     j=0;k=0;
	 for(i=0; i<sizex; i++){
		 //std::cout<<"updatemap5"<<std::endl;
		 a=xcell[i-1];
		 b=ycell[i-1];
		 
		 //std::cout<<a<<"  polje a     "<<std::endl;
		 
		 //std::cout<<flags[a][b]<<"  polje flags    "<<std::endl;
		 
		 //std::cout<<i<<"  polje i     "<<std::endl;
		 //std::cout<<j<<"  j   "<<std::endl;
		 if(flags[a][b] == 0){ flags[a][b]=1;
		 
		 //std::cout<<flags[a][b]<<"  polje flags    "<<std::endl;
		 
		 //std::cout<<scanResult.ranges[senNum]<<"  scanResult.ranges[senNum]   "<<std::endl;
		 //std::cout<<j<<"  j   "<<std::endl;
		 //std::cout<<rBeam[j]<<"  rBeam[j]    "<<std::endl;
		 //std::cout<<angle[i%3]<<"  angle[i%3]   "<<std::endl;
		 float P = inverseSensorModel(scanResult.ranges[senNum], rBeam[j], angle[k]);
		 
		 //std::cout<<P<<"  peeeee    "<<std::endl;
		 
		 //std::cout<<a<<"  polje a     "<<std::endl;
		 //std::cout<<b<<"  polje b     "<<std::endl;
		 //std::cout<<localMap[1][1]<<"  ono    "<<std::endl;
		 float lk = std::log(P/(1-P))+ std::log(localMap[a][b]/(1-localMap[a][b]));
         
         //std::cout<<localMap[a][b]<<"  ono    "<<std::endl;
         
         localMap[a][b]=1/(1+exp(-lk));
		 
		 //std::cout<<localMap[a][b]<<"  mapaaa    "<<std::endl;
	 }
		 if(i%3==0 && i>2) j++;
		 k=i%3;
		 
	 }	
	}
	
}
int main (int argc, char **argv) {
	
	ros::init(argc, argv, "localMap");
	
	ros::NodeHandle n;
	
    ros::Subscriber reader=n.subscribe("/Alfa/scan", 1, readerCallback);
   //ros::Subscriber startMapping=n.subscribe("/Alfa/scan", 1, startMappingCallback);

// TODO TF odometry za položaj početka
	
	ros::Rate loop_rate(1000);
	
	
	while(ros::ok()){
		
		//std::cout<<startMappingBool;
		initData();
		
		std::cout<<"-------------------------------------------------------------------------------"<<std::endl;
	
	
		if(startMappingBool){
		
		updateMap();	
			
		}
		//printaj();
		writePGM();
	
	
	ros::spinOnce();
	loop_rate.sleep();
	
}}
