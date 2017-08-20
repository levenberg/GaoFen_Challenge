#include <ros/ros.h>
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

//#include<geometry_msgs/PoseWithCovarianceStamped.h>

#include<std_msgs/Int8.h>
#include<std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>
#include<math.h>
#include<fstream>

#define GIMBAL_USED
#define filter_N 4   
#define EPS 0.0000000001
#define USE_OBDIST  
   
#define PI 3.1415926536
#define VEL_MODE

ofstream writeF ( "/root/log.txt");
using namespace DJI::onboardSDK;
using namespace actionlib;
static bool Detection_fg = false;
int delay_count = 300;
static uint8_t land_mode = 0;   
static int cbring_mode = 0;
static float last_flight_x = 0.0;
static float last_flight_y = 0.0;
static float last_flight_yaw = 0.0;//not used yet
static float ob_distance[5]= {10.0};
float flying_height_control_tracking=0.0;
float searching_height = 1.0;

bool parking(float &cmd_fligh_x,float &cmd_flight_y,float height, uint8_t detection_flag,DJIDrone* drone);
bool cbarrier(float &cmd_fligh_x,float &cmd_flight_y, float &cmd_yaw, float height,float set_height, uint8_t detection_flag,DJIDrone* drone,int &delayCount);

void tag_detection_resultCallback ( const std_msgs::Bool & num_of_detection )
{
    Detection_fg = num_of_detection.data;
    //ROS_INFO ( "%d tag(s) are detected",numOfDetections );
}

/*obstacle_distance from guidance*/
/*0-down, 1-forward,2-right,3-backward,4-left*/

void obstacle_distance_callback ( const sensor_msgs::LaserScan & g_oa )
{
    for ( int i=0; i<5; i++ )
        ob_distance[i]=g_oa.ranges[i];

    //ROS_INFO("1: %f 2:%f 3:%f 4: %f 5:%f",ob_distance[0],ob_distance[1],ob_distance[2],ob_distance[3],ob_distance[4]);
}
float sum ( float a[],size_t len )
{
    float sum = 0;
    for ( int i = 0; i<len; i++ )
    {
        sum += a[i];
    }
    return sum;
}

float find_min ( float a[], size_t len )
{
    float min = a[0];

    for ( int  j = 1; j<len; j++ )
    {
        if ( a[j]<min )
            min = a[j];

    }

    return min;
}

float find_max ( float a[], size_t len )
{
    float max = a[0];

    for ( int  j = 1; j<len; j++ )
    {
        if ( a[j]>max )
            max = a[j];

    }
    return max;
}



int main ( int argc, char **argv )
{
    //Some params

    int max_takeoff_waitcount = 700;
    double tracking_flight_height = 2.0;
    double descending_height_delta = 0.005;
    double initial_descending_height_at_search = 10.0;
    double initial_searching_height=1.5;

    ros::init ( argc, argv, "sdk_client" );
    ROS_INFO ( "sdk_service_client_test" );


    ros::NodeHandle node_priv ( "~" );
    node_priv.param<int> ( "delayCount",delay_count,100 );
    node_priv.param<double> ( "initTrackingHeight",tracking_flight_height,2.0 );
    node_priv.param<double> ( "descendSpeed",descending_height_delta,0.005 );
    node_priv.param<double> ( "initDescedingHeightSearch",initial_descending_height_at_search,10.0 );
    node_priv.param<int> ( "maxTakeoffWaitCount",max_takeoff_waitcount,700 );
    node_priv.param<double>("initial_searching_height",initial_searching_height,1.7);
    ros::NodeHandle nh; 
    DJIDrone* drone = new DJIDrone ( nh );

    writeF<<ros::Time::now() <<endl;
    //virtual RC test data

    ros::Publisher  mission_type_pub = nh.advertise<std_msgs::Bool> ( "dji_sdk_demo/mission_type",10 );  


    ros::Subscriber tag_detections_sub = nh.subscribe ( "tag_detection/detections",100,tag_detection_resultCallback );
    ros::Subscriber obstacle_distance_sub = nh.subscribe ( "/guidance/obstacle_distance",10,obstacle_distance_callback );

    //USED_APRILTAG_TYPE
    ros::Publisher start_searching_pub = nh.advertise<std_msgs::Bool> ( "/dji_sdk_demo/start_searching",10 );
    ros::Publisher state_in_mission_pub = nh.advertise<std_msgs::Int8>("/dji_sdk_demo/state_in_mission",10);

    flying_height_control_tracking = tracking_flight_height;

 
    ros::Rate loop_rate ( 50 );


    //float flight_x_filtered = 0.0;
    // std_msgs::Float64 filtered_x_msg,not_filtered_x_msg;
    float filtered_x=0.0,filtered_y=0.0, filtered_yaw=0.0;
    //filtered_x_msg.data = 0.0;
    float yaw=0;
    // not_filtered_x_msg.data = 0.0;

    //For filtering;
    float filter_seq_x[filter_N]= {0},filter_seq_y[filter_N]= {0};


    float initial_DeltaX, initial_DeltaY,DeltaX,DeltaY,initial_speedingup_height;
    float last_x_tag, last_y_tag;
    drone->gimbal_angle_control ( 0,-900,0,20 ); //Head down at the beginning.

    int time_count=0;
    int state_in_mission = 0;
    std_msgs::Int8 state_msg;
    state_msg.data = state_in_mission;
    float start_yaw = 0.0;

    std_msgs::Bool start_searching;
    start_searching.data= false;
    bool flip_once = false;
    
    /******************************************IMPORTANT*************************************************/
    std_msgs::Bool mission_type;
    mission_type.data=false; //false for round 3, true for round 4
 

    while ( ros::ok() )
    {
        ros::spinOnce();

	if(drone->rc_channels.gear==-10000)
	{
	  state_in_mission=0;
	}
        for ( int i = 0; i< filter_N-1; i++ )
        {
            filter_seq_x[i] = filter_seq_x[i+1];
            filter_seq_y[i] = filter_seq_y[i+1];
        }

        filter_seq_x[filter_N-1] = drone->flight_x;
        filter_seq_y[filter_N-1] = drone->flight_y;
	last_flight_yaw = filtered_yaw;

        filtered_x =  drone->flight_x;//( sum ( filter_seq_x,filter_N )-find_max ( filter_seq_x,filter_N )-find_min ( filter_seq_x,filter_N ) ) / ( filter_N-2 );

        filtered_y =  ( sum ( filter_seq_y,filter_N )-find_max ( filter_seq_y,filter_N )-find_min ( filter_seq_y,filter_N ) ) / ( filter_N-2 );
	
	filtered_yaw= drone->flight_yaw;
	if(abs(filtered_yaw-last_flight_yaw)>50.0)   filtered_yaw = last_flight_yaw;
	if(filtered_yaw>30) filtered_yaw=30;
        if(filtered_yaw<-30) filtered_yaw=-30;

        // if start_searching=1, follow line
        start_searching_pub.publish ( start_searching );
	mission_type_pub.publish ( mission_type );
	
	//TODO:IF obstacle is the horizontial level, how to use obstacle distance to avoid. Maybe it already used.
	switch ( drone->transparentdata.data.at ( 0 ) )
	{
	  
	  /*****************************************************'Q' : TAKE OFF**********************************************/
	  /********************************************************************************************************************/
	  case 'q':
	  {
	    ROS_INFO ( "drone status: %d", drone->flight_status);
	    //writeF<<"take off"<<drone->local_position.z<<endl;
	    //cannot use flying_height_control_tracking for second time flight.
	    //drone->gimbal_angle_control ( 0,0,0,20 ); //Head down at the beginning.
	    //drone->landing();
	    drone->attitude_control ( 0x9B,0,0,tracking_flight_height,0 );
	    flying_height_control_tracking = tracking_flight_height;
	    start_yaw = drone->yaw_from_drone;//Record the yaw of taking off	

	    break;
	  }
	  /************************************************'d' START LINE FOLLOW MISSION**********************************************************************/
	  /*****************************************************************************************************************/
	  case 'd':  
	  {
	    ROS_INFO ( "In Mission" );
	    //writeF<<"In Mission, height="<<ob_distance[0]<<"state= "<<state_in_mission<<endl;
	    writeF<<"state_in_mission="<<state_in_mission<<endl;
	    
	    state_msg.data = state_in_mission;
	    state_in_mission_pub.publish(state_msg);
	    ROS_INFO("state_in_mission= %d",state_in_mission);
	     switch (state_in_mission) 
	     {
	       case 0: //for parking pad 2
		 flip_once = parking(filtered_x,filtered_y,tracking_flight_height,Detection_fg,drone);
		 if(flip_once)
		 {
		   state_in_mission=1;
		   flip_once=false;
		 }
		 break;
	       case 1:  //for crossing circle 3
		 flip_once= cbarrier(filtered_x,filtered_y,filtered_yaw,1.2,2.1,Detection_fg,drone,time_count);
		 if(flip_once)
		 {
		   state_in_mission=2; 
		   flip_once=false;
		 }
		 break;
	       case 2:  //for parking pad 4
		 flip_once = parking(filtered_x,filtered_y,tracking_flight_height,Detection_fg,drone);
		 if(flip_once)
		 {
		   state_in_mission=3;
		   flip_once=false;
		 }
		 break;
	       case 3: //for crossing circle 5
		 flip_once= cbarrier(filtered_x,filtered_y,filtered_yaw,1.4,2.3,Detection_fg,drone,time_count);
		 if(flip_once)
		 {
		   state_in_mission=4;
		   flip_once=false;
		 }
		 break;
	       case 4:  //for crossing circle 6
		 flip_once= cbarrier(filtered_x,filtered_y,filtered_yaw,1.6,2.5,Detection_fg,drone,time_count);
		 if(flip_once)
		 {
		   state_in_mission=5;
		   flip_once=false;
		 }
		 break;
	       case 5: //for parkinng pad 7
		 flip_once = parking(filtered_x,filtered_y,tracking_flight_height,Detection_fg,drone);
		 if(flip_once)
		 {
		   state_in_mission=6;
		   flip_once=false;
		 }
		 break;
	       case 6: //for parking pad 8
		 flip_once = parking(filtered_x,filtered_y,tracking_flight_height,Detection_fg,drone);
		 if(flip_once)
		 {
		   state_in_mission=7;
		   flip_once=false;
		 }
		 break;
	       case 7: //for crossing circle 9
		 flip_once= cbarrier(filtered_x,filtered_y,filtered_yaw,1.8,2.7,Detection_fg,drone,time_count);
		 if(flip_once)
		 {
		   state_in_mission=8;
		   flip_once=false;
		 }
		 break;
	       case 8: //for parking pad 10
		 flip_once = parking(filtered_x,filtered_y,tracking_flight_height,Detection_fg,drone);
		 if(flip_once)
		 {
		   state_in_mission=100;
		   flip_once=false;
		 }
		 break;
	       case 100:
		 drone->landing();
		 break;
	       default:
		 break;
	     }
	    
	    break; 
	  } 
	  
	  /*****************************************************'a' for abort mission***************************************************************************/
	  /********************************************************************************************************************************************/
	  case 'a':
	    ROS_INFO ( "abort mision" );
	    writeF<<"abort mision"<<drone->local_position.z<<endl;
	    // for ( int i = 0; i<200; i++ )
	    drone->attitude_control ( 0x9B,0,0,drone->local_position.z,0 );
	    // while ( !drone->release_sdk_permission_control() )
	    
	    drone->release_sdk_permission_control();
	    //usleep ( 10000 );
	    break;
	    
	  default:
	      ROS_INFO ( "Waithing for command from mobile!\n" );
	    break;
	}
	//ROS_INFO ( "Command code is %c",drone->transparentdata.data.at ( 0 ) );
	loop_rate.sleep();
	
    }
    writeF.close();
    return 0;
}

bool parking(float &cmd_fligh_x,float &cmd_flight_y,float height, uint8_t detection_flag,DJIDrone* drone)
{
  drone->gimbal_angle_control ( 0,-900,0,20 );
  
  if(cmd_fligh_x>0.8) cmd_fligh_x=0.8;
  else if(cmd_fligh_x<-0.8) cmd_fligh_x=-0.8;
  
  if(cmd_flight_y>0.8) cmd_flight_y=0.8;
  else if(cmd_flight_y<-0.8) cmd_flight_y=-0.8;
  
  if(land_mode==0&&drone->gimbal.pitch<-85)   //closing to landing part and landing
  {
    if(detection_flag)  //parking pad detected
    {
      if(abs(cmd_fligh_x)>0.16||abs(cmd_flight_y)>0.16)  //closing and keep height
      {
	if ( ob_distance[0]<height-0.1)
	{
	  flying_height_control_tracking += 0.003;
	}
	else if ( (ob_distance[0]>height+0.1)&&ob_distance[0]<10)
	{ 
	  flying_height_control_tracking -= 0.003;
	}
	drone->attitude_control(0x9B,cmd_fligh_x,cmd_flight_y,flying_height_control_tracking,0);
      }
      else if(abs(cmd_fligh_x)<0.16&&abs(cmd_flight_y)<0.16) 
      {
	if ( ob_distance[0]<1.4)
	{
	  flying_height_control_tracking += 0.005;
	}
	else if ( (ob_distance[0]>1.6)&&ob_distance[0]<10)
	{ 
	  flying_height_control_tracking -= 0.005;
	}
	drone->attitude_control(0x9B,cmd_fligh_x,cmd_flight_y,flying_height_control_tracking,0);   //TODO: height adjusting
	if(ob_distance[0]<1.6)
	{
	  drone->landing();
	  land_mode=1;
	  sleep(1);
	}
      }
    }
    else    //parkinng pad undetected
      ;//TODO:   for different tag, do different pre-moving to detect tag
  } 
  else if(land_mode==1)   //automatic takeoff
  {
    //sleep(4);
    if(drone->flight_status == 1)  //1: standby; 2: take off, 3: in air; 4:landing, 5: finish landing
    drone->takeoff();
    flying_height_control_tracking=1.2;
    //drone->attitude_control(0x9B,0,0,flying_height_control_tracking,0);
    if(drone->flight_status == 3&&ob_distance[0]>0.5&&ob_distance[0]<10)   //TODO: adjusting using the land pad.
    {
      land_mode=0;
      return true;
    }
  }
  
  return false;
}

bool cbarrier(float &cmd_fligh_x,float &cmd_flight_y, float &cmd_yaw,float height, float set_height, uint8_t detection_flag,DJIDrone* drone,int &delayCount)
{
  if(cbring_mode==0)  //turn the gimbal direction
  {
    drone->gimbal_angle_control(0,0,0,20);   //look farward.
    if ( ob_distance[0]<height-0.1)
    {
      flying_height_control_tracking += 0.002;
    }
    else if ( (ob_distance[0]>height+0.1)&&ob_distance[0]<10)
    { 
      flying_height_control_tracking -= 0.002;
    }
    drone->attitude_control ( 0x9B,0,0,flying_height_control_tracking,0);
    if(drone->gimbal.pitch>-5&&abs(ob_distance[0]-height)<0.5)
      cbring_mode=1; 
    writeF<<"decending for tag detection, pitch="<<drone->gimbal.pitch<<",height="<<ob_distance[0]<<endl;
  }
  else if(cbring_mode==1)  //closing to the hanging pad. first x,y then z
  {
    writeF<<"mode: "<<cbring_mode<<endl;
    //cmd_fligh_x-0.5 is the safe distance of drone to the pad, adjusting
    if ( ob_distance[0]<height-0.1)
    {
      flying_height_control_tracking += 0.001;
    }   
    else if ( (ob_distance[0]>height+0.1)&&ob_distance[0]<10)
    { 
      flying_height_control_tracking -= 0.001;
    }
    if(detection_flag)
    {
      if(cmd_flight_y>0.15)  cmd_flight_y = 0.15;
      if(cmd_flight_y<-0.15) cmd_flight_y =-0.15;
      if(abs(cmd_yaw)<5)
	drone->attitude_control ( 0x9B,0.2,cmd_flight_y,flying_height_control_tracking,cmd_yaw);
      else
	drone->attitude_control ( 0x9B,0.0,cmd_flight_y,flying_height_control_tracking,cmd_yaw);
      writeF<<"tag detected, cmd_yaw="<<cmd_yaw<<endl;
    }
    else   
    {
      drone->attitude_control ( 0x9B,0,0,flying_height_control_tracking,0);
      writeF<<"tag NOT detected, cmd_yaw="<<cmd_yaw<<endl;   //yaw is not zero for no tag, need to be fixed.
    }
    if(abs(cmd_fligh_x-1.4)<0.2&&abs(cmd_flight_y)<0.2&&abs(cmd_yaw)<5)
      cbring_mode=2;
  }
  else if(cbring_mode==2)   //go up
  {
    writeF<<"mode: "<<cbring_mode<<endl;
    if ( ob_distance[0]<set_height-0.1)
    {
      flying_height_control_tracking += 0.003;
    }
    else if ( (ob_distance[0]>set_height+0.1)&&ob_distance[0]<10)
    {
      flying_height_control_tracking -= 0.003;
    }
    drone->attitude_control ( 0x9B,cmd_fligh_x-1.4,0,flying_height_control_tracking,0);
    
    if(abs(set_height-ob_distance[0])<0.2)
      cbring_mode=3;
  }
  else if(cbring_mode==3)  //cross
  { 
    writeF<<"mode: "<<cbring_mode<<", dtime="<<delayCount<<endl;
    if ( ob_distance[0]<set_height-0.1)
    {
      flying_height_control_tracking += 0.001;
    }
    else if ( (ob_distance[0]>set_height+0.1)&&ob_distance[0]<10)
    { 
      flying_height_control_tracking -= 0.001;
    }
    drone->attitude_control ( 0x9B,1,0,flying_height_control_tracking,0);  //go farward
    delayCount++;
    //sleep(10);
    //if(detection_flag==0) 
    //   drone->attitude_control ( 0x9B,1,0,flying_height_control_tracking,0);  //go farward
    //else
    if(delayCount>delay_count)
    {
      delayCount=0;
      return true; 
    }
  }
  
  return false;
}
