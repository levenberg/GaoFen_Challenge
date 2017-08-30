

This project is for Gaofen challenge 2017 using M100. The hardware of m100 was updated to v 1.3.10. Guidance-SDK-ROS haven't been upgrated. Using the obstacle distance to keep the height of M100 for the unreliable height measurement from M100.

There are four round main code for the challenge, if the mission type is set false, the main_1.cpp is used, and it is for round 1-Line following, gaofen_1.launch is for round 1;

If the mission type is set true, the main_2.cpp is used, and it is for round 2-human tracking, gaofen_2.launch is used for round 2.

For the final challenge, if the mission type is set false, the main_3.cpp is used, and it is for round 3-UAV break through.
If  the mission type is set true, the main_4.cpp is used, and it is for round 4-UAV collaboration.

For the vision pacakge(dji_sdk_read_cam), the video processing thread trackLoop function is the core function to process images. For round 1 and round 2:
if(tracker->m_mission_type==false)  //line follow
{
	 tracker->Line_detection(gray, result);
	 //ROS_INFO("Line following");
}
if(tracker->m_mission_type==true)  //human follow
{
      tracker->processImage ( gray );
      // ROS_INFO("Human tracking");
}


For the mible sdk app, the current version is on https://github.com/luckykelfor/DJI_M100_DEV.git, only one botton need to be clicked-"d":  request_sdk_permission_control and start the task.

For different tasks, you only need to source the bash and roslaunch gaofen_*.launch file.
# GaoFen_Challenge

