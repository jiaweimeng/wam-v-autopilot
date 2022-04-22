#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float32.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sstream>
#include <vector>
#include <math.h>
#include <cstddef>
#include <iostream>
//#include <unistd.h>
#include <ctime>
#include "PID.h"

// Head files used to subscribe visualization_msgs::Marker information
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#define success false
#define running true

PID::PID() {}

PID::~PID() {}

bool status = running;

using namespace std; 

ros::Subscriber wamv_state_pub;
geometry_msgs::Pose pose;

ros::Subscriber final_path_sub;
geometry_msgs::Point point;
double path_x[1000];
double path_y[1000];

vector<double> toEulerAngle(const double x,const double y,const double z,const double w)
{
    vector<double> eularAngle;
    double roll, pitch, yaw;

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (w * x + y * z);
    double cosr_cosp = +1.0 - 2.0 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);
    eularAngle.push_back(roll);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);
    eularAngle.push_back(pitch);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (w * z + x * y);
    double cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);
    eularAngle.push_back(yaw);    

    return eularAngle;
}

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

void wamvStatesCallback(const gazebo_msgs::ModelStates &wamv_state_current)
{
    int wamv_index = -1;    
    std::vector<std::string> model_names = wamv_state_current.name;

    for(size_t i = 0; i < model_names.size(); i++)
    {
        if(model_names[i] == "wamv")   
            wamv_index = i;
    }
    pose = wamv_state_current.pose[wamv_index];
    //eularOrientation = toEulerAngle(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    //ROS_INFO_STREAM("Position:" << std::endl << pose);
    //ROS_INFO_STREAM("Eular Orientation:" << std::endl << eularOrientation[0]);
    //ROS_INFO_STREAM("Eular Orientation:" << std::endl << eularOrientation[1]);
    //ROS_INFO_STREAM("Eular Orientation:" << std::endl << eularOrientation[2]);
}

void finalPathCallback(const visualization_msgs::Marker & markers)
{
	if(markers.ns == "path_planner" && markers.id == 2)
	{
		//ROS_INFO_STREAM("Marker_information:" << std::endl << markers);
		for(size_t i = 0; i < markers.points.size(); i++)
		{
			//ROS_INFO_STREAM("Waypoints_num:" << std::endl << markers.points.size());
        		path_x[i] = markers.points[i].x *10;
			path_y[i] = markers.points[i].y *10;
			//ROS_INFO_STREAM("markers.points[i].x:" << std::endl << markers.points[i].x);
			//ROS_INFO_STREAM("markers.points[i].y:" << std::endl << markers.points[i].y);
		}
	}
	for(size_t j = 0; j < 100; j++)
	{
		ROS_INFO_STREAM("path_x[j]:" << std::endl << path_x[j]);
		ROS_INFO_STREAM("j:" << std::endl << j);
	}
}

//PID functions
void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  i_error = 0.0;
  p_error = 0.0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  i_error += cte;
  d_error = cte - p_error; //it can be initialized with CTE value because the simulator is responsive only after 2 cycles
  p_error = cte;
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double total_error;
  total_error = Kp * p_error + Kd * d_error + Ki * i_error;
  //std::cout<<"The composition of steering angle is: "<<std::endl;
  //std::cout<<"Kp "<< Kp <<" * p_error "<<p_error<<" + Kd "<< Kd <<" * d_error " << d_error <<"= "<<total_error<<std::endl;
  return total_error;  // TODO: Add your total error calc here!
}

int main(int argc, char **argv)
{
	PID pid_steering, pid_speed;
	pid_steering.Init(1.5,0.0,12.5); //This is the tuned paratmeter; please do not change!
	pid_speed.Init(2.5, 0.00, 2.0); //This is the tuned paratmeter; please do not change! P = 2.5; I = 0.05; D = 1.7.  Another option: P = 2.5; I = 0.0; D = 2.0
	
	//double path_x[] = {};
	//double path_y[] = {};
	double velocity = 3.0; //Average moving speed of the USV
	
	double distance, current_time, previous_time, time_step, previous_position_x, previous_position_y, travelled_distance, current_velocity;
	float waypoint_theta;

	vector<double> eularOrientation;

	ros::init(argc, argv, "waypoints_publisher");
	ros::NodeHandle n;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	//Subscribing the position of wamv
	ros::Subscriber wamv_state_sub = n.subscribe("/gazebo/model_states", 100, wamvStatesCallback);

	//Subscribing the waypoints on finalpath from gpmp2_node
	ros::Subscriber final_path_sub = n.subscribe("/path_planner_gpmp2", 100, finalPathCallback);

	//while(1); //Delete this later as it is just used for the second callback complie

	std_msgs::Float32 usv_x, usv_y;

	//Publishing and controlling the turnning and moving forward of wamv
	ros::Publisher left_thrust_cmd_pub = n.advertise<std_msgs::Float32>("/wamv/thrusters/desired_left_thrust_cmd", 100);
	ros::Publisher right_thrust_cmd_pub = n.advertise<std_msgs::Float32>("/wamv/thrusters/desired_right_thrust_cmd", 100);
	ros::Publisher left_thrust_angle_pub = n.advertise<std_msgs::Float32>("/wamv/thrusters/desired_left_thrust_angle", 100);
	ros::Publisher right_thrust_angle_pub = n.advertise<std_msgs::Float32>("/wamv/thrusters/desired_right_thrust_angle", 100);

	ros::Publisher current_velocity_pub = n.advertise<std_msgs::Float32>("/wamv/current_velocity", 100);
	ros::Publisher desired_velocity_pub = n.advertise<std_msgs::Float32>("/wamv/desired_velocity", 100);
	ros::Publisher waypoint_orientation_pub = n.advertise<std_msgs::Float32>("/wamv/waypoint_orientation", 100);
	ros::Publisher current_orientation_pub = n.advertise<std_msgs::Float32>("/wamv/current_orientation", 100);
	
	ros::Publisher usv_x_pub = n.advertise<std_msgs::Float32>("/wamv/current_position/x", 100);
	ros::Publisher usv_y_pub = n.advertise<std_msgs::Float32>("/wamv/current_position/y", 100);
	ros::Publisher next_waypoint_x_pub = n.advertise<std_msgs::Float32>("/wamv/next_waypoint_x/x", 100);
	ros::Publisher next_waypoint_y_pub = n.advertise<std_msgs::Float32>("/wamv/next_waypoint_y/y", 100);

	sleep(1);
	
	//ROS_INFO_STREAM("Position:" << std::endl << pose);

	for(int i = 0; i < sizeof(path_x); i++)
	{
		//Change the orientation of the propeller
		bool correct_position = true;
		while(correct_position)
		{
			//Control the orientation
			eularOrientation = toEulerAngle(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w); 
			
			//Calculate the angle between the usv's current orientation and the direction of the line which connects the current position of usv and the next waypoint
			if( (path_y[i] > pose.position.y) && (path_x[i] > pose.position.x) )
			{
				waypoint_theta = acos((sqrt((path_x[i] - pose.position.x) * (path_x[i] - pose.position.x)))/(sqrt((pose.position.y - path_y[i])*(pose.position.y - path_y[i]) + 			(path_x[i] - pose.position.x) * (path_x[i] - pose.position.x))));
			}
			if( (path_y[i] > pose.position.y) && (path_x[i] < pose.position.x) )
			{
				waypoint_theta = M_PI - acos((sqrt((path_x[i] - pose.position.x) * (path_x[i] - pose.position.x)))/(sqrt((pose.position.y - path_y[i])*(pose.position.y - 				path_y[i]) + (path_x[i] - pose.position.x) * (path_x[i] - pose.position.x))));
			}
			if( (path_y[i] < pose.position.y) && (path_x[i] > pose.position.x) )
			{
				waypoint_theta = -acos((sqrt((path_x[i] - pose.position.x) * (path_x[i] - pose.position.x)))/(sqrt((pose.position.y - path_y[i])*(pose.position.y - path_y[i]) 				+ (path_x[i] - pose.position.x) * (path_x[i] - pose.position.x))));
			}
			if( (path_y[i] < pose.position.y) && (path_x[i] < pose.position.x) )
			{
				waypoint_theta = acos((sqrt((path_x[i] - pose.position.x) * (path_x[i] - pose.position.x)))/(sqrt((pose.position.y - path_y[i])*(pose.position.y - path_y[i]) + 			(path_x[i] - pose.position.x) * (path_x[i] - pose.position.x)))) - M_PI;
			}
			if( (path_y[i] == pose.position.y) && (path_x[i] > pose.position.x) )
			{
				waypoint_theta = 0;
			}
			if( (path_y[i] == pose.position.y) && (path_x[i] < pose.position.x) )
			{
				waypoint_theta = M_PI;
			}
			if( (path_y[i] > pose.position.y) && (path_x[i] == pose.position.x) )
			{
				waypoint_theta = 0.5*M_PI;
			}
			if( (path_y[i] < pose.position.y) && (path_x[i] == pose.position.x) )
			{
				waypoint_theta = -0.5*M_PI;
			}

			std_msgs::Float32 msg_next_waypoint_x;
			std_msgs::Float32 msg_next_waypoint_y;
			msg_next_waypoint_x.data = path_x[i];
			msg_next_waypoint_y.data = path_y[i];
			next_waypoint_x_pub.publish(msg_next_waypoint_x);
			next_waypoint_y_pub.publish(msg_next_waypoint_y);

			//ROS_INFO_STREAM("Waypoint_orientation:" << std::endl << waypoint_theta);
			//ROS_INFO_STREAM("Current_orientation:" << std::endl << eularOrientation[2]);

			std_msgs::Float32 msg_wo;
			msg_wo.data = waypoint_theta;			
	    		waypoint_orientation_pub.publish(msg_wo);

			std_msgs::Float32 msg_co;
			msg_co.data = eularOrientation[2];			
	    		current_orientation_pub.publish(msg_co);	


			//PID controller for angle
			double cte = eularOrientation[2] - waypoint_theta;		//The error in PID controller
			double steer_value;

			pid_steering.UpdateError(cte);					//Update the error of PID
			steer_value = pid_steering.TotalError();

			if (steer_value > 1.0) 
			{
          			steer_value = 1.0;
          		}
          		if (steer_value < -1.0) 
			{
            			steer_value = -1.0;
          		}

			std_msgs::Float32 msg_angle;
			msg_angle.data = steer_value;

			left_thrust_angle_pub.publish(msg_angle);
			right_thrust_angle_pub.publish(msg_angle);

			//ROS_INFO_STREAM("previous_position_x:" << std::endl << previous_position_x);
			//ROS_INFO_STREAM("previous_position_y:" << std::endl << previous_position_y);
			//ROS_INFO_STREAM("pose.position.x:" << std::endl << pose.position.x);
			//ROS_INFO_STREAM("pose.position.y:" << std::endl << pose.position.y);

			//Control the velocity
			travelled_distance = sqrt((previous_position_x - pose.position.x)*(previous_position_x - pose.position.x) + (previous_position_y - pose.position.y)*(previous_position_y - pose.position.y));				//Calculate the travelled distance

			previous_position_x = pose.position.x;
			previous_position_y = pose.position.y;

			current_time = clock();
			time_step = 5*(current_time - previous_time)/CLOCKS_PER_SEC;	//Calculate the time step; the scaler '5' is used to offset the difference between PC running time realistic time
			previous_time = current_time;

			current_velocity = travelled_distance/time_step;   		//Calculate the current velocity
			//ROS_INFO_STREAM("travelled_distance:" << std::endl << travelled_distance);
			//ROS_INFO_STREAM("time_step:" << std::endl << time_step);

			//ROS_INFO_STREAM("current_velocity:" << std::endl << current_velocity);
			std_msgs::Float32 msg_cv;
			msg_cv.data = current_velocity;			
	    		current_velocity_pub.publish(msg_cv);

			//ROS_INFO_STREAM("desired_velocity:" << std::endl << velocity);
			std_msgs::Float32 msg_dv;
			msg_dv.data = velocity;			
	    		desired_velocity_pub.publish(msg_dv);
			
	
			//PID controller for velocity
			double cte_1 = velocity - current_velocity;				//The error in PID controller
			double steer_value_1;
			std_msgs::Float32 msg_cmd;

			pid_speed.UpdateError(cte_1);						//Update the error of PID
			steer_value_1 = pid_speed.TotalError();

			//if (steer_value_1 > 5.0) 
			//{
          			//steer_value_1 = 5.0;
          		//}
          		//if (steer_value_1 < 0.25) 
			//{
            			//steer_value_1 = 0.25;
          		//}

			msg_cmd.data = steer_value_1;

	    		left_thrust_cmd_pub.publish(msg_cmd);
	    		right_thrust_cmd_pub.publish(msg_cmd);

			//ROS_INFO_STREAM("path_theta:" << std::endl << path_theta);
			//ROS_INFO_STREAM("Eular Orientation_z:" << std::endl << eularOrientation[2]);
			distance = sqrt((path_x[i] - pose.position.x)*(path_x[i] - pose.position.x) + (path_y[i] - pose.position.y)*(path_y[i] - pose.position.y));
			//ROS_INFO_STREAM("Distance:" << std::endl << distance);

			usleep(100000);

			usv_x.data = pose.position.x;		
			usv_y.data = pose.position.y;	
			usv_x_pub.publish(usv_x);
			usv_y_pub.publish(usv_y);

            		//Arrive at the target position? 
	    		if(distance < 5)
	    		{
				correct_position = false;
				ROS_INFO_STREAM("TARGET REACHED!" << std::endl);
	    		}

		}
		//Break the path tracking controller when it has passed the goal point
		if(path_x[i] == 0 && path_y[i] == 0)
		{
			break;
		}

	}
	sleep(20);
	ros::waitForShutdown();
	return 0;
}
