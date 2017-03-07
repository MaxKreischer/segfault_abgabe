#include "std_msgs/String.h"
#include <math.h>
#include "sstream"
#include <iostream>
#include <utility>
#include <ros/ros.h>
#include <pses_basis/Command.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <functional>

//Subscriber Callbacks
void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg,
     geometry_msgs::Twist *cmd_vel){
            *cmd_vel = *cmd_vel_msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "segfault_nav_ctrl");
  ros::NodeHandle nstack_ctrl;
  ros::Rate loop_rate(40);
  //Declaration and Initialization of variables
  pses_basis::Command cmd;
  geometry_msgs::Twist cmd_vel;
  /*
        Maximal values for steering and motor level
        in accordance with vehicle specs; bias term for errors in steering
  */
  const float max_motor_level = 20.f;
  const float min_motor_level = -20.f;
  const float max_steering_level = 50.f;
  const float min_steering_level = -50.f;
  const float bias = -4.f;
  float speed_level = 0.f;
  double PI = 3.14159;
  double velCmd = 0;
  double yawCmd = 0;
  float BoundCheck = 0;
  /*
        Polynomial Interpolation for right and left steering direction
          --> maps from angles in deg to steering_lvl:

        polyL4 := Polynomial-fit for steering to the left:
          4th order | 3rd order | 2nd order | 1st order | 0th order
          0.0010      -0.0355     0.3800      0.3266      0.0071

        polyR4 := Polynomial-fit for steering to the right:
        4th order | 3rd order | 2nd order | 1st order | 0th order
        -0.0003     0.0102      -0.1148     -1.0710     -0.0381
  */
  float polyL4[] = {0.0010, -0.0355, 0.3800, 0.3266, 0.0071};
  float polyR4[] = {-0.0003, 0.0102, -0.1148, -1.0710, -0.0381};

  //Publisher for cmd.steering_level and cmd.motor_level
  ros::Publisher commands_pub =
  nstack_ctrl.advertise<pses_basis::Command>("pses_basis/command", 10);

  //Subscriber for cmd_vel output of navigation stack
  ros::Subscriber cmd_vel_sub =
  nstack_ctrl.subscribe<geometry_msgs::Twist>
   ("cmd_vel", 10, std::bind(cmd_velCallback, std::placeholders::_1, &cmd_vel));

   while(ros::ok())
    {
        //Gets linear and angular control velocities from navigation stack
        velCmd = (double)cmd_vel.linear.x;
        yawCmd = (double)cmd_vel.angular.z;
        /*
              Approx. max. vel.: 2 metres/second @ motor_level: 20
              thus metric velocity values linear with motor Level
              at a slope of 10 motor_level per 1 metre/second.
        */
        speed_level = std::ceil(velCmd * 10.f);
        cmd.motor_level =
        (speed_level > max_motor_level) ? (max_motor_level) : (speed_level);

        cmd.motor_level =
        (speed_level < min_motor_level) ? (min_motor_level) : (speed_level);
        /*
              Check if min. motor_level to overcome friction is reached;
              otherwise set motor_level to the necessary value.
              Also checks if maxima of motor level are exceeded.
        */
      	if((cmd.motor_level < 6) && (cmd.motor_level > 0)){
      	    	cmd.motor_level = 5;
        }
      	if((cmd.motor_level > -12) && (cmd.motor_level < 0)){
      	    	cmd.motor_level = -12;
        }

        //Converts steering angle to degrees
        yawCmd = yawCmd * (180.f/PI);

        BoundCheck = std::ceil(yawCmd * 2.0);
        if(BoundCheck > 50){
        	cmd.steering_level = 50;
        }else if(BoundCheck < -50){
        	cmd.steering_level = -50;
        }else{
        	cmd.steering_level = BoundCheck;
			    cmd.steering_level -= 4;
        }
        /*
              Steers to the right if angle is negative otherwise to the left.
              Respective polynomials are used to translate steering angles
              in degree to steering level. Also checks if maxima of steering
              are exceeded.
        */
        if( yawCmd < 0 ){
            BoundCheck =
                polyR4[0]*pow(fabs(yawCmd),4)+polyR4[1]*pow(fabs(yawCmd),3)
                +polyR4[2]*pow(fabs(yawCmd),2)+polyR4[3]*fabs(yawCmd)+polyR4[4];
            if(BoundCheck <  -50){
                cmd.steering_level =  -50;
            }else{
                cmd.steering_level = BoundCheck;
            }
        }
        else if( yawCmd >= 0 ){
            BoundCheck =
                polyL4[0]*pow(fabs(yawCmd),4)+polyL4[1]*pow(fabs(yawCmd),3)
                +polyL4[2]*pow(fabs(yawCmd),2)+polyL4[3]*fabs(yawCmd)+polyL4[4];
            if(BoundCheck >  50){
                cmd.steering_level =  50;
            }else{
                cmd.steering_level = BoundCheck;
            }
        }
        /*
            Outputs velocity and yaw commands and their Respective
            motor_level and steering_level.
        */
        ROS_INFO("cmd_vel:velocity: [%f]; cmd_vel:yaw: [%f]",
                                                                 velCmd,yawCmd);
        ROS_INFO("Motor Level: [%d]; Steering_Level: [%d]",
                                           cmd.motor_level, cmd.steering_level);
        //Publish motor_level and steering_level
        commands_pub.publish(cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }
  ros::spin();
  return 0;
}
