#include "std_msgs/String.h"
#include <math.h>
#include "sstream"
#include <iostream>
#include <utility>
#include <ros/ros.h>
#include <pses_basis/SensorData.h>
#include <pses_basis/Command.h>
#include <pses_basis/CarInfo.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <functional>

/*
    Function to calculate the average of an array of type double
      Input:  Array of values, size of that array
      Output: Average value of values in input array
*/
double calcAvg(double arr[], int size){
  double sum = 0;
  for (int i = 0; i < size; i++) {
      sum += arr[i];
  }
  return (sum/double(size));
}
//Subscriber Callbacks
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg,
                                            nav_msgs::Odometry* odom){
    *odom = *msg;
}

void sensorCallback(const pses_basis::SensorData::ConstPtr& sens_msg,
                                      pses_basis::SensorData *sensors){
  *sensors = *sens_msg;
}

void carInfoCallback(const pses_basis::CarInfo::ConstPtr& carInfo_msg,
                                          pses_basis::CarInfo *carInfo){
  *carInfo = *carInfo_msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "segfault_wallfollower");
  ros::NodeHandle nctrl;
  ros::Rate loop_rate(40);
  //Declare and Initialize variables
  pses_basis::Command cmd;
  pses_basis::SensorData sensors;
  sensor_msgs::Imu imu;
  nav_msgs::Odometry odom;
  pses_basis::CarInfo carInfo;
  /*
    Control variables:
      yr        := current distance to right wall
      wr        := desired distance to right wall
      ur        := output of controller to be passed to steering
      Kopt      := matrix of optimal values for k1, k2
  */
  float yr = 0;
  float yl = 0;
  float yrOld = 0;
  float yaw = 0;
  float yawOld = 0;
  float angvZ = 0;
  float angvZ_diff;
  float wr = 0.4;
  float wl = 0.25;
  float wphi = 0.0;
  float ur = 0;
  float er, el, erI;
  float erDiff = 0;
  float elDiff = 0;
  float erOld = 0;
  float elOld = 0;
  float urBoundCheck = 0;
  int count = 0;
  float yawOffset = 0;
  float Ki = 0.005;
  float Kd = 0.7;
  float Kopt[] ={1.4142 , 0.6098};
  const float PI = 3.14159265;
  int idx = 0;
  float yr_avg = 0;
  double y[] = {0,0,0,0,0,0,0,0,0,0};
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
  float dt = 1.f/40.f;

  //Pubisher and Subscriber function calls
  ros::Publisher commands_pub =
            nctrl.advertise<pses_basis::Command>("pses_basis/command", 10);
  ros::Subscriber odom_sub =
            nctrl.subscribe<nav_msgs::Odometry>("odom",50, std::bind(odometryCallback, std::placeholders::_1, &odom));
  ros::Subscriber sensor_sub =
          nctrl.subscribe<pses_basis::SensorData>("pses_basis/sensor_data", 10, std::bind(sensorCallback, std::placeholders::_1, &sensors));
  ros::Subscriber carInfo_sub =
          nctrl.subscribe<pses_basis::CarInfo>("pses_basis/car_info",10, std::bind(carInfoCallback, std::placeholders::_1, &carInfo));

  while(ros::ok())
    {
      //Start driving forwards
      cmd.motor_level = 7;

      // Read sensor values
      yr = sensors.range_sensor_right;
      yl = sensors.range_sensor_left;
      yaw = carInfo.yaw;
      /*
          Set yaw Offset according to measured changes in yaw:
          if yaw has changed more than 90° either direction a turn has been made
          thus a new reference yaw is computed using yawOffset
      */
      // In case of past left turn
      yaw += yawOffset;
      if( ((yaw - yawOffset) > PI/2.f) && (fabs(yr_avg - wr) < 0.5) ){
          yawOffset -= PI/2.f;
      }
      // In case of past right turn
      else if( ((yaw - yawOffset) < -PI/2.f) && (fabs(yr_avg - wr) < 0.5)){
          yawOffset += PI/2.f;
      }
      /*
          Compute average of distance to wall over last 10 timesteps:
            New distance measurements are written to the y[] array to be used
            in computation of an average each timestep, if the index of this
            array should be out of bounds it is reset. Oldest entry of array
            is overwritten at each timestep.
      */
      (idx > 9) ? (idx = 0) : (y[idx] = yr);
      idx++;
      yr_avg = calcAvg(y, 10);
      // compute control error
      el = wl - yl;
      er = wr - yr;
      /*
          Computation of integral control error
          Low-pass filter to prevent integration of too much noise:
            The threshold value (0.1) has been choosen after some testing.
      */
      erDiff = er - erOld;
      (erDiff > 0.1) ? (erI += er*dt) : (erI = erI);
      /*
          Stops vehicle if front ultra-sound sensors
          detects obstacle at less than 15cm distance,
          also resets error integration as well as sensor history.
      */
      if(sensors.range_sensor_front <= 0.15){
        erI = 0;
        yaw = 0;
        yrOld = 0;
        cmd.motor_level = 0;
        cmd.steering_level = 0;
      }
      /*
          If front sensor detects obstacle and more vacant space is to the
          left than to the right: steers sharply left
      */
      else if( (sensors.range_sensor_front < 0.9) &&
                              (sensors.range_sensor_front != 0) && (yl > yr) ){
        ur = 50;
        cmd.steering_level = ur;
	    }
      /*
          Same as above but in case more space is to the right.
      */
      else if( (sensors.range_sensor_front < 0.9) &&
                              (sensors.range_sensor_front != 0) && (yr > yl) ){
        ur = -50;
        cmd.steering_level = ur;
	    }
      else{
          /*
              Control law: P-component of distance to wall and yaw angles
              originate from linear-quadratic regulator design; additional
              integral and differential control for distance to wall.
              Arctan because of linearized Ackermann kinematics during
              controller design.
          */
          ur = atan(Kopt[0]*er + Kopt[1]*(wphi-yaw) + Ki*erI + Kd*erDiff/dt);
          /*
              Convert control output to deegres since polynomial maps angle in
              degrees to steering_level values.
          */
          ur = ur * (180.f/PI);
          //Steers right if control < 0.
          if( ur < 0 ){
            urBoundCheck =
                    polyR4[0]*pow(fabs(ur),4)+polyR4[1]*pow(fabs(ur),3)
                    +polyR4[2]*pow(fabs(ur),2)+polyR4[3]*fabs(ur)+polyR4[4];
            if(urBoundCheck <  -50){
              cmd.steering_level =  -50;
            }else{
              cmd.steering_level = urBoundCheck;
            }
          }
          //Steers left if control > 0.
          else if( ur > 0 ){
            urBoundCheck =
                    polyL4[0]*pow(fabs(ur),4)+polyL4[1]*pow(fabs(ur),3)
                    +polyL4[2]*pow(fabs(ur),2)+polyL4[3]*fabs(ur)+polyL4[4];
            if(urBoundCheck >  50){
              cmd.steering_level =  50;
            }else{
              cmd.steering_level = urBoundCheck;
            }
          }

      }
      /*
          Save distance to wall and control error of this timestep to use in
          subsequent timesteps.
      */
      yrOld = yr;
      erOld = er;
      // Publish motor_level and steering_level commands
      commands_pub.publish(cmd);
      ros::spinOnce();
      loop_rate.sleep();
    }
  ros::spin();
  return 0;
}
