#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <math.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
                                                                MoveBaseClient;
/*
    List of goals excluding the first goal set before while loop.
    Goals are composed of x, y, z coordinates for positioning and a
    Quaternion for orientation.
*/
double goalList[4][7] = {
  {27.53, -6.76, 0, 0, 0, 0.68,  0.73},
  {27.13, 1.1,   0, 0, 0, 1,    -0.02},
  {13.02, 0.183, 0, 0, 0, -0.7,  0.71},
  {13.31, -3.4,  0, 0, 0, -0.66, 0.75}
};
int cnt = 0;
double epsilon = 0.3;

/*
    Function that switches the vehicles current goal on the map.
    Inputs: the simple action client handle, reference to the goal variables
            that needs to be written and sent to the action server,
            preempting the former goal.
    The list of goals is global and thus always available to the function.
*/
void switchGoal(MoveBaseClient &ac, move_base_msgs::MoveBaseGoal &goal){
    //Return from function if 4th goal has already been reached
    if(cnt > 3) return;
    //Command the action server to stop tracking the former goal.
    ac.stopTrackingGoal();
    //Write the new goal to the goal variable.
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x    = goalList[cnt][0];
    goal.target_pose.pose.position.y    = goalList[cnt][1];
    goal.target_pose.pose.position.z    = goalList[cnt][2];
    goal.target_pose.pose.orientation.x = goalList[cnt][3];
    goal.target_pose.pose.orientation.y = goalList[cnt][4];
    goal.target_pose.pose.orientation.z = goalList[cnt][5];
    goal.target_pose.pose.orientation.w = goalList[cnt][6];
    ROS_INFO("Sending Goal [%d]!", cnt+1);
    //Send goal to the action server and increment goal count by one.
    ac.sendGoal(goal);
    cnt++;
}

using namespace std;
int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  tf::TransformListener listener;
  double xPos,yPos, distance;

  MoveBaseClient ac("move_base", true);
  while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
  }
  bool a = (ac.isServerConnected());
  cout << "Server conneceted: [%d]" << a << endl;

  move_base_msgs::MoveBaseGoal goal;

  // First goal to be sent to the action server.
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 13.7;
  goal.target_pose.pose.position.y = -7.4;
  goal.target_pose.pose.position.z = 0;
  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = 0;
  goal.target_pose.pose.orientation.w = 1;
  ROS_INFO("Sending 1st goal");
  ac.sendGoal(goal);
  while(ros::ok()){
          /*
              Get tf listener for transform from base_footprint to map
              coordinates. Subsequently calculate distance between current
              position and current goal on map.
          */
          listener.waitForTransform("/map", "/base_footprint",
                                    ros::Time(0), ros::Duration(10.0));
	        tf::StampedTransform transform;
          try
          	{
          		listener.lookupTransform("/map", "/base_footprint",
                                        ros::Time(0), transform);
          		xPos = transform.getOrigin().x();
          		yPos = transform.getOrigin().y();
          		ROS_INFO("Current position: (%f, %f, )\n", xPos,yPos);
          	}
          	catch(tf::TransformException &ex)
          	{
          		ROS_ERROR("%s", ex.what());
          		ros::Duration(1.0).sleep();
          }
          //Calculate euclidean distance to current goal
          distance = sqrt(pow((goal.target_pose.pose.position.x - xPos),2)
                          + pow((goal.target_pose.pose.position.y - yPos),2));
          ROS_INFO("Distance to goal: [%f]", distance);
          /*
              Check wether distance to goal is within acceptable bounds
              (epsilon determined in experiments) or wether the state of the
              action server evalutes to SUCCEDED, thus indicating that the goal
              was reached. If true switch to the next goal.
          */
          if((distance < epsilon) ||
                (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)){
                switchGoal(ac, goal);
                ROS_INFO("SWITCHING!");
            }
  }
  ros::spin();
  return 0;
}
