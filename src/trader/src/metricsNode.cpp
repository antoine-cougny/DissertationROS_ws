#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "trader/Task.h"
#include "trader/metrics.h"
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include "move_base_msgs/MoveBaseFeedback.h"

#include <iostream>
using namespace std;

geometry_msgs::Pose myPose;

void position_cb(const move_base_msgs::MoveBaseActionFeedback &msg)
{
    myPose = msg.feedback.base_position.pose;
}

/*
 * Returns the cartesian distance between the current position and the given
 * target.
 *  ! I am aware that a path may be longer but at the scale of our implementation
 *  this will be enough
 *
 *  @param: geometry_msgs::Pose currentPose, targetPose
 *  @return: flaot - norm 2
 */
float calcMovingCost(geometry_msgs::Pose currentPose,
                     geometry_msgs::Pose targetPose  )
{
    float result = pow(currentPose.position.x - targetPose.position.x, 2)
                  + pow(currentPose.position.y - targetPose.position.y, 2) 
                  + pow(currentPose.position.z - targetPose.position.z, 2);
    /* ROS_INFO("Current Position: %f, %f, %f", */ 
    /*             currentPose.position.x, */ 
    /*             currentPose.position.y, */ 
    /*             currentPose.position.z      ); */

    /* ROS_INFO("Target Position: %f, %f, %f", */ 
    /*             targetPose.position.x, */
    /*             targetPose.position.y, */
    /*             targetPose.position.z       ); */
    return sqrt(result);
}

bool metricsCalc_cb(trader::metrics::Request &Task, trader::metrics::Response &metrics)
{
    /* ROS_INFO("Got new position"); */
    metrics.cost = 10*calcMovingCost(myPose, Task.task.goalPosition_p)
                    + Task.task.toDo.waitTime;
    ROS_INFO("Cost: %.3f", metrics.cost);
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "metricsNode");
    ros::NodeHandle n;

    // Topic used by the global planner of the Navigation Stack
    ros::Subscriber position_sub = n.subscribe("move_base/feedback", 1000, position_cb);

    // Service used by the trader to know the cost of going to such position
    ros::ServiceServer service = n.advertiseService("metricsNode", metricsCalc_cb);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        // End of loop, wait, check the queues
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
