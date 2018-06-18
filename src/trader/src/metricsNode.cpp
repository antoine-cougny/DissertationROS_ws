#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "trader/Task.h"
#include "trader/metrics.h"
/* #include <move_base_msgs/MoveBaseAction.h> */
#include "move_base_msgs/MoveBaseFeedback.h"


//Action Feedback and Result
/* typedef boost::shared_ptr< ::move_base_msgs::MoveBaseActionFeedback const > MoveBaseActionFeedbackConstPtr; */

geometry_msgs::Pose myPose;

void position_cb(const move_base_msgs::MoveBaseFeedback &msg)
{
    /* extern geometry_msgs::Pose myPos; */
    myPose = msg.base_position.pose;
}

/*
 * Return the cartesian distance between the current position and the given
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
    return sqrt(result);
}

bool metricsCalc_cb(trader::metrics::Request &Task, trader::metrics::Response &metrics)
{
    metrics.cost = calcMovingCost(myPose, Task.task.goalPosition_p);
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "metricsNode");
    ros::NodeHandle n;

    /* geometry_msgs::Pose myPos; */
    // Topic used by the global planner of the Navigation Stack
    ros::Subscriber position_sub = n.subscribe("move_base_simple/feedback", 1000, position_cb);

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
