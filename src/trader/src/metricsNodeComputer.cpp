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

bool metricsCalc_cb(trader::metrics::Request &Task, trader::metrics::Response &metrics)
{
    metrics.cost = 100000;
    ROS_DEBUG("Cost PC: %d", metrics.cost);
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "metricsNode");
    ros::NodeHandle n;

    // Service used by the trader to know the cost of going to such position
    ros::ServiceServer service = n.advertiseService("metricsNode", metricsCalc_cb);

    ros::Rate loop_rate(10);
    ros::spin();

    return 0;
}
