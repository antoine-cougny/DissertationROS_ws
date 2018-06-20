#include <string>
// Ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
// Custom messages
#include "trader/Task.h"
#include "trader/metrics.h"
#include "trader/announcement.h"
// Task class
#include "task.h"

using namespace std;

int idRobot = 0;
string ns;

void announcement_cb(const trader::announcement &msg)
{
    ROS_INFO("got msg");
}

void decision_cb(const trader::task)
{
    // Do stuff
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "traderRobot");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // For the n NodeHandle, we get param from the launch file
    n.param<int>("idRobot", idRobot, 0);

    ns = ros::this_node::getNamespace();

    // Announcement Messages
    ros::Publisher  announcement_pub;
    ros::Subscriber announcement_sub;
    announcement_pub = n.advertise<trader::announcement>("/announcementGlobal", 1);
    announcement_sub = n.subscribe("/announcementGlobal", 1, announcement_cb);

    // Getting task from the decision node
    ros::Subscriber decision_sub;
    decision_sub = n.subscribe("decisionNode", 1, decision_cb);
    
    while(ros::ok())
    {

        if (is_task_available_for_trading)
        {
            // Do STUFF
        }

        loop_rate.sleep();
    }

    return 0;
}
