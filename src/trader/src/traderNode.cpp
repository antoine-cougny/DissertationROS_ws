#include <string>
// Ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
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
bool isIdle = true,
     hasTasks = false;
string ns;

void announcement_cb(const trader::announcement &msg)
{
    ROS_INFO("got msg");
}

void taskToTrade_cb(const trader::Task &msg)
{
    // Do stuff
}

void isIdle_cb(const std_msgs::Bool &msg)
{
    isIdle = msg.data;
}

void hasTasks_cb(const std_msgs::Bool &msg)
{
    hasTasks = msg.data;
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

    // Getting robot state from decisionNode and taskExec
    ros::Subscriber isIdle_sub = n.subscribe("isIdle", 1, isIdle_cb);
    ros::Subscriber hasTasks_sub = n.subscribe("hasTasks", 1, hasTasks_cb);

    // Getting task from the decision node
    ros::Subscriber taskToTrade_sub = n.subscribe("taskToTrade", 2, taskToTrade_cb);

    // Sending a new task to decisionNode
    ros::Publisher newTask_pub = n.advertise<trader::Task>("newTask", 1);

    // Variables declaration
    bool is_task_available_for_trading = false;
    
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
