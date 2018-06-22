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
string ns;

/*******************************************************************************
 *
 * Basically, what this node does is store an array of task to perform.
 * It will pick one from the list, check its metrics and based on a treshold
 * value, send it to the task exec or to the trader to get rid of it.
 *
 * If the task list is empty, the node will advertise the robot as idle
 *
 ******************************************************************************/

void newTask_cb(const trader::Task &msg)
{
    // Do some stuff like adding it to an array or a vector
    // Pay attention to pointers and content of the message
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traderRobot");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // For the n NodeHandle, we get param from the launch file
    n.param<int>("idRobot", idRobot, 0);

    // Advertise it there are tasks saved
    ros::Publisher hasTasks_pub;
    hasTasks_pub = n.advertise<std_msgs::Bool>("hasTasks", 10);
    // Sending goal to the task exec
    ros::Publisher sendGoal_pub;
    sendGoal_pub = n.advertise<trader::Task>("sendGoal", 10);
    // Send a goal to be traded
    ros::Publisher taskToTrade_pub;
    taskToTrade_pub = n.advertise<trader::Task>("taskToTrade", 2);

    // Getting a new task from traderNode (to be stored)
    ros::Subscriber newTask_sub = n.subscribe("newTask", 2, newTask_cb);

    // Metrics Service client, to get the metrics value
    ros::ServiceClient metrics_srvC;
    metrics_srvC = n.serviceClient<trader::metrics>("metricsNode");

    // Task object
    trader::Task task_msg;
    int tradingThreshold; // Used to know if we do the task or we triger an auction
    n.param<int>("tradingThreshold", tradingThreshold, 2);

    std_msgs::Bool hasTasks_status;

    while (ros::ok())
    {
        // Select a task in the list/vector
        // TODO implement that :p

        // Check the metrics of the selected task
        trader::metrics metric_srv;
        metric_srv.request.task = task_msg; 
        if (metrics_srvC.call(metric_srv))
        {
            ROS_INFO("Cost of the task: %.3f", metric_srv.response.cost);
            if (metric_srv.response.cost > tradingThreshold)
            {
                // Send task to trader
                ROS_INFO("Task too expensive, will be sent to traderNode");
                taskToTrade_pub.publish(task_msg);
            }
            else
            {
                // Do the task
                ROS_INFO("We will do the task");
                sendGoal_pub.publish(task_msg); 
            }
        }
        else
        {
            ROS_ERROR("Failed to call service metricsNode");
        }

        // Send status of task
        int nb = 1; // TODO
        hasTasks_status.data = (nb == 0) ? false : true;
        hasTasks_pub.publish(hasTasks_status);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
