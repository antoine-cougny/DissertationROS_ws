#include <string>
#include <vector>
// Ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
// Custom messages
#include "trader/Task.h"
#include "trader/announcement.h"
// Custom services
#include "trader/metrics.h"
#include "trader/taskToBeTraded.h"
// Task class
#include "task.h"

using namespace std;

bool isIdle = true;
int idRobot = 1000;
string ns;
vector<trader::Task> vecTaskToDo;

/*******************************************************************************
 *
 * Basically, what this node does is store an array of task to perform.
 * It will pick one from the list, check its metrics and based on a treshold
 * value, send it to the task exec or to the trader to get rid of it.
 *
 * If the task list is empty, the node will advertise the robot as idle
 *
 ******************************************************************************/

void isIdle_cb(const std_msgs::Bool &msg)
{
    isIdle = msg.data;
}

void newTask_cb(const trader::Task &msg)
{
    // Do some stuff like adding it to an array or a vector
    // Pay attention to pointers and content of the message
    vecTaskToDo.push_back(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decisionRobot");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // For the n NodeHandle, we get param from the launch file
    n.param<int>("idRobot", idRobot, 0);

    // Advertise it there are tasks saved
    std_msgs::Bool hasTasks_status;
    ros::Publisher hasTasks_pub;
    hasTasks_pub = n.advertise<std_msgs::Bool>("hasTasks", 10);
    // Getting robot state from taskExec
    ros::Subscriber isIdle_sub = n.subscribe("isIdle", 10, isIdle_cb);
    // Sending goal to the task exec
    ros::Publisher sendGoal_pub;
    sendGoal_pub = n.advertise<trader::Task>("goalSender", 10);
    // Send a goal to be traded to the traderNode
    ros::ServiceClient taskToTrade_srvC;
    taskToTrade_srvC = n.serviceClient<trader::taskToBeTraded>("taskToTrade");

    // Getting a new task from traderNode (to be stored)
    ros::Subscriber newTask_sub = n.subscribe("newTask", 2, newTask_cb);

    // Metrics Service client, to get the metrics value
    ros::ServiceClient metrics_srvC;
    metrics_srvC = n.serviceClient<trader::metrics>("metricsNode");

    // Task object
    trader::Task task_msg;
    int tradingThreshold; // Used to know if we do the task or we trigger an auction
    n.param<int>("idRobot", idRobot, 0);
    n.param<int>("tradingThreshold", tradingThreshold, 2);

    // Random pick a task
    default_random_engine generator;

    while (ros::ok())
    {
        // Before picking a task in the vector, there must be at least one
        // Although the taskexec can handle a set of task in its own queue,
        // we do not want to send a new one while it is busy.
        // Note that a task could be decomposed as a set of goal which would
        // be handled in this node.
        if (vecTaskToDo.size() && isIdle)
        {
            ROS_INFO("We have %d tasks stored", (int) vecTaskToDo.size());
            // Select a task in the vector (uniform distribution)
            // TODO: check if (random) does not come from here
            uniform_int_distribution<int> distribution(0,vecTaskToDo.size());
            int index_selectedTask = distribution(generator);
            task_msg = vecTaskToDo[index_selectedTask];

            // Check the metrics of the selected task
            trader::metrics metric_srv;
            metric_srv.request.task = task_msg; 
            if (metrics_srvC.call(metric_srv))
            {
                float benefit = task_msg.reward - metric_srv.response.cost;
                ROS_INFO("Potential benefit of the task: %.3f", benefit);
                // If the task is still interesting to do
                if (benefit > tradingThreshold)
                {
                    // Do the task
                    ROS_INFO("Robot %d We will do the task", idRobot);
                    sendGoal_pub.publish(task_msg);
                    // Delete the task from the vector list
                    vecTaskToDo.erase(vecTaskToDo.begin() + index_selectedTask);
                }
                else
                {
                    // Send task to trader
                    ROS_INFO("Task too expensive, will be sent to traderNode");

                    // We use a service to have a feedback on the status of traderNode
                    trader::taskToBeTraded taskTrade_srv;
                    taskTrade_srv.request.task = task_msg;
                    if (taskToTrade_srvC.call(taskTrade_srv))
                    {
                        if (taskTrade_srv.response.auctionReady)
                        {
                            ROS_INFO("traderNode accepted to launch an auction");
                            // Delete task from vector
                            vecTaskToDo.erase(vecTaskToDo.begin() + index_selectedTask);
                        }
                        else
                        {
                            // Trader is busy, leave task in vector, pick another one
                            // TODO: if trader is busy, shall we perform the task or try later?
                            ROS_INFO("traderNode is busy");
                        }
                    }
                    else
                        ROS_ERROR("Robot %d Failed service call for task to traderNode", idRobot);
                }
            }
            else
                ROS_ERROR("Robot %d Failed to call service metricsNode", idRobot);
        }

        // Send status of task
        hasTasks_status.data = (vecTaskToDo.size() == 0) ? false : true;
        hasTasks_pub.publish(hasTasks_status);

        // Check ROS queues and callback
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
