#include <string>
#include <iostream>
#include <vector>
// Ros
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_srvs/Trigger.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
// Custom messages
#include "trader/Task.h"
#include "blockchain_handler/transactionBC.h"
// Custom service
#include "trader/taskToBeTraded.h"

using namespace std;

string ns;
int idRobot = 1000;
string robotPK;

bool transactionAvailable = false,
     markTransactionDoneAvailable = false;
blockchain_handler::transactionBC transaction_msg;
blockchain_handler::transactionBC taskDone;  

bool sendTransaction_bc(blockchain_handler::transactionBC::Request  &req,
                        blockchain_handler::transactionBC::Response &res)
{
    res.status = false;
    if (!transactionAvailable)
    {
        res.status = true;
        transactionAvailable = true;
        transaction_msg.request = req;
    }
    return true;
}

bool markTransactionDone_bc(trader::taskToBeTraded::Request  &req,
                            trader::taskToBeTraded::Response &res)
{
    res.auctionReady = false;
    if (!markTransactionDoneAvailable)
    {
        res.auctionReady = true;
        markTransactionDoneAvailable = true;
        taskDone.request.idTask = req.task.id;
    }
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blockchainHandler");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // For the n NodeHandle, we get param from the launch file
    n.param<int>("idRobot", idRobot, 0);

    ns = ros::this_node::getNamespace();
    ns = ns.substr(1, ns.size() - 1); // Delete an extra '/' at the beginning

    // Get transaction from traderNode
    ros::ServiceServer transactionBC_srvS;
    transactionBC_srvS = n.advertiseService("transactionBC", sendTransaction_bc);
    // Get transaction from taskExec, mark task as done
    ros::ServiceServer markTaskDone_srvS;
    markTaskDone_srvS = n.advertiseService("markTaskDone", markTransactionDone_bc);

    // Send task to JS node / eth client
    ros::ServiceClient deployTransaction_srvC;
    deployTransaction_srvC = n.serviceClient
            <blockchain_handler::transactionBC>("/deployTransactionBC");
    // Send task to be marked as done to JS node / eth client
    ros::ServiceClient markTaskDone_srvC;
    markTaskDone_srvC = n.serviceClient
            <blockchain_handler::transactionBC>("/markTaskDoneBC");

    n.param<int>("idRobot", idRobot, 0);
    n.param<string>("robotPK", robotPK, "xxx");

    /* In case we have a problem somewhere. I don't want amything stuck
     * I would rather loose a transaction on the blockchain than having the
     * whole system crashing.
     * However, as I improved the DApp (faster and more stable) now, 10 failed
     * attempts can be seen as a timeout.
     * Improving this can be done by storing a vector of the different 
     * incoming transactions
     */
    int counter = 10;

    while(ros::ok())
    {
        while (transactionAvailable && counter)
        {
            counter--;
            ROS_DEBUG("A transaction is available to be sent to the BC");
            if (deployTransaction_srvC.call(transaction_msg))
            {
                ROS_DEBUG("Contacted JS node for task trading");
                if (transaction_msg.response.status)
                {
                    ROS_INFO("Transaction (transfer) will be processed by the node");
                    transactionAvailable = false;
                }
                else
                {
                    ROS_INFO("The node is busy, we will wait for 5 sec");
                    ros::Duration(5).sleep();
                }
            }
            else
                ROS_ERROR("Could not connect to the JS interface to transfer the task");
                ros::Duration(1).sleep();
        }

        while (markTransactionDoneAvailable && counter)
        {
            counter--;
            ROS_DEBUG("Task %s will be marked as done on the BC",
                      taskDone.request.idTask.c_str());
            taskDone.request.idSeller = robotPK;
            if (markTaskDone_srvC.call(taskDone))
            {
                ROS_DEBUG("Contacted JS node to mark task as done");
                if (taskDone.response.status)
                {
                    ROS_INFO("Transaction (markTaskDone %s) will be processed by the node",
                             taskDone.request.idTask.c_str());
                    markTransactionDoneAvailable = false;
                }
                else
                {
                    ROS_INFO("The node is busy, we will wait for 5 sec");
                    ros::Duration(5).sleep();
                }
            }
            else
                ROS_ERROR("Could not connect to the JS interface to set the task as done");
                ros::Duration(1).sleep();
        }
            
        ros::spinOnce();
        loop_rate.sleep();
        counter = 10;
    }

    return 0;
}
