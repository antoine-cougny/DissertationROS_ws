#include <string>
#include <iostream>
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
#include "trader/bid.h"
// Custom service
#include "trader/metrics.h"
#include "trader/auctionWinner.h"
#include "trader/taskToBeTraded.h"
#include "trader/bid_srv.h"
#include "blockchain_handler/transactionBC.h"
// Task class
#include "task.h"

using namespace std;

string ns;
int idRobot = 1000;
string robotPK;
bool isIdle = true,
     hasTasks = false;
// Global var used by callback fct on auction
bool is_task_available_for_trading = false,
     is_robot_available_for_trading = true,
     external_auction_available = false;
trader::Task receivedTaskToTrade;
trader::announcement receivedTaskToSave;
trader::announcement announceTask;

// When we get a bid
vector<int> vecBid;
vector<int> vecIdRobotSrv;

// When we won an auction
ros::Publisher newTask_pub;

int indexofSmallestElement(vector<int>& vec);
void listeningSleep(ros::Rate loop_rate, int sleepDuration);

/*
 * Callback function used by the announcement topic
 * We accept an auction is we have not triggered it.
 */
void announcement_cb(const trader::announcement &msg)
{
    if (msg.idTask.compare(announceTask.idTask) != 0)
    {
        ROS_INFO("Accepted incomming task for trading");
        receivedTaskToSave = msg;
        external_auction_available = true;
    }
    else
    {
        // Ignore message because we triggered this auction
        // !!! Not safe if 2 auctions at the same time.
        // !!! TODO: verif on ID but will be implemented later by using the PK
        //           of the token on the blockchain to id it.
        ROS_INFO("Task rejected because we launched it");
    }
}

/*
 * Callback function used by the servive linking the decisionNode and the
 * traderNode to open an auction on the given task. 
 */
bool taskToTrade_cb_srv(trader::taskToBeTraded::Request &req,
                        trader::taskToBeTraded::Response &res)
{
    receivedTaskToTrade = req.task;
    // If the robot is already in an auction we send this back to the node
    res.auctionReady = is_robot_available_for_trading;
    is_task_available_for_trading = is_robot_available_for_trading;

    return true;
}

void isIdle_cb(const std_msgs::Bool &msg)
{
    isIdle = msg.data;
}

void hasTasks_cb(const std_msgs::Bool &msg)
{
    hasTasks = msg.data;
}

/* void bidTrade_cb(const trader::bid &msg) */
/* { */
/*     if (msg.idTask.compare(announceTask.idTask) == 0) */
/*     { */
/*         // The received bid is about the task we announced */
/*         vecBid.push_back(msg.bid); */
/*         vecIdRobotSrv.push_back(msg.idRobot); */
/*     } */
/* } */

/*
 * Callback function used when a trader wants to bid on a task proposed by the
 * trader leader. If the bid is accepted, ie, the bid corresponds to a task that
 * has been open on this service, the response is a true boolean.
 */
bool bidTrade_cb_srv(trader::bid_srv::Request  &req,
                     trader::bid_srv::Response &res)
{
    res.acceptedBid = false;

    if (req.idTask.compare(announceTask.idTask) == 0)
    {
        // The received bid is about the task we announced
        vecBid.push_back(req.bid);
        vecIdRobotSrv.push_back(req.idRobot);
        res.acceptedBid = true;
    }

    return true;
}

/*
 * This callback function is used to let auctionners if they won or not the
 * auction they participated in.
 */
bool biddingStatus_cb(trader::auctionWinner::Request  &req,
                      trader::auctionWinner::Response &res)
{
    // Used to be told if we won or not
    if (req.isWinner)
    {
        // We send the task to the decisionNode and send back the pk
        ROS_INFO("Robot %d won the auction", idRobot);
        res.pk = robotPK;
        newTask_pub.publish(receivedTaskToSave.task);
    }
    else
    {
        ROS_INFO("Robot %d lost the auction", idRobot);
        res.pk = "";
    }


    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traderRobot");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // For the n NodeHandle, we get param from the launch file
    n.param<int>("idRobot", idRobot, 0);

    ns = ros::this_node::getNamespace();
    ns = ns.substr(1, ns.size() - 1); // Delete an extra / at the beginning

    // Announcement Messages
    ros::Publisher  announcement_pub;
    ros::Subscriber announcement_sub;
    announcement_pub = n.advertise<trader::announcement>("/announcementGlobal", 1);
    announcement_sub = n.subscribe("/announcementGlobal", 1, announcement_cb);

    // Getting robot state from decisionNode and taskExec
    ros::Subscriber isIdle_sub = n.subscribe("isIdle", 10, isIdle_cb);
    ros::Subscriber hasTasks_sub = n.subscribe("hasTasks", 10, hasTasks_cb);

    // Getting task from the decision node
    /* ros::Subscriber taskToTrade_sub = n.subscribe("taskToTrade", 2, taskToTrade_cb); */
    ros::ServiceServer taskToTrade_srv = n.advertiseService("taskToTrade", taskToTrade_cb_srv);

    // Sending a new task to decisionNode
    // Use of global variable in order to use the publisher in a callback fct
    newTask_pub = n.advertise<trader::Task>("newTask", 2);

    // Metrics Service client, to get the metrics value
    ros::ServiceClient metrics_srvC;
    metrics_srvC = n.serviceClient<trader::metrics>("metricsNode");

    // Variables declaration
    int sleepTrading = 20; // sec.
    int acceptanceThreshold; // Used to know if we do the task or we trigger an auction
    n.param<int>("idRobot", idRobot, 0);
    n.param<int>("acceptanceThreshold", acceptanceThreshold, 2);
    n.param<string>("robotPK", robotPK, "xxx"); // TODO
    
    while(ros::ok())
    {
        // TODO: divide code in functions :p
        if (is_task_available_for_trading)
        {
            ROS_INFO("Start of auction");
            // We trigger an auction 
            is_robot_available_for_trading = false;

            // Announcement message
            announceTask.idTask = receivedTaskToTrade.id + "_" + to_string(ros::Time::now().sec); // TODO: use pk of blockchain. TODO: task id is task pk instead?
            announceTask.idRobot = idRobot;
            announceTask.task = receivedTaskToTrade;
            announceTask.topic = ns + "_" + announceTask.idTask;
            ROS_WARN("Name of service to bid: %s ", announceTask.topic.c_str());

            // Create topic on the fly and Publish annoucement message
            // Switch to service
            ROS_INFO("Start custom service server to collect bid");
            ros::ServiceServer customSrv_bid;
            customSrv_bid = n.advertiseService(announceTask.topic, bidTrade_cb_srv);

            // Publish the annoucement message
            ROS_INFO("Publish announcement message");
            announcement_pub.publish(announceTask);

            // We wait 10 sec
            // TODO: find a better implementation
            ROS_INFO("Wait %d seconds", sleepTrading);
            /* ros::Duration(sleepTrading).sleep(); */
            ros::Time start_waiting = ros::Time::now();
            while (ros::Time::now() - start_waiting < ros::Duration(sleepTrading))
            {
                ros::spinOnce();
                loop_rate.sleep();
            }

            // We make sure we got at least one bid
            ROS_INFO("We got %d bid(s)", vecIdRobotSrv.size());
            if (vecIdRobotSrv.size())
            {
                // We will select the winner and a service message to him
                // The others will get a no
                // TODO: what is the smallest cost is bigger than our cost?
                int indexWinner = indexofSmallestElement(vecBid);
                
                // We tell the winner his victory
                ROS_INFO("Tell the winner the auction ended");
                ROS_INFO("The winner is robot id %d", vecIdRobotSrv[indexWinner]);
                ROS_WARN("Contacting the winner with the service %s", 
                    ("/" + announceTask.idTask + "_" + to_string(vecIdRobotSrv[indexWinner])).c_str());
                trader::auctionWinner winnerSrv;
                winnerSrv.request.isWinner = true;
                ros::ServiceClient winnerSrvClient;
                winnerSrvClient = n.serviceClient<trader::auctionWinner>
                    ("/" + announceTask.idTask + "_" + to_string(vecIdRobotSrv[indexWinner]));
                if (winnerSrvClient.call(winnerSrv))
                {
                    ROS_INFO("Winner is aware");

                    // We send a signal to the blockchainNode
                    ROS_INFO("Sending a msg to blockchainNode");
                    ros::ServiceClient sendTransaction_srvC;
                    sendTransaction_srvC = n.serviceClient
                        <blockchain_handler::transactionBC>("transactionBC");
                    // Service message
                    blockchain_handler::transactionBC bc_srv;
                    bc_srv.request.idTask = announceTask.task.id;
                    bc_srv.request.idSeller = robotPK;
                    bc_srv.request.idBuyer = winnerSrv.response.pk;
                    // Send Messsage
                    if(sendTransaction_srvC.call(bc_srv))
                        ROS_INFO("Transaction initiated");
                    else
                        ROS_ERROR("Could not contact blockchainNode");
                }
                else
                    ROS_ERROR("Failed to contact winner");

                // We tell the others that they lost
                ROS_INFO("Tell the other robots they lost");
                for (int i = 0; i < vecIdRobotSrv.size() ; i++)
                {
                    if (i != indexWinner)
                    {
                        trader::auctionWinner looserSrv;
                        looserSrv.request.isWinner = false;
                        ros::ServiceClient looserSrvClient = n.serviceClient
                            <trader::auctionWinner>
                            ("/" + announceTask.idTask + "_" + to_string(vecIdRobotSrv[i]));
                        ROS_INFO("Contacting looser on %s",
                            ("/" + announceTask.idTask + "_" + to_string(vecIdRobotSrv[i])).c_str());
                        if (looserSrvClient.call(looserSrv))
                            ROS_INFO("Contacted looser %d", vecIdRobotSrv[i]);
                        else
                            ROS_ERROR("Failed to contact looser %d", vecIdRobotSrv[i]);
                    }
                }
            }
            else
            {
                // Send the task back to the decision node
                // TODO: make test on id/ns: for computer, we want to continue sending it
                newTask_pub.publish(receivedTaskToSave.task);
                ROS_INFO("Sending the task back to the decision node");
                /* receivedTaskToSave = 0; */
            }
 
            // Auction has ended, clear variables
            ROS_INFO("Cleaning variables");
            is_robot_available_for_trading = true;
            is_task_available_for_trading = false;
            vecIdRobotSrv.clear();
            vecBid.clear();
        }

        // We participate in an external auction
        else if (external_auction_available)
        {
            is_robot_available_for_trading = false;
            // Check the metrics of the selected task
            ROS_INFO("We received an auction offer");
            trader::metrics metric_srv;
            metric_srv.request.task = receivedTaskToSave.task; 
            if (metrics_srvC.call(metric_srv))
            {
                ROS_INFO("Cost of the task: %.3f", metric_srv.response.cost);
                if (metric_srv.response.cost < acceptanceThreshold)
                {
                    ROS_INFO("Robot %d is bidding", idRobot);
                    // SERVICE VERSION
                    // We decide to bid on the task
                    
                    ROS_INFO("Creation of the service server to get answer from the trader leader");
                    ROS_WARN("Name of the service we bid on: %s ",receivedTaskToSave.topic.c_str());
                    ros::ServiceClient customSrv_bid_srvC;
                    customSrv_bid_srvC = n.serviceClient<trader::bid_srv>
                            (receivedTaskToSave.topic);

                    // Create bid message - service
                    ROS_INFO("Create bid_msg");
                    trader::bid_srv bid_msg;
                    bid_msg.request.idTask = receivedTaskToSave.idTask;
                    bid_msg.request.idRobot = idRobot;
                    bid_msg.request.bid = metric_srv.response.cost;

                    // Create service server before sending bid the handle
                    // the result of the auction
                    ROS_WARN("Name of the service to announce the result: %s",
                        ("/" + receivedTaskToSave.idTask + "_" + to_string(idRobot)).c_str());
                    ros::ServiceServer biddingSrvServer = n.advertiseService
                        ("/" + receivedTaskToSave.idTask + "_" + to_string(idRobot), biddingStatus_cb);

                    // Sending
                    if (customSrv_bid_srvC.call(bid_msg))
                        ROS_INFO("Called bidding service with the bid %.3f", (double) bid_msg.request.bid);
                    else
                        ROS_ERROR("Robot %d Failed to submit bid", idRobot);
                    

                    // Wait for answer - THIS IS DONE WITH THE CALLBACK FCT
                    // If we won, send the task to the decisionNode
                    ros::Time start_waiting = ros::Time::now();
                    while (ros::Time::now() - start_waiting < ros::Duration(sleepTrading + 5))
                    {
                        ros::spinOnce();
                        loop_rate.sleep();
                    }
                }
                else
                    ROS_INFO("Robot %d We do not bid on this task", idRobot);
            }
            else
                ROS_ERROR("Robot %d Failed to call service metricsNode", idRobot);

            // Clear Variables
            is_robot_available_for_trading = true;
            external_auction_available = false;
            ROS_INFO("Auction DONE for trader");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


int indexofSmallestElement(vector<int>& vec)
{
    int index = 0;

    for(int i = 1; i < vec.size(); i++)
    {
        if(vec[i] < vec[index])
            index = i;              
    }

    return index;
}

void listeningSleep(ros::Rate loop_rate, int sleepDuration)
{
    ros::Time start_waiting = ros::Time::now();
    while (ros::Time::now() - start_waiting < ros::Duration(sleepDuration))
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
