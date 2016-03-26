#include "MobilityHelper.h"

//ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>

//ROS messages
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <shared_messages/TagsImage.h>


// To handle shutdown signals so the node quits properly in response to "rosnode kill"
#include <ros/ros.h>
#include <signal.h>

#include "Rover.h"
#include "Publishers.h"
#include "Subscribers.h"
#include "Timers.h"
#include "Uniform.h"

#include <iostream>

#include <math.h>
// MobilityHelper::MobilityHelper(){
// 	subs = new Subscribers;
// 	pubs = new Publishers;
// 	state = TRANSFORM;
// }

MobilityHelper::MobilityHelper(Rover* rover)
{
    uniform_ = new Uniform;


    rover_ = rover;
    pubs = new Publishers;
    subs = new Subscribers;
    timers = new Timers;
    state = TRANSFORM;
    rover->setVelPublisher(pubs);
    objectDetected_ = false;
}

void MobilityHelper::search(const ros::TimerEvent&)
{
    random_numbers::RandomNumberGenerator rng;
    std_msgs::String stateMachineMsg;
    geometry_msgs::Pose2D currentLocation = rover_->getPosition();
    geometry_msgs::Pose2D goalLocation = rover_->getGoalPosition();
    if(current_mode_ == 2 || current_mode_ == 3)
    {
        switch(state)
        {
            case TRANSFORM:
            {
                stateMachineMsg.data = "TRANSFORMING";

                //If angle between current and goal is significant
                if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.1) {
                    state = ROTATE; //rotate
                }

                //If goal has not yet been reached
                else if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
                    state = TRANSLATE; //translate
                }

                //If returning with a target
                else if (rover_->hasTag() 	) {
                //If goal has not yet been reached
                    if (hypot(0.0 - currentLocation.x, 0.0 - currentLocation.y) > 0.5) {
                        //set angle to center as goal heading
                        goalLocation.theta = M_PI + atan2(currentLocation.y, currentLocation.x);
                        //set center as goal position
                        goalLocation.x = 0.0;
                        goalLocation.y = 0.0;
                    }
                    //Otherwise, reset target and select new random uniform heading
                    else {
                        rover_->dropTag();
                        goalLocation.theta = rng.uniformReal(0, 2 * M_PI);
                    }
                }

                //Otherwise, assign a new goal
                else {
                    goalLocation = uniform_->getNextPositionThreeRovers();

                    //select new heading from Gaussian distribution around current heading

                    //goalLocation.theta = rng.gaussian(currentLocation.theta, 0.25);

                    //select new position 50 cm from current location
//					goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
//					goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
                }
                    //Purposefully fall through to next case without breaking
            }

            //Calculate angle between currentLocation.theta and goalLocation.theta
            //Rotate left or right depending on sign of angle
            //Stay in this state until angle is minimized
            case ROTATE: {
                stateMachineMsg.data = "ROTATING";
                if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) > 0.1) {
                    if(!objectDetected_)
                        rover_->translate(0.0, 0.2); //rotate left
                    else
                        rover_->translate(0.0, 0.4); //rotate left
                }
                else if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) < -0.1) {
                    if(!objectDetected_)
                        rover_->translate(0.0, -0.2); //rotate left
                    else
                        rover_->translate(0.0, -0.4); //rotate left
                }
                else {
                    rover_->translate(0.0, 0.0); //stop
                    state = TRANSLATE; //move to translate step
                }
                break;
            }

            case TRANSLATE: {
                stateMachineMsg.data = "TRANSLATING";
                if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
                    rover_->translate(0.3, 0.0);
                }
                else {
                    rover_->translate(0.0, 0.0); //stop
                    state = TRANSFORM; //move back to transform step
                }
                break;
            }

            default: {
                break;
            }
        }
    }

    else { // mode is NOT auto
        // publish current state for the operator to see
        stateMachineMsg.data = "WAITING";
    }
    rover_->setRoverPosition(currentLocation);
    rover_->setGoalPosition(goalLocation);
    // publish state machine string for user, only if it has changed, though
    if (strcmp(stateMachineMsg.data.c_str(), prev_state_machine) != 0) {
        pubs->stateMachinePublish.publish(stateMachineMsg);
        sprintf(prev_state_machine, "%s", stateMachineMsg.data.c_str());
    }
}


void MobilityHelper::joyCmdHandler(const geometry_msgs::Twist::ConstPtr& message)
{
    if(current_mode_ == 0 || current_mode_ == 1)
        rover_->translate(message->linear.x, message->angular.z);
}

void MobilityHelper::modeHandler(const std_msgs::UInt8::ConstPtr& message)
{
    current_mode_ = message->data;
    rover_->translate(0.0, 0.0);
}

void MobilityHelper::targetHandler(const shared_messages::TagsImage::ConstPtr& message)
{
    //if this is the goal target
    if (message->tags.data[0] == 256)
    {
        //if we were returning with a target
        if (rover_->hasTag())
        {
            //publish to scoring code
            pubs->targetDropOffPublish.publish(message->image);
            rover_->dropTag();

            geometry_msgs::Pose2D gLoc = uniform_-> getCurrentPosition();
            gLoc.theta = atan2(gLoc.y, gLoc.x);

            rover_->setGoalPosition(gLoc);
        }
    }


    if (!rover_->hasTag() && message->tags.data[0] != 256)
    {
        //check if target has not yet been collected
            std_msgs::Int16 targetDetected;
            targetDetected.data = message->tags.data[0];
            //set angle to center as goal heading
            geometry_msgs::Pose2D gLoc;
            gLoc.theta = M_PI + atan2(rover_->getPosition().x, rover_->getPosition().y);
            gLoc.x = 0.0;
            gLoc.y = 0.0;
            rover_->setGoalPosition(gLoc);
            rover_->pickUpTag(message);
            //publish detected target
            pubs->targetsCollectedPublish.publish(targetDetected);
            //publish to scoring code
            pubs->targetPickUpPublish.publish(message->image);
            //switch to transform state to trigger return to center
            state = TRANSFORM;

    }
}

void MobilityHelper::obstacleHandler(const std_msgs::UInt8::ConstPtr& message)
{
    if (message->data > 0) {
        objectDetected_ = true;
        geometry_msgs::Pose2D currentLocation = rover_->getPosition();
        geometry_msgs::Pose2D goalLocation = rover_->getPosition();
        //obstacle on right side
        if (message->data == 1) {
            //select new heading 0.2 radians to the left
            goalLocation.theta = currentLocation.theta + 0.2;
        }
            //obstacle in front or on left side
        else if (message->data == 2) {
            //select new heading 0.2 radians to the right
            goalLocation.theta = currentLocation.theta - 0.2;
        }
        //select new position 50 cm from current location
        goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
        goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));

        rover_->setGoalPosition(goalLocation);
        rover_->setRoverPosition(currentLocation);

        //switch to transform state to trigger collision avoidance
        state = TRANSFORM;
    }
    else
    {
        objectDetected_ = false;
    }
}

void MobilityHelper::odometryHandler(const nav_msgs::Odometry::ConstPtr& message) {
    geometry_msgs::Pose2D currentLocation;
    //Get (x,y) location directly from pose
    currentLocation.x = message->pose.pose.position.x;
    currentLocation.y = message->pose.pose.position.y;
    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y, message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currentLocation.theta = yaw;
    rover_->setRoverPosition(currentLocation);
}

void MobilityHelper::publishStatusTimerEventHandler(const ros::TimerEvent&)
{
    std_msgs::String msg;
    msg.data = "online";
    pubs->status_publisher.publish(msg);
}

// Safety precaution. No movement commands - might have lost contact with ROS. Stop the rover.
// Also might no longer be receiving manual movement commands so stop the rover.
void MobilityHelper::killSwitchTimerEventHandler(const ros::TimerEvent& t)
{
    // No movement commands for killSwitchTime seconds so stop the rover
    rover_->translate(0.0, 0.0);
    double current_time = ros::Time::now().toSec();
    ROS_INFO("In mobility.cpp:: killSwitchTimerEventHander(): Movement input timeout. Stopping the rover at %6.4f.", current_time);
}

void MobilityHelper::targetsCollectedHandler(const std_msgs::Int16::ConstPtr& message) {
    // targetsCollected[message->data] = 1;
}
