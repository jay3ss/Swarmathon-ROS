#include "RandomSearch.h"

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>

#include <random_numbers/random_numbers.h>

#include "Publishers.h"
#include "Subscribers.h"
#include "Timers.h"
#include "Rover.h"
RandomSearch::RandomSearch(Rover* rover)
{
	rover_ = rover;
	pubs = new Publishers;
	subs = new Subscribers;
	timers = new Timers;
}

void RandomSearch::searchss(const ros::TimerEvent&)
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
				stateMachineMsg.data = "TRASNFORMING";
				//If angle between current and goal is significant
				if (fabs(angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta)) > 0.1) {
					state = ROTATE; //rotate
				}
				//If goal has not yet been reached
				else if (fabs(angles::shortest_angular_distance(currentLocation.theta, atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x))) < M_PI_2) {
					state = TRANSLATE; //translate
				}
				//If returning with a target
				else if (rover_->target_detected_.data != -1) {
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
						rover_->target_detected_.data = -1;
						goalLocation.theta = rng.uniformReal(0, 2 * M_PI);
					}
				}

				//Otherwise, assign a new goal
				else {
					//select new heading from Gaussian distribution around current heading
					goalLocation.theta = rng.gaussian(currentLocation.theta, 0.25);
					//select new position 50 cm from current location
					goalLocation.x = currentLocation.x + (0.5 * cos(goalLocation.theta));
					goalLocation.y = currentLocation.y + (0.5 * sin(goalLocation.theta));
				}	
					//Purposefully fall through to next case without breaking
			}
			
		
			//Calculate angle between currentLocation.theta and goalLocation.theta
			//Rotate left or right depending on sign of angle
			//Stay in this state until angle is minimized
			case ROTATE: {
				stateMachineMsg.data = "ROTATING";
				if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) > 0.1) {
					rover_->translate(0.0, 0.2); //rotate left
				}
				else if (angles::shortest_angular_distance(currentLocation.theta, goalLocation.theta) < -0.1) {
					rover_->translate(0.0, -0.2); //rotate right
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