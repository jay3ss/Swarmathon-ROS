#ifndef OBSTACLEHANDLER_H
#define OBSTACLEHANDLER_H
#include "MobilityHelper"
#include <std_msgs/UInt8.h>
	class ObstacleHandler: public MobilityHelper{
	public:
		ObstacltHander();
		void obstacleHandler(const std_msgs::UInt8::ConstPtr& message);
	private:
		bool isRetracting_;
	};
#endif