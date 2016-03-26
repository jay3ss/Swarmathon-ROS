/********************************************************************
   * Created by Abner Hernandez                                     *
   * Purpose of this function is to search within a 1m x 1m meter   *
   * tile.                                                          *
   ******************************************************************
   */

// Includes
#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <angles/angles.h>
#include <string>
//#include <TagObject.h>
//#include "Rover.h"

// Definition
#ifndef SEARCHTILE_H
#define SEARCHTILE_H


class SearchTile
{

private:

  geometry_msgs::Pose2D    position_; //<* Stores the center of the current tile to search
  geometry_msgs::Pose2D    future_position_; //<*Stores the center of the NEXT tile to search

  //Rover                    rover_;//<*Stores a rover object so that we may use it to move

  geometry_msgs::Pose2D    top_right_;// <* Top right coordinates of the the tile to search
  geometry_msgs::Pose2D    top_left_; // <* Top left coordinates of the the tile to search
  geometry_msgs::Pose2D    bottom_right_; // <* Bottom right coordinates of the the tile to search
  geometry_msgs::Pose2D    bottom_left_; // <* Bottom right coordinates of the the tile to search

  geometry_msgs::Pose2D    search_points_[6];

  int index_ ; // <* stores the index to search. This is used to get the search points


public:

  SearchTile();//<* Constructor
  ~SearchTile();//<* Deconstructor

  void setSearchPoints();//<* set search points in a tile

  void setPostition(geometry_msgs::Pose2D); //<* Set Position of the current tile
  void setFuturePostition(geometry_msgs::Pose2D);//<* Set Position of the current tile

  void setTopRight(geometry_msgs::Pose2D);//<* Set Top right coordinates of the the tile to search
  void setTopLeft(geometry_msgs::Pose2D);//<* Set Top left coordinates of the the tile to search
  void setBottomRight(geometry_msgs::Pose2D);//<* Set Bottom right coordinates of the the tile to search
  void setBottomLeft(geometry_msgs::Pose2D);//<* Set bottom left coordinates of the the tile to search

  void setNewCorners();// <* Initialize the rovers to move

  geometry_msgs::Pose2D getPoint(); // <* returns the current point to go to in the search point array

  void setNextPoint();//<* Increases the index so that we get the next point

  //void initializePos();// <* Move at the bot right corner

};

#endif