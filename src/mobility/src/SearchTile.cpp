#include "SearchTile.h"

// Constructor. This takes no parameters
SearchTile::SearchTile()
{
  index_ = 0;
}

//Deconstructor
SearchTile::~SearchTile(){} //<* This takes no parameters

void SearchTile::setPostition(geometry_msgs::Pose2D pos_)
{
  position_ = pos_;//<* Sets current tile position to search

}
void SearchTile::setFuturePostition(geometry_msgs::Pose2D future_pose_ )
{
  future_position_ = future_pose_;//<* Sets NEXT tile position to search
}

void SearchTile::setTopRight(geometry_msgs::Pose2D top_right)
{
  top_right = top_right; //<* Set Top right coordinates of the the tile to search
}

void SearchTile::setTopLeft(geometry_msgs::Pose2D top_left)
{
  top_left_ = top_left; //<* Set Top left coordinates of the the tile to search
}

void SearchTile::setBottomRight(geometry_msgs::Pose2D bot_right)
{
  bottom_right_ = bot_right; //<* Set Bottom right coordinates of the the tile to search
}

void SearchTile::setBottomLeft(geometry_msgs::Pose2D bot_left)
{
  bottom_left_ = bot_left;//<* Set bottom left coordinates of the the tile to search
}

// Sets the corners relative to its current position and the future postion
void SearchTile::setNewCorners()
{
      // If out future tile is north of current position
  if(future_position_.x - position_.x <= 0 && future_position_.y - position_.y  > 0)
  {
    top_right_.x = future_position_.x + 0.5;
    top_right_.y = future_position_.y + 0.5;

    top_left_.x = future_position_.x - 0.5;
    top_left_.y = future_position_.y + 0.5;

    bottom_right_.x = future_position_.x - 0.5;
    bottom_right_.y = future_position_.y - 0.5;


    bottom_left_.x = future_position_.x + 0.5 ;
    bottom_left_.y = future_position_.y - 0.5;

  }
  // If Future tile is west of current position
  else if(future_position_.x - position_.x < 0 && future_position_.y - position_.y  <= 0)
  {
      // top right = top left
    top_right_.x = future_position_.x - 0.5;
    top_right_.y = future_position_.y + 0.5;

    // top left would be bot left
    top_left_.x = future_position_.x + 0.5 ;
    top_left_.y = future_position_.y - 0.5;

// bot right would be top rught
    bottom_right_.x = future_position_.x + 0.5;
    bottom_right_.y = future_position_.y + 0.5;

//bot left would be bot right
    bottom_left_.x = future_position_.x - 0.5;
    bottom_left_.y = future_position_.y - 0.5;

  }
  // If future tile is south of current position
  else if(future_position_.x - position_.x <= 0 && future_position_.y - position_.y  < 0)
  {
    // bot left
    top_right_.x = future_position_.x + 0.5;
    top_right_.y = future_position_.y - 0.5;

    // bot right
    top_left_.x = future_position_.x - 0.5;
    top_left_.y = future_position_.y - 0.5;

    // top left
    bottom_right_.x = future_position_.x - 0.5;
    bottom_right_.y = future_position_.y + 0.5;

    // top right
    bottom_left_.x = future_position_.x + 0.5;
    bottom_left_.y = future_position_.y + 0.5;

  }
  // If future tile is east of current position
  else
  {
    // Bot right
    top_right_.x = future_position_.x - 0.5;
    top_right_.y = future_position_.y - 0.5;
    // top right
    top_left_.x = future_position_.x + 0.5;
    top_left_.y = future_position_.y + 0.5;

    //bot left
    bottom_right_.x = future_position_.x + 0.5;
    bottom_right_.y = future_position_.y - 0.5;

// top left
    bottom_left_.x = future_position_.x - 0.5;
    bottom_left_.y = future_position_.y + 0.5;

  }

}

/* This will move the rover below the bottom right tile
void SearchTile::initializePos()
{
  geometry_msgs::Pose2D initial_position_;
  double a = 1/3;

  initial_position_.x = bottom_right_.x - (a/2);
  initial_position_.y = bottom_right_.y - (a/2);

  rover_.setGoalPosition(initial_position_);

  rover_.faceGoal();
  // We will now move to our rover to its goal
  rover_.translate();

  while(!rover_.isAtGoal())
  {

  }

}
*/

// Newly added function to make it search a tile

  /******************************************************************
   * Created by Abner Hernandez                                     *
   * Purpose of this function is to search within a 1m x 1m meter   *
   * tile.                                                          *
   ******************************************************************
   */

// Finds the six points to give to search in a 1 X 1 Tile
void SearchTile::setSearchPoints()
{
  // Lawnmower search method
  // Explore within the four corners

  double a = 1/3; //<* the microgrid (1m x 1m tile) is divided into three adjacent thirds,
                  //<* further divided into three pieces - 9 total (appromixately the size of one rover)
                  //<* thus, in order to keep accuracy we must store 1/3 to maintain as many sig figs


/* The rover translates forward to the center of the smaller .33m x by .33 m piece within this tile,
 * to account for the viewing window of the rover's camera, due to not having visibility within
 * approximately .3m directly in front, therefore a is divided into 2 to make it the center
 */



  geometry_msgs::Pose2D next_Goal; // <*
  next_Goal.x = bottom_right_.x - (a/2);
  next_Goal.y = bottom_right_.y + (a/2);

  search_points_[0] = next_Goal;


  next_Goal.x = top_right_.x - (a/2); //<* We will set the next goal to be either the top
  next_Goal.y = top_right_.y - (a/2);

  search_points_[1] = next_Goal;


  // Our goal will be in the
  next_Goal.x = top_right_.x - (a + (a/2)) ;
  next_Goal.y = top_right_.y - (a/2);

  search_points_[2] = next_Goal;


  next_Goal.x = bottom_right_.x - (a + (a/2)) ;
  next_Goal.y = bottom_right_.y + (a/2) ;

  search_points_[3] = next_Goal;



  // We can move
  next_Goal.x = bottom_left_.x  + (a/2);
  next_Goal.y = bottom_left_.y + (a/2);

  search_points_[4] = next_Goal;

  // Finally move to the edge

  next_Goal.x = top_left_.x  + (a/2);
  next_Goal.y = top_left_.y - (a/2);

  search_points_[5] = next_Goal;


  index_ = 0;
}

geometry_msgs::Pose2D SearchTile::getPoint()
{
  return search_points_[index_];
}

void SearchTile::setNextPoint()
{
  index_++;
}
