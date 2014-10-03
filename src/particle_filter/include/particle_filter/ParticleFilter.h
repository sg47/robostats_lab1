/*!
 * \file particle_filter.h
 */

#ifndef _PARTICLE_FILTER_H_
#define _PARTICLE_FILTER_H_

#include <list>
#include <vector>
#include <armadillo>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <boost/variant.hpp>

#define MAP_RESOLUTION  0.1   // grid cell width (m)
#define MAP_WIDTH       800   // (x_max - x_min)/resolution
#define MAP_HEIGHT      800   // (y_max - y_min)/resolution


class ParticleFilter
{
public:
  ParticleFilter();

  bool initialize(const ros::NodeHandle& n);

  int getOccValueAtXY(const double x, const double y);

  void getIndiciesFromXY(const double x, const double y,
                         unsigned int& row, unsigned int& col);
  
private:
  bool loadParameters(const ros::NodeHandle& n);
  bool registerCallbacks(const ros::NodeHandle& n);

  typedef struct laser_data
  {
     arma::vec3 robot_pose;
     arma::vec3 laser_pose;
     arma::vec::fixed<180> ranges;
  } laser_data_t;

  arma::mat::fixed<MAP_HEIGHT,MAP_WIDTH> occ_grid_matrix;
  nav_msgs::OccupancyGrid occ_grid_msg;
  std::map <double, boost::variant<laser_data_t, arma::vec3> > stampedData;

  ros::Publisher map_pub;
  ros::Timer drawmap_timer;
  
  bool loadData();
  bool loadMap();
  void publishMap(const ros::TimerEvent& te);

};


#endif
