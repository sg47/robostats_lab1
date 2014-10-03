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

class ParticleFilter
{
public:
  ParticleFilter();

  bool initialize(const ros::NodeHandle& n);

private:
  bool loadParameters(const ros::NodeHandle& n);
  bool registerCallbacks(const ros::NodeHandle& n);

  typedef struct laser_data
  {
     arma::vec3 robot_pose;
     arma::vec3 laser_pose;
     arma::vec::fixed<180> ranges;
  } laser_data_t;

  arma::mat::fixed<800,800> occ_grid_matrix;
  nav_msgs::OccupancyGrid occ_grid_msg;
  std::map <double, boost::variant<laser_data_t, arma::vec3> > stampedData;

  ros::Publisher map_pub;
  ros::Timer drawmap_timer;
  
  bool loadData();
  bool loadMap();
  void publishMap(const ros::TimerEvent& te);

};


#endif
