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
     laser_data(const arma::vec3& rp, const arma::vec3& lp,
                const arma::vec::fixed<180> r) : robot_pose(rp),
                laser_pose(lp), ranges(r) {}
  } laser_data_t;

  std::map <double, boost::variant<laser_data_t, arma::vec3> > stampedData;

};


#endif
