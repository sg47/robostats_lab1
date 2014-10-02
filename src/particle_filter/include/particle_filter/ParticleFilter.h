/*!
 * \file particle_filter.h
 */

#ifndef _PARTICLE_FILTER_H_
#define _PARTICLE_FILTER_H_

#include <list>
#include <vector>
#include <armadillo>

#include <ros/ros.h>


class ParticleFilter
{
public:
  ParticleFilter();

  bool initialize(const ros::NodeHandle& n);
  
private:

  bool loadParameters(const ros::NodeHandle& n);
  bool registerCallbacks(const ros::NodeHandle& n);


};


#endif
