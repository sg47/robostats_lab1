#include <iostream>
#include <ctime>
#include <math.h>
#include <unistd.h>

#include <particle_filter/ParticleFilter.h>


using namespace arma;
using namespace std;


ParticleFilter::ParticleFilter()
{
}

bool ParticleFilter::initialize(const ros::NodeHandle& n)
{
  std::string name = ros::names::append(n.getNamespace(), "ParticleFilter");

  if (!loadParameters(n))
  {
    ROS_ERROR("%s: failed to load parameters", name.c_str());
    return false;
  }

  if (!registerCallbacks(n))
  {
    ROS_ERROR("%s: failed to register callbacks", name.c_str());
    return false;
  }

  return true;
}


bool ParticleFilter::loadParameters(const ros::NodeHandle& n)
{
  return true;
}

bool ParticleFilter::registerCallbacks(const ros::NodeHandle& n)
{
  return true;
}
