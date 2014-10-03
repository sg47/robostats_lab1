
#include <iostream>
#include <fstream>
#include <ctime>

#include <particle_filter/ParticleFilter.h>

using namespace std;
using namespace arma;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "particle_filter");
  ros::NodeHandle n("~");

  ParticleFilter pf;
  pf.initialize(n);

  ros::spin();
  
  return 0;
}
