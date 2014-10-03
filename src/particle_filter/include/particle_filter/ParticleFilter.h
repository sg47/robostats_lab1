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
#include <boost/random/mersenne_twister.hpp>
#include <particle_filter/ParameterUtils.h>


class ParticleFilter
{
public:
  ParticleFilter();

  bool initialize(const ros::NodeHandle& n);

private:
  typedef struct laser_data
  {
     arma::vec3 robot_pose;
     arma::vec3 laser_pose;
     arma::vec::fixed<180> ranges;
     laser_data(const arma::vec3& rp, const arma::vec3& lp,
                const arma::vec::fixed<180> r) : robot_pose(rp),
                laser_pose(lp), ranges(r) {}
  } laser_data_t;

  typedef struct particle
  {
    double weight;
    arma::vec3 pose;
    particle(double w, const arma::vec3& p) : weight(w), pose(p) {}
    // TODO: add print function
  } particle_t;

  bool loadParameters(const ros::NodeHandle& n);
  bool registerCallbacks(const ros::NodeHandle& n);
  void initializeParticles();
  void processDynamics(const arma::vec3& pose_in, const arma::vec3& pose_delta,
                       arma::vec3& pose_out);
  void getPoseDelta(const arma::vec3& before, const arma::vec3& after,
                    arma::vec3& delta);
  double unroll(double x);
  double normalize_angle(double x);
  double shortest_angular_distance(double from, double to);
  void processUpdate(const arma::vec3& u);

  std::map <double, boost::variant<laser_data_t, arma::vec3> > stampedData;
  std::vector <particle_t> particle_bag;

  unsigned int num_particles;

  // standard deviation parameters of the delta pose odometry "control input"
  double sigma_dx, sigma_dy, sigma_dyaw;

  boost::mt19937 rng;

  std::string name;
  std::string fixed_frame_id;
  std::string odom_frame_id;
  std::string laser_frame_id;

  ros::Publisher particle_pub;
};


#endif
