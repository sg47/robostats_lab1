#include <iostream>
#include <ctime>
#include <math.h>
#include <unistd.h>
#include <numeric>

#include <boost/random/uniform_real.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/math/distributions/normal.hpp>

#include <particle_filter/ParticleFilter.h>

namespace pu = parameter_utils;

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

  initializeParticles();

  // seed noise distribution and random number generator
  rng.seed(ros::WallTime::now().toSec());
  srand(ros::WallTime::now().toSec());

  return true;
}


bool ParticleFilter::loadParameters(const ros::NodeHandle& n)
{

  if (!pu::get("algorithm/num_particles", num_particles)) return false;
  if (!pu::get("algorithm/sigma/x", sigma_dx)) return false;
  if (!pu::get("algorithm/sigma/y", sigma_dy)) return false;
  if (!pu::get("algorithm/sigma/yaw", sigma_dyaw)) return false;

  return true;
}

bool ParticleFilter::registerCallbacks(const ros::NodeHandle& n)
{
  return true;
}

void ParticleFilter::initializeParticles()
{
// TODO: for initializing particles, randomly generate them and only accept them for initialization if they lie in a known (as opposed to an unknown) grid cell
}

void ParticleFilter::processDynamics(const arma::vec3& pose_in,
                                     const arma::vec3& pose_delta,
                                     arma::vec3& pose_out)
{
  double sy = sin(pose_in(2));
  double cy = cos(pose_in(2));

  pose_out(0) = pose_in(0) + cy*pose_delta(0) - sy*pose_delta(1);
  pose_out(1) = pose_in(1) + sy*pose_delta(0) + cy*pose_delta(1);
  pose_out(2) = pose_in(2) + pose_delta(2);
}

void ParticleFilter::getPoseDelta(const arma::vec3& before,
                                  const arma::vec3& after,
                                  arma::vec3& delta)
{
  double yaw_prev = before(2);
  double sy = sin(yaw_prev);
  double cy = cos(yaw_prev);
  double dxw = after(0) - before(0);
  double dyw = after(1) - before(1);
  delta(0) = cy*dxw + sy*dyw;
  delta(1) = -sy*dxw + cy*dyw;
  delta(2) = shortest_angular_distance(yaw_prev, after(2));
}

double ParticleFilter::unroll(double x)
{
  x = fmod(x, 2.0*M_PI);
  if (x < 0) x += 2.0*M_PI;
  return x;
}

double ParticleFilter::normalize_angle(double x)
{
  x = fmod(x + M_PI, 2.0*M_PI);
  if (x < 0) x += 2.0*M_PI;
  return x - M_PI;
}

double ParticleFilter::shortest_angular_distance(double from, double to)
{
  double result = unroll(unroll(to) - unroll(from));
  if (result > M_PI)
    result = -(2.0*M_PI - result);
  return normalize_angle(result);
}

void ParticleFilter::processUpdate(const arma::vec3& u)
{
  // Generate Gaussian about u with sigma_dx, sigma_dy, sigma_dyaw
  static boost::normal_distribution<double> gauss_dx(0.0, sigma_dx);
  static boost::normal_distribution<double> gauss_dy(0.0, sigma_dy);
  static boost::normal_distribution<double> gauss_dyaw(0.0, sigma_dyaw);

  static boost::variate_generator<boost::mt19937,
    boost::normal_distribution<double> > ndx(rng, gauss_dx);
  static boost::variate_generator<boost::mt19937,
    boost::normal_distribution<double> > ndy(rng, gauss_dy);
  static boost::variate_generator<boost::mt19937,
    boost::normal_distribution<double> > ndyaw(rng, gauss_dyaw);

  // replace each particle x_t[m] in particle_bag with a particle
  // x_{t+1}[m] sampled from p(x_{t+1} | x_t[m], u_t)


}

