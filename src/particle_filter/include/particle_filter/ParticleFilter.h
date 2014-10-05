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
#include <boost/random/mersenne_twister.hpp>
#include <particle_filter/ParameterUtils.h>
#include <sensor_msgs/PointCloud.h>

#define MAP_RESOLUTION  0.1   // grid cell width (m)
#define MAP_WIDTH       800   // (x_max - x_min)/resolution
#define MAP_HEIGHT      800   // (y_max - y_min)/resolution

class ParticleFilter
{
public:
  ParticleFilter();
  ~ParticleFilter();

  bool initialize(const ros::NodeHandle& n);

  double getOccValueAtXY(const double x, const double y);

  void getIndiciesFromXY(const double x, const double y,
                         unsigned int& row, unsigned int& col);

  void run();

private:
  typedef struct laser_data
  {
     arma::vec3 robot_pose;
     arma::vec3 laser_pose;
     arma::vec::fixed<180> ranges;
  } laser_data_t;

  typedef struct particle
  {
    double weight;
    arma::vec3 pose;
    void print() const
    {
       printf("particle w = %8.5g \t x = %8.5g \t y = %8.5g \t psi = %8.5g \n",
           weight, pose(0), pose(1), pose(2));
    }
  } particle_t;

  static bool sortParticlesByWeight(const particle_t& lhs,
                                    const particle_t& rhs)
  {
    return lhs.weight < rhs.weight;
  }

  bool loadParameters(const ros::NodeHandle& n);
  bool registerCallbacks(const ros::NodeHandle& n);
  void initializeParticles();
  arma::vec3 processDynamics(const arma::vec3& pose_in,
                             const arma::vec3& pose_delta);
  void processUpdate(const arma::vec3& u);
  void correctionUpdate(const arma::vec::fixed<180>& ranges);
  double getLogScanProbability(const arma::vec3& robot_pose,
                            const arma::vec::fixed<180>& ranges);
  double predictLaserRange(const double x_laser, const double y_laser,
                           const double ray_yaw);
  void resampleImportance();
  void addNewParticle(const arma::vec& weights, std::vector<particle_t>& target);

  // geometry utility functions
  arma::vec3 getPoseDelta(const arma::vec3& before, const arma::vec3& after);
  double unroll(double x);
  double normalize_angle(double x);
  double shortest_angular_distance(double from, double to);

  // output functions
  void printAllParticles(const std::string& prefix) const;
  void visualize();
  void publishMap(const ros::TimerEvent& te);

  // data parsing functions
  bool loadData(const std::string& data_path);
  bool loadMap(const std::string& map_path);

  // probability utility functions
  double getUniformRV(double min, double max);
  double getGaussianRV(double mean, double stddev);

  
  // CONSTANT DATA
  double map_min_x;
  double map_min_y;
  double map_max_x;
  double map_max_y;

  unsigned int max_range_int;

  unsigned int num_particles;

  // a container to store laser and odom messages
  std::vector <boost::variant<laser_data_t, arma::vec3> > stampedData;
  
  // standard deviation parameters of the delta pose odometry "control input"
  double sigma_dx, sigma_dy, sigma_dyaw;

  // size of the step taken along a ray to determine the occupancy status of
  // the underlying cell
  double ray_stepsize;

  // probability level above which we consider a cell to be occupied
  double cell_full_threshold;

  // probability level below which we consider a cell to be unoccupied
  // (for the purposes of initializing particles)
  double cell_empty_threshold;

  // offset (in meters) of laser frame from robot frame along +x axis in robot frame
  double laser_offset;

  // --  Laser Range Return Probability Distribution Parameters -----
  // stdev of the laser range return about the correct range
  double laser_hit_sigma;

  // weight for contribution due to noise about correct measurement
  // weight for contribution due to max range laser failure
  double w_hit;
  double w_max;

  double laser_max_range;

  double sim_rate;

  // string parameters
  std::string name;
  std::string fixed_frame_id;
  std::string odom_frame_id;
  std::string laser_frame_id;
  std::string base_frame_id;

  std::string data_file_name;
  std::string map_file_name;
  std::string base_path;

  boost::mt19937 rng;

  // VARIABLE DATA

  // occupancy grid
  arma::mat::fixed<MAP_HEIGHT,MAP_WIDTH> occ_grid_matrix;
  nav_msgs::OccupancyGrid occ_grid_msg;

  // a container for all particles
  std::vector <particle_t> particle_bag;

  arma::vec3 prev_odom;

  // publishers
  ros::Publisher map_pub;
  ros::Publisher particle_pub;

  // timer for map
  ros::Timer drawmap_timer;
};

#endif
