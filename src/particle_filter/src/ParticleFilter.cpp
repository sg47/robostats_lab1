#include <iostream>
#include <ctime>
#include <math.h>
#include <unistd.h>
#include <numeric>
#include <string>

#include<boost/tokenizer.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/math/distributions/normal.hpp>

#include <particle_filter/ParticleFilter.h>

namespace pu = parameter_utils;
using namespace std;

ParticleFilter::ParticleFilter() {}

ParticleFilter::~ParticleFilter() {}

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

int ParticleFilter::getOccValueAtXY(const double x, const double y)
{
  unsigned int row, col;
  getIndiciesFromXY(x, y, row, col);
  return occ_grid_matrix(row,col);
}

void ParticleFilter::getIndiciesFromXY(const double x, const double y,
                                       unsigned int& row, unsigned int& col)
{
  // TODO: check whether cells are corner aligned or center aligned, x-y alignment
  col = floor(x/MAP_RESOLUTION);
  row = floor(y/MAP_RESOLUTION);
}

void ParticleFilter::run()
{
  // TODO: iterate through EVERYTHING
}

bool ParticleFilter::loadParameters(const ros::NodeHandle& n)
{
  std::cout << "Hello World.. map" << std::endl;

  if (!pu::get("algorithm/num_particles", num_particles)) return false;
  if (!pu::get("algorithm/sigma/dx", sigma_dx)) return false;
  if (!pu::get("algorithm/sigma/dy", sigma_dy)) return false;
  if (!pu::get("algorithm/sigma/dyaw", sigma_dyaw)) return false;

  if (!pu::get("frame_id/fixed", fixed_frame_id)) return false;
  if (!pu::get("frame_id/base", base_frame_id)) return false;
  if (!pu::get("frame_id/odom", odom_frame_id)) return false;
  if (!pu::get("frame_id/laser", laser_frame_id)) return false;

  if (!pu::get("files/data", data_file_name)) return false;
  if (!pu::get("files/map", map_file_name)) return false;
  if (!pu::get("files/path", base_path)) return false;

  loadData(base_path + "data/log/" + data_file_name);
  std::cout << "Loaded " << stampedData.size() << " data entries" << std::endl;

  loadMap(base_path + "data/map/" + map_file_name);
  std::cout << "Loaded occupancy grid with " << occ_grid_msg.data.size() << " cells" << std::endl;

  return true;
}

bool ParticleFilter::registerCallbacks(const ros::NodeHandle& n)
{
  ros::NodeHandle nl(n, "");

  map_pub = nl.advertise<nav_msgs::OccupancyGrid>("map", 1);

  drawmap_timer = nl.createTimer(ros::Duration(1.0), &ParticleFilter::publishMap, this);

  particle_pub =
    nl.advertise<sensor_msgs::PointCloud>("particles", 10, false);

  return true;
}

void ParticleFilter::initializeParticles()
{
#if 0
  boost::uniform_real<double> x_uniform(map_min_x, map_max_x);
  boost::uniform_real<double> y_uniform(map_min_y, map_max_y);
  boost::uniform_real<double> yaw_uniform(-M_PI, M_PI);

  boost::variate_generator<boost::mt19937,
    boost::uniform_real<double> > x_rand(rng, x_uniform);
  boost::variate_generator<boost::mt19937,
    boost::uniform_real<double> > y_rand(rng, y_uniform);
  boost::variate_generator<boost::mt19937,
    boost::uniform_real<double> > yaw_rand(rng, yaw_uniform);
#endif

  // TODO: for initializing particles, randomly generate them and only accept them for initialization if they lie in a known (as opposed to an unknown) grid cell

  for (unsigned int i = 0; i < num_particles; i++)
  {
#if 0
    bool cell_is_known = false;
    double x, y;
    while(!cell_is_known)
    {
      x = x_rand();
      y = y_rand();
      if cell at (x,y) is known
        cell_is_known = true;
    }
    particle_t p(x, y, yaw_rand(), 1.0/num_particles);
    particle_bag.push_back(p);
#endif
  }

}

bool ParticleFilter::loadData(const std::string& data_path)
{
  string line;
  ifstream datafile (data_path.c_str());
  if (datafile.is_open())
  {
    double ts;
    while ( getline (datafile,line))
    {
      if (line[0] == 'L') // Load laser data entry
      {
        laser_data_t ld;
        unsigned int idx = 0;
        boost::escaped_list_separator<char> els(""," ","");
        boost::tokenizer<boost::escaped_list_separator<char> > tok(line, els);
        for(boost::tokenizer<boost::escaped_list_separator<char> >::iterator beg=tok.begin(); beg!=tok.end();++beg)
        {
          if (idx > 0 && idx < 4)
          {
              ld.robot_pose(idx-1) = atof(beg->c_str());
          }
          else if (idx >= 4 && idx < 7)
          {
              ld.laser_pose(idx-4) = atof(beg->c_str());
          }
          else if (idx == 187)
          {
            ts = atof(beg->c_str());
          }
          else if (idx >= 7)
          {
            ld.ranges(idx-7) = atoi(beg->c_str());
          }
          idx++;
        }
        stampedData[ts] = ld;
      }
      else      // Load odom data entry
      {
        arma::vec3 od;
        sscanf(line.c_str(),"%*c %lf %lf %lf %lf",&od(0),&od(1),&od(2),&ts);
        stampedData[ts] = od;
      }
    }
    datafile.close();
    return true;
  }
  else
  {
    cout << "Unable to open data file" << endl;
    return false;
  }
}


bool ParticleFilter::loadMap(const std::string& map_path)
{
  string line;
  ifstream datafile (map_path.c_str());
  if (datafile.is_open())
  {
    double ts;
    unsigned int mapfile_line_idx = 0;
    while ( getline (datafile,line))
    {
      if (mapfile_line_idx<7)
      {
        mapfile_line_idx++;
        continue;
      }

      boost::escaped_list_separator<char> els(""," ","");
      boost::tokenizer<boost::escaped_list_separator<char> > tok(line, els);
      unsigned int col_idx = 0;
      for(boost::tokenizer<boost::escaped_list_separator<char> >::iterator beg=tok.begin(); beg!=tok.end();++beg)
      {
        if (col_idx==800)
          continue;

        double val = atof(beg->c_str());
        occ_grid_matrix(mapfile_line_idx-7, col_idx) = val;
        col_idx++;
      }

      occ_grid_msg.data.clear();
      for (unsigned int i=0; i<800; i++)
        for (unsigned int j=0; j<800; j++)
        {
          double val = occ_grid_matrix(i,j);
          val = val<0 ? val : 100*(1-val);
          occ_grid_msg.data.push_back(val);
        }


      mapfile_line_idx++;
    }
    return true;
  }
  else
  {
    cout << "Unable to open map file" << endl;
    return false;
  }

}

void ParticleFilter::publishMap(const ros::TimerEvent& te)
{
  occ_grid_msg.header.stamp = ros::Time::now();
  occ_grid_msg.header.frame_id = "world";
  occ_grid_msg.info.resolution = 10;
  occ_grid_msg.info.width = MAP_WIDTH;
  occ_grid_msg.info.height = MAP_HEIGHT;

  map_pub.publish(occ_grid_msg);
  cout << "published map" << endl;
}


arma::vec3 ParticleFilter::processDynamics(const arma::vec3& pose_in,
                                           const arma::vec3& pose_delta)
{
  arma::vec3 pose_out;

  double sy = sin(pose_in(2));
  double cy = cos(pose_in(2));

  pose_out(0) = pose_in(0) + cy*pose_delta(0) - sy*pose_delta(1);
  pose_out(1) = pose_in(1) + sy*pose_delta(0) + cy*pose_delta(1);
  pose_out(2) = pose_in(2) + pose_delta(2);

  return pose_out;
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
  // Generate Gaussians about components of u with sigma_dx, sigma_dy, sigma_dyaw
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
  for(unsigned int i = 0; i < particle_bag.size(); i++)
  {
    arma::vec3 noisy_delta;
    noisy_delta(0) = u(0) + ndx();
    noisy_delta(1) = u(1) + ndy();
    noisy_delta(2) = u(2) + ndyaw();
    particle_bag[i].pose = processDynamics(particle_bag[i].pose, noisy_delta);
  }
}


void ParticleFilter::printAllParticles(const std::string& prefix) const
{
  if(!prefix.empty())
    printf("%s: \n", prefix.c_str());

  for(unsigned int i = 0; i < particle_bag.size(); i++)
  {
    printf("particle %04d: ", i);
    particle_bag[i].print();
  }
}

void ParticleFilter::visualize()
{
  if(particle_pub.getNumSubscribers())
  {
    sensor_msgs::PointCloud pcld;
    pcld.header.frame_id = fixed_frame_id;
    pcld.points.resize(particle_bag.size());
    for (unsigned int i = 0; i < particle_bag.size(); i++)
    {
      pcld.points[i].x = particle_bag[i].pose(0);
      pcld.points[i].y = particle_bag[i].pose(1);
      pcld.points[i].z = 0.0;
    }
  }
}

