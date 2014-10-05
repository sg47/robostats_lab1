#include <iostream>
#include <ctime>
#include <math.h>
#include <unistd.h>
#include <numeric>
#include <string>

#include <boost/tokenizer.hpp>
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

  return true;
}

double ParticleFilter::getOccValueAtXY(const double x, const double y)
{
  if (x < map_min_x || x > map_max_x || y < map_min_y || y > map_max_y)
      return -1.0;
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
  ros::Rate rr(sim_rate);

  for (std::vector <boost::variant<laser_data_t, arma::vec3> >::iterator iter=stampedData.begin();
       iter!=stampedData.end(); ++iter)
  {
    if(!ros::ok())
      break;

    // 0 = Laser, 1 = Odom
    if (iter->which() == 0)
    {
      laser_data_t ld = boost::get<laser_data_t>(*iter);
      correctionUpdate(ld.ranges);
      resampleImportance();
    }
    else
    {
      arma::vec3 od = boost::get<arma::vec3>(*iter);
      arma::vec3 pose_delta = getPoseDelta(prev_odom, od);
      processUpdate(pose_delta);
      prev_odom = od;
    }

    visualize();
    ros::spinOnce();
    rr.sleep();
  }
  ROS_WARN("Finished processing all data");
}

bool ParticleFilter::loadParameters(const ros::NodeHandle& n)
{
  if (!pu::get("sim_rate", sim_rate)) return false;

  if (!pu::get("algorithm/num_particles", num_particles)) return false;
  if (!pu::get("algorithm/sigma/dx", sigma_dx)) return false;
  if (!pu::get("algorithm/sigma/dy", sigma_dy)) return false;
  if (!pu::get("algorithm/sigma/dyaw", sigma_dyaw)) return false;
  if (!pu::get("algorithm/sigma/laser/hit", laser_hit_sigma)) return false;

  if (!pu::get("algorithm/cell_full_threshold", cell_full_threshold)) return false;
  if ((cell_full_threshold > 1.0) || (cell_full_threshold < 0.0))
  {
    ROS_WARN("cell_full_threshold is a probability, must be between 0 and 1");
    return false;
  }

  if (!pu::get("algorithm/cell_empty_threshold", cell_empty_threshold)) return false;
  if ((cell_empty_threshold > 1.0) || (cell_empty_threshold < 0.0))
  {
    ROS_WARN("cell_empty_threshold is a probability, must be between 0 and 1");
    return false;
  }

  if (!pu::get("algorithm/w_hit", w_hit)) return false;
  if ((w_hit > 1.0) || (w_hit < 0.0))
  {
    ROS_WARN("w_hit is a probability, must be between 0 and 1");
    return false;
  }
  else
    w_max = 1.0 - w_hit;

  if (!pu::get("algorithm/laser_offset", laser_offset)) return false;
  if (!pu::get("algorithm/laser_max_range", laser_max_range)) return false;
  if (!pu::get("algorithm/ray_stepsize", ray_stepsize)) return false;

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
  map_min_x = 0.0;
  map_min_y = 0.0;
  map_max_x = MAP_RESOLUTION*MAP_WIDTH;
  map_max_y = MAP_RESOLUTION*MAP_HEIGHT;

  // seed noise distribution and random number generator
  double seed_value = ros::WallTime::now().toSec();
  rng.seed(seed_value);

  boost::uniform_real<double> x_uniform(map_min_x, map_max_x);
  boost::variate_generator<boost::mt19937,
    boost::uniform_real<double> > x_rand(rng, x_uniform);

  rng.seed(seed_value - 1000.0);

  boost::uniform_real<double> y_uniform(map_min_y, map_max_y);
  boost::variate_generator<boost::mt19937,
    boost::uniform_real<double> > y_rand(rng, y_uniform);

  rng.seed(seed_value - 100000.0);

  boost::uniform_real<double> yaw_uniform(-M_PI, M_PI);
  boost::variate_generator<boost::mt19937,
    boost::uniform_real<double> > yaw_rand(rng, yaw_uniform);

  double p;
  double lw = log(1.0/num_particles);
  for (unsigned int i = 0; i < num_particles; i++)
  {
    bool cell_is_free = false;
    double x, y;
    while(!cell_is_free)
    {
      x = x_rand();
      y = y_rand();
      p = getOccValueAtXY(x, y);
      if((p > -0.5) && (p < 0.1))
        cell_is_free = true;
    }
    particle_t p;
    p.weight = lw;
    p.pose << x << y << yaw_rand();
    particle_bag.push_back(p);
  }
}

bool ParticleFilter::loadData(const std::string& data_path)
{
  max_range_int = 0;
  bool first_data_reached = false;
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
            ld.ranges(idx-7) = atof(beg->c_str())/100.0;
          }
          idx++;
        }
        stampedData.push_back(ld);
      }
      else      // Load odom data entry
      {
        arma::vec3 od;
        sscanf(line.c_str(),"%*c %lf %lf %lf %lf",&od(0),&od(1),&od(2),&ts);
        od /= 100.0;
        if(!first_data_reached)
        {
          prev_odom = od;
          first_data_reached = true;
        }
        stampedData.push_back(od);
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
        val = val<0 ? val : 1.0 - val;
        occ_grid_matrix(mapfile_line_idx-7, col_idx) = val;
        col_idx++;
      }

      occ_grid_msg.data.clear();
      for (unsigned int i=0; i<800; i++)
        for (unsigned int j=0; j<800; j++)
        {
          double val = occ_grid_matrix(i,j);
          val = val<0 ? val : 100*val;
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
  occ_grid_msg.info.resolution = 0.10;
  occ_grid_msg.info.width = MAP_WIDTH;
  occ_grid_msg.info.height = MAP_HEIGHT;

  map_pub.publish(occ_grid_msg);
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

arma::vec3 ParticleFilter::getPoseDelta(const arma::vec3& before,
                                  const arma::vec3& after)
{
  double yaw_prev = before(2);
  double sy = sin(yaw_prev);
  double cy = cos(yaw_prev);
  double dxw = after(0) - before(0);
  double dyw = after(1) - before(1);

  arma::vec3 delta;

  delta(0) = cy*dxw + sy*dyw;
  delta(1) = -sy*dxw + cy*dyw;
  delta(2) = shortest_angular_distance(yaw_prev, after(2));

  return delta;
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
  // seed noise distribution and random number generator
  double seed_value = ros::WallTime::now().toSec();
  rng.seed(seed_value);

  // Generate Gaussians about components of u with sigma_dx, sigma_dy, sigma_dyaw
  static boost::normal_distribution<double> gauss_dx(0.0, sigma_dx);
  static boost::variate_generator<boost::mt19937,
    boost::normal_distribution<double> > ndx(rng, gauss_dx);

  rng.seed(seed_value - 1000.0);

  static boost::normal_distribution<double> gauss_dy(0.0, sigma_dy);
  static boost::variate_generator<boost::mt19937,
    boost::normal_distribution<double> > ndy(rng, gauss_dy);

  rng.seed(seed_value - 5000.0);

  static boost::normal_distribution<double> gauss_dyaw(0.0, sigma_dyaw);
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

void ParticleFilter::correctionUpdate(const arma::vec::fixed<180>& ranges)
{
  double total_weight = 0.0;
  for (unsigned int i = 0; i < particle_bag.size(); i++)
  {
     double lsp = getLogScanProbability(particle_bag[i].pose, ranges);
     particle_bag[i].weight += lsp;
     total_weight += exp(particle_bag[i].weight);
  }

  // normalize weights
  for (unsigned int i = 0; i < particle_bag.size(); i++)
     particle_bag[i].weight -= log(total_weight);
}

double ParticleFilter::getLogScanProbability(const arma::vec3& robot_pose,
                                             const arma::vec::fixed<180>& ranges)
{
  double x_laser = robot_pose(0) + laser_offset*cos(robot_pose(2));
  double y_laser = robot_pose(1) + laser_offset*sin(robot_pose(2));

  // assume that laser beam range returns are independent
  // P(z_{1:180} | x) = product_{i=1:180} P(z_i | x)
  // log P(z_{1:180} | x) = sum_{i=1:180} log P(z_i | x)

  double lp = 0.0;

  // there are 180 - 1 = 179 spaces between entries
  double conv_factor = M_PI/179.0;

  for (unsigned int j = 0; j < 180; j++)
  {
    // j = 0 corresponds to theta = -PI/2, j = 180 correspond to theta = PI/2
    // j = 90 corresponds with theta = 0 (positive laser x-axis)
    double theta_rad = static_cast<double>(j)*conv_factor - M_PI/2.0;
    double z_pred = predictLaserRange(x_laser, y_laser, robot_pose(2) + theta_rad);

    double mean_shift = ranges(j) - z_pred;

    lp -= mean_shift*mean_shift/(2*laser_hit_sigma*laser_hit_sigma);
  }

  return lp;
}

double ParticleFilter::predictLaserRange(const double x_laser,
                                         const double y_laser,
                                         const double ray_yaw)
{
   double x_cur = x_laser;
   double y_cur = y_laser;
   double range = 0.0;
   double cy = cos(ray_yaw);
   double sy = sin(ray_yaw);

#if 0
   printf("cell_full_threshold = %f \t laser_max_range = %f \n",
          cell_full_threshold, laser_max_range);

   printf("prob occupied = %f \t range = %f \n",
            getOccValueAtXY(x_cur, y_cur), range);
#endif

   // while (x_cur, y_cur) is unoccupied
   while(getOccValueAtXY(x_cur, y_cur) < cell_full_threshold)
   {
     //printf("prob occupied = %f \t range = %f \n",
     //       getOccValueAtXY(x_cur, y_cur), range);
     if(range > laser_max_range)
     {
       //ROS_ERROR("range > laser_max_range");
       return laser_max_range;
     }

     // return random range value in [range, laser_max_range] if
     // we have exceeded the bounds of the known map
     if(getOccValueAtXY(x_cur, y_cur) < -0.5)
     {
       rng.seed(ros::WallTime::now().toSec());
       boost::uniform_real<double> r_uniform(range, laser_max_range);
       boost::variate_generator<boost::mt19937,
         boost::uniform_real<double> > r_rand(rng, r_uniform);
       //ROS_WARN("UNKNOWN cell");
       return r_rand();
     }

     range += ray_stepsize;
     x_cur += ray_stepsize*cy;
     y_cur += ray_stepsize*sy;
   }
   return range;
}

void ParticleFilter::resampleImportance()
{
  std::vector<particle_t> new_particles;

  arma::vec weights(particle_bag.size());

  for(unsigned int i = 0; i < particle_bag.size(); i++)
    weights(i) = particle_bag[i].weight;

  //weights.print("weights");

  for(unsigned int i = 0; i < particle_bag.size(); i++)
    addNewParticle(weights, new_particles);

  particle_bag = new_particles;
}

void ParticleFilter::addNewParticle(const arma::vec& weights,
                                    std::vector<particle_t>& target)
{
  arma::vec cw = arma::cumsum(arma::exp(weights));

  rng.seed(ros::WallTime::now().toSec());

  double lw = log(1.0/particle_bag.size());

  static boost::uniform_real<double> w_uniform(0.0,1.0);
  static boost::variate_generator<boost::mt19937,
    boost::uniform_real<double> > w_rand(rng, w_uniform);

  double wr = w_rand();

  for (unsigned int i=0; i<particle_bag.size(); i++)
  {
    if (wr < cw(i))
    {
      particle_t p;
      p.pose = particle_bag[i].pose;
      p.weight = lw;
//      particle_bag[i].print();
      target.push_back(p);
      break;
    }
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
  sensor_msgs::PointCloud pcld;
  pcld.header.frame_id = fixed_frame_id;
  pcld.points.resize(particle_bag.size());
  for (unsigned int i = 0; i < particle_bag.size(); i++)
  {
    pcld.points[i].x = particle_bag[i].pose(0);
    pcld.points[i].y = particle_bag[i].pose(1);
    pcld.points[i].z = 0.0;
  }
  particle_pub.publish(pcld);
}

