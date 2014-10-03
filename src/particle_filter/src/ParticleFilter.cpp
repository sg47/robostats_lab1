#include <iostream>
#include <ctime>
#include <math.h>
#include <unistd.h>
#include <string>

#include<boost/tokenizer.hpp>

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
  cout << "Hello World.. map" << endl;

  loadData();
  cout << "Loaded " << stampedData.size() << " data entries" << endl;

  loadMap();
  cout << "Loaded occupancy grid with " << occ_grid_msg.data.size() << " cells" << endl;

  return true;
}

bool ParticleFilter::registerCallbacks(const ros::NodeHandle& n)
{
  ros::NodeHandle ln(n);
  map_pub = ln.advertise<nav_msgs::OccupancyGrid>("map", 1);

  drawmap_timer = ln.createTimer(ros::Duration(1.0), &ParticleFilter::publishMap, this);

  return true;
}


bool ParticleFilter::loadData()
{
  string line;
  ifstream datafile ("/home/vishnu/Documents/RoboStats/robostats_lab1/data/log/robotdata1.log");
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


bool ParticleFilter::loadMap()
{
  string line;
  ifstream datafile ("/home/vishnu/Documents/RoboStats/robostats_lab1/data/map/wean.dat");
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


