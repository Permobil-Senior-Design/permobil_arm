// Converts the collision octomap into a point cloud message
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/transforms.h"
#include "std_msgs/String.h"
#include "moveit_msgs/PlanningScene.h"
#include "sensor_msgs/PointCloud2.h"
#include <sstream>
#include <pcl_conversions/pcl_conversions.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
// #include <octomap/OcTreeLabeled.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

using namespace std;
using namespace octomap;

sensor_msgs::PointCloud2 out_msg;

void cloud_cb(boost::shared_ptr<const moveit_msgs::PlanningScene_<std::allocator<void>>> input)
{
  // Extracting the MoveIt! planning scene world published by /move_group
  moveit_msgs::PlanningScene::Ptr my_planning_scene(new moveit_msgs::PlanningScene);
  *my_planning_scene = *input;
  moveit_msgs::PlanningSceneWorld my_world = (*my_planning_scene).world;

  // Extracting only the OctoMap
  octomap_msgs::OctomapWithPose octomap_pose = my_world.octomap;
  octomap_msgs::Octomap octomap = octomap_pose.octomap;

  // Conversion from octomap_msgs to octomap
  AbstractOcTree *my_abstract_map = octomap_msgs::msgToMap(octomap);

  // Obtaining the actual OctoMap tree
  OcTree *tree = (OcTree *)my_abstract_map;

  unsigned int maxDepth = tree->getTreeDepth();
  cout << "tree depth is " << maxDepth << endl;

  // expand collapsed occupied nodes until all occupied leaves are at maximum depth
  vector<OcTreeNode *> collapsed_occ_nodes;
  do
  {
    collapsed_occ_nodes.clear();
    for (OcTree::iterator it = tree->begin(); it != tree->end(); ++it)
    {
      if (tree->isNodeOccupied(*it) && it.getDepth() < maxDepth)
      {
        collapsed_occ_nodes.push_back(&(*it));
      }
    }
    for (vector<OcTreeNode *>::iterator it = collapsed_occ_nodes.begin(); it != collapsed_occ_nodes.end(); ++it)
    {
      tree->expandNode(*it);
    }
    cout << "expanded " << collapsed_occ_nodes.size() << " nodes" << endl;
  } while (collapsed_occ_nodes.size() > 0);

  vector<point3d> pcl;
  for (OcTree::iterator it = tree->begin(); it != tree->end(); ++it)
  {
    if (tree->isNodeOccupied(*it))
    {
      pcl.push_back(it.getCoordinate());
    }
  }

  ofstream f;
  f.open("$HOME/ws_permobil/src/deep_grasp_demo/moveit_task_constructor_gpd/data/pointclouds/debug.pcd");
  f << "# .PCD v0.7" << endl
    << "VERSION 0.7" << endl
    << "FIELDS x y z" << endl
    << "SIZE 4 4 4" << endl
    << "TYPE F F F" << endl
    << "COUNT 1 1 1" << endl
    << "WIDTH " << pcl.size() << endl
    << "HEIGHT 1" << endl
    << "VIEWPOINT 0 0 0 1 0 0 0" << endl
    << "POINTS " << pcl.size() << endl
    << "DATA ascii" << endl;
  for (size_t i = 0; i < pcl.size(); i++)
    f << pcl[i].x() << " " << pcl[i].y() << " " << pcl[i].z() << endl;
  f.close();
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_converter");
  ros::NodeHandle nh;
  // tf2_ros::Buffer tfBuffer;
  // tf2_ros::TransformListener tfListener(tfBuffer);
  // sleep(5);

  while (ros::ok())
  {
    ROS_INFO("Waiting for octomap");
    boost::shared_ptr<const moveit_msgs::PlanningScene_<std::allocator<void>>> scene = ros::topic::waitForMessage<moveit_msgs::PlanningScene>("/move_group/monitored_planning_scene", nh);
    cloud_cb(scene);
    ROS_INFO("Saved GPD Cloud");
    return 0;
  }
}