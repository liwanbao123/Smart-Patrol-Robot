/**
 * *********************************************************
 *
 * @file: distance_layer.cpp
 * @brief: euclidean distance layer plugin for costmap
 * @author: Yang Haodong
 * @date: 2024-06-28
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */

#include "distance_layer.h"

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::DistanceLayer, nav2_costmap_2d::Layer)

namespace nav2_costmap_2d
{
void DistanceLayer::onInitialize()
{
  auto node = node_.lock();
  current_ = true;
  clock_ = node->get_clock();
  edf_manager_ = std::make_unique<euclidean_distance_field::EuclideanDistanceField>();
  distance_grid_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("/distance_field", 1);
  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);
}

void DistanceLayer::reset()
{
  return;
}

bool DistanceLayer::isClearable()
{
  return false;
}

const std::vector<std::vector<double>>& DistanceLayer::getEDF() const
{
  return edf_;
}

double DistanceLayer::getDistance(double x, double y)
{
  return edf_manager_->getDistance(x, y);
}

void DistanceLayer::getGradient(double x, double y, double& gx, double& gy)
{
  edf_manager_->getGradient(x, y, gx, gy);
}

boost::mutex& DistanceLayer::getMutex()
{
  return mutex_;
}

void DistanceLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                 double* max_x, double* max_y)
{
  if (!enabled_)
  {
    return;
  }
}

void DistanceLayer::updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_)
  {
    return;
  }

  boost::unique_lock<boost::mutex> lock(mutex_);
  edf_manager_->setGridMap(master_grid.getCharMap(), master_grid.getSizeInCellsX(), master_grid.getSizeInCellsY());
  edf_manager_->compute2d();
  edf_ = edf_manager_->getEDF();
  publishDistanceMap(master_grid);
}

void DistanceLayer::publishDistanceMap(const nav2_costmap_2d::Costmap2D& master_grid)
{
  unsigned int nx = master_grid.getSizeInCellsX();
  unsigned int ny = master_grid.getSizeInCellsY();
  double resolution = master_grid.getResolution();
  nav_msgs::msg::OccupancyGrid grid;

  grid.header.frame_id = "map";
  grid.header.stamp = clock_->now();
  grid.info.resolution = resolution;
  grid.info.width = nx;
  grid.info.height = ny;
  double wx, wy;
  master_grid.mapToWorld(0, 0, wx, wy);
  grid.info.origin.position.x = wx - resolution / 2;
  grid.info.origin.position.y = wy - resolution / 2;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.resize(nx * ny);

  double max_dist = -std::numeric_limits<double>::max();
  for (unsigned int y = 0; y < ny; y++)
    max_dist = std::max(max_dist, *std::max_element(edf_[y].begin(), edf_[y].end()));

  for (unsigned int x = 0; x < nx; x++)
    for (unsigned int y = 0; y < ny; y++)
      grid.data[x + y * nx] = static_cast<unsigned int>(-100.0f / max_dist * edf_[y][x] + 100.0f);

  distance_grid_pub_->publish(grid);
}

}  // namespace nav2_costmap_2d