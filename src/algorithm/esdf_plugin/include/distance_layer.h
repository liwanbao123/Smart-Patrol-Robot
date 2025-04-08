/**
 * *********************************************************
 *
 * @file: distance_layer.h
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
#ifndef DISTANCE_LAYER_H
#define DISTANCE_LAYER_H

#include <boost/thread.hpp>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "esdf.h"

namespace nav2_costmap_2d
{
class DistanceLayer : public Layer
{
public:
  DistanceLayer() = default;
  virtual ~DistanceLayer() = default;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                    double* max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override;
  const std::vector<std::vector<double>>& getEDF() const;
  double getDistance(double x, double y);
  void getGradient(double x, double y, double& gx, double& gy);
  boost::mutex& getMutex();

private:
  void publishDistanceMap(const nav2_costmap_2d::Costmap2D& master_grid);

private:
  boost::mutex mutex_;
  std::vector<std::vector<double>> edf_;
  std::unique_ptr<euclidean_distance_field::EuclideanDistanceField> edf_manager_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr distance_grid_pub_;
};

}  // namespace nav2_costmap_2d

#endif