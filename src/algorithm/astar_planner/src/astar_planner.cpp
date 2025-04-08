#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "astar_planner.hpp"

namespace astar_planner
{

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  if (!initialized_)
  {
    initialized_ = true;
    node_ = parent.lock();
    tf_ = tf;
    clock_ = node_->get_clock();

    // 初始化代价地图
    costmap_ = costmap_ros->getCostmap();

    // 代价地图的框架 ID
    frame_id_ = costmap_ros->getGlobalFrameID();

    expand_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>("expand", 1);
  }
}

void AStarPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void AStarPlanner::activate()
{
  nx_ = costmap_->getSizeInCellsX();
  ny_ = costmap_->getSizeInCellsY();
  resolution_ = costmap_->getResolution();
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void AStarPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path plan;
  planner_ = std::make_unique<global_planner::AStar>(nx_, ny_, resolution_);
  // 将代价地图信息传递给规划器（必需）
  planner_->setOrigin(costmap_->getOriginX(), costmap_->getOriginY());
  planner_->setConvertOffset(0.0);
  planner_->costmap_ = costmap_;

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(planner_->costmap_->getMutex()));

  if (!initialized_)
  {
    RCLCPP_ERROR(logger_, "This planner has not been initialized yet, but it is being used, please call initialize() before use");
  }

  // 判断目标和起始节点是否在代价地图框架中
  if (goal.header.frame_id != frame_id_)
  {
    RCLCPP_ERROR(logger_, "The goal pose passed to this planner must be in the %s frame. It is instead in the %s frame.",
              frame_id_.c_str(), goal.header.frame_id.c_str());
  }

  if (start.header.frame_id != frame_id_)
  {
    RCLCPP_ERROR(logger_, "The start pose passed to this planner must be in the %s frame. It is instead in the %s frame.",
              frame_id_.c_str(), start.header.frame_id.c_str());
  }

  // 获取从世界坐标到代价地图的目标和起始节点坐标转换
  double wx = start.pose.position.x, wy = start.pose.position.y;
  unsigned int m_start_x, m_start_y, m_goal_x, m_goal_y;

  planner_->costmap_->worldToMap(wx, wy, m_start_x, m_start_y);
  wx = goal.pose.position.x, wy = goal.pose.position.y;
  planner_->costmap_->worldToMap(wx, wy, m_goal_x, m_goal_y);

   // 从代价地图转换到网格地图
  int g_start_x, g_start_y, g_goal_x, g_goal_y;
  g_start_x = static_cast<int>(m_start_x);
  g_start_y = static_cast<int>(m_start_y);
  g_goal_x = static_cast<int>(m_goal_x);
  g_goal_y = static_cast<int>(m_goal_y);

  Node start_node(g_start_x, g_start_y, 0, 0, planner_->grid2Index(g_start_x, g_start_y), 0);
  Node goal_node(g_goal_x, g_goal_y, 0, 0, planner_->grid2Index(g_goal_x, g_goal_y), 0);
  
  // 计算路径
  std::vector<Node> path;
  std::vector<Node> expand;
  bool path_found = planner_->plan(costmap_->getCharMap(), start_node, goal_node, path, expand);

  // 将路径转换为 ROS 计划
  if (path_found)
  {
    if (_getPlanFromPath(path, plan))
    {
      geometry_msgs::msg::PoseStamped goalCopy = goal;
      goalCopy.header.stamp = clock_->now();
      plan.poses.push_back(goalCopy);
    }
    else
      RCLCPP_ERROR(logger_, "Failed to get a plan from path when a legal path was found. This shouldn't happen.");
  }
  else
    RCLCPP_ERROR(logger_, "Failed to get a path.");

  _publishExpand(expand);

  return plan;
}


/**
 * @brief 发布扩展区域
 * @param expand 扩展节点集合
 */
void AStarPlanner::_publishExpand(std::vector<Node>& expand)
{
  RCLCPP_DEBUG(logger_, "Expand Zone Size: %ld", expand.size());

  nav_msgs::msg::OccupancyGrid grid;

  // 构建扩展区域
  grid.header.frame_id = frame_id_;
  grid.header.stamp = clock_->now();
  grid.info.resolution = resolution_;
  grid.info.width = nx_;
  grid.info.height = ny_;

  double wx, wy;
  costmap_->mapToWorld(0, 0, wx, wy);
  grid.info.origin.position.x = wx - resolution_ / 2;
  grid.info.origin.position.y = wy - resolution_ / 2;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;
  grid.data.resize(nx_ * ny_);

  for (unsigned int i = 0; i < grid.data.size(); i++)
    grid.data[i] = 0;
  for (unsigned int i = 0; i < expand.size(); i++)
    grid.data[expand[i].id_] = 50;

  expand_pub_->publish(grid);
}

/**
 * @brief 从规划路径计算计划
 * @param path 全局规划器生成的路径
 * @param plan 从路径转换的计划，即 [start, ..., goal]
 * @return bool 如果成功返回 true，否则返回 false
 */
bool AStarPlanner::_getPlanFromPath(std::vector<Node>& path, nav_msgs::msg::Path& plan)
{
  if (!initialized_)
  {
    RCLCPP_ERROR(logger_, "This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }

  for (int i = path.size() - 1; i >= 0; i--)
  {
    double wx, wy;
    // planner_->map2World(static_cast<double>(path[i].x_), static_cast<double>(path[i].y_), wx, wy);
    planner_->costmap_->mapToWorld(path[i].x_, path[i].y_, wx, wy);

    // 按消息类型编码
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }

  plan.header.stamp = clock_->now();
  plan.header.frame_id = frame_id_;

  return !plan.poses.empty();
}

}  
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(astar_planner::AStarPlanner, nav2_core::GlobalPlanner)
