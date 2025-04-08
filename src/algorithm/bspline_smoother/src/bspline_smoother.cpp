#include <vector>
#include <memory>
#include "bspline_smoother/bspline_smoother.hpp"

namespace nav2_smoother
{
  void BSplineSmoother::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
      std::string /*name*/, std::shared_ptr<tf2_ros::Buffer> /*tf*/,
      std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub,
      std::shared_ptr<nav2_costmap_2d::FootprintSubscriber> /*footprint_sub*/)
  {
    costmap_sub_ = costmap_sub;
    bspline_gen_ = std::make_shared<trajectory_generation::BSpline>(trajectory_generation::BSpline());
    auto node = parent.lock();
    logger_ = node->get_logger();
  }

  bool BSplineSmoother::smooth(
      nav_msgs::msg::Path &path,
      const rclcpp::Duration & /*max_time*/)
  {
    int seg_num = 48;
    int jump_num = seg_num / 6;
    bool success = true;
    trajectory_generation::Points2d smooth_path;
    for (unsigned int idx = 0; idx < path.poses.size(); idx += seg_num)
    {
      if (idx + seg_num >= path.poses.size())
      {
        for (unsigned int i = idx; i < path.poses.size(); i++)
          smooth_path.emplace_back(path.poses[i].pose.position.x, path.poses[i].pose.position.y);
        break;
      }

      trajectory_generation::Points2d input_seg, output_seg;
      for (unsigned int i = idx; i < idx + seg_num; i += jump_num)
        input_seg.emplace_back(path.poses[i].pose.position.x, path.poses[i].pose.position.y);
      success = success && bspline_gen_->run(input_seg, output_seg);
      for (const auto &os : output_seg)
        smooth_path.emplace_back(os.first, os.second);
    }

    path.poses.clear();
    for (const auto &pt : smooth_path)
    {
      geometry_msgs::msg::PoseStamped p;
      p.pose.position.x = pt.first;
      p.pose.position.y = pt.second;
      path.poses.push_back(p);
    }

    return success;
  }
} // namespace nav2_smoother

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_smoother::BSplineSmoother, nav2_core::Smoother)
