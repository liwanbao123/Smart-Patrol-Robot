#ifndef NAV2_SMOOTHER__BSPLINE_SMOOTHER_HPP_
#define NAV2_SMOOTHER__BSPLINE_SMOOTHER_HPP_

#include <cmath>
#include <vector>
#include <string>
#include <iostream>
#include <memory>
#include <queue>
#include <utility>

#include "nav2_core/smoother.hpp"
#include "nav2_smoother/smoother_utils.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/utils.h"
#include "bspline_curve.h"

namespace nav2_smoother
{

  /**
   * @class nav2_smoother::BSplineSmoother
   * @brief A path smoother implementation
   */
  class BSplineSmoother : public nav2_core::Smoother
  {
  public:
    /**
     * @brief A constructor for nav2_smoother::BSplineSmoother
     */
    BSplineSmoother() = default;

    /**
     * @brief A destructor for nav2_smoother::BSplineSmoother
     */
    ~BSplineSmoother() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
        std::string name, std::shared_ptr<tf2_ros::Buffer>,
        std::shared_ptr<nav2_costmap_2d::CostmapSubscriber>,
        std::shared_ptr<nav2_costmap_2d::FootprintSubscriber>) override;

    /**
     * @brief Method to cleanup resources.
     */
    void cleanup() override { costmap_sub_.reset(); }

    /**
     * @brief Method to activate smoother and any threads involved in execution.
     */
    void activate() override { RCLCPP_WARN(logger_, "Using: B-spline smoother"); }

    /**
     * @brief Method to deactivate smoother and any threads involved in execution.
     */
    void deactivate() override {}

    /**
     * @brief Method to smooth given path
     *
     * @param path In-out path to be smoothed
     * @param max_time Maximum duration smoothing should take
     * @return If smoothing was completed (true) or interrupted by time limit (false)
     */
    bool smooth(
        nav_msgs::msg::Path &path,
        const rclcpp::Duration &max_time) override;

    std::shared_ptr<trajectory_generation::BSpline> bspline_gen_;
    std::shared_ptr<nav2_costmap_2d::CostmapSubscriber> costmap_sub_;
    rclcpp::Logger logger_{rclcpp::get_logger("BSplineSmoother")};
  };

} // namespace nav2_smoother

#endif // NAV2_SMOOTHER__SIMPLE_SMOOTHER_HPP_
