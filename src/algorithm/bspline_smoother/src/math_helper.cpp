#include "bspline_smoother/math_helper.h"

namespace helper
{
  /**
   * @brief 计算两个节点之间的距离。
   * @param n1 节点1
   * @param n2 节点2
   * @return 节点之间的距离
   */
  double dist(const std::pair<double, double> &node1, const std::pair<double, double> &node2)
  {
    return std::hypot(node1.first - node2.first, node1.second - node2.second);
  }

  double dist(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2)
  {
    return std::hypot(point1.x() - point2.x(), point1.y() - point2.y());
  }

  /**
   * @brief 计算两个节点之间相对于x轴的角度。
   * @param n1 节点1
   * @param n2 节点2
   * @return 两个节点之间相对于x轴的角度
   */
  double angle(const std::pair<double, double> &node1, const std::pair<double, double> &node2)
  {
    return atan2(node2.second - node1.second, node2.first - node1.first);
  }

  /**
   * @brief 对2π进行取模操作。
   * @param theta    要取模的角度
   * @return theta_m 取模后的角度
   */
  double mod2pi(double theta)
  {
    return theta - 2.0 * M_PI * floor(theta / M_PI / 2.0);
  }

  /**
   * @brief 将角度截断到-π到π的区间。
   * @param theta    要截断的角度
   * @return theta_t 截断后的角度
   */
  double pi2pi(double theta)
  {
    while (theta > M_PI)
      theta -= 2.0 * M_PI;
    while (theta < -M_PI)
      theta += 2.0 * M_PI;
    return theta;
  }

  /**
   * @brief 计算与以原点为中心的圆相交的直线的交点公式
   * @note  https://mathworld.wolfram.com/Circle-LineIntersection.html
   * @param p1/p2     线段上的两个点
   * @param r         以原点为中心的圆的半径
   * @return points   直线与圆的交点
   */
  std::vector<std::pair<double, double>> circleSegmentIntersection(const std::pair<double, double> &p1,
                                                                   const std::pair<double, double> &p2, double r)
  {
    std::vector<std::pair<double, double>> i_points;

    double x1 = p1.first;
    double x2 = p2.first;
    double y1 = p1.second;
    double y2 = p2.second;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double dr2 = dx * dx + dy * dy;
    double D = x1 * y2 - x2 * y1;

    // 第一个元素是线段内的交点
    double d1 = x1 * x1 + y1 * y1;
    double d2 = x2 * x2 + y2 * y2;
    double dd = d2 - d1;

    double delta = std::sqrt(r * r * dr2 - D * D);

    if (delta >= 0)
    {
      if (delta == 0)
        i_points.emplace_back(D * dy / dr2, -D * dx / dr2);
      else
      {
        i_points.emplace_back((D * dy + std::copysign(1.0, dd) * dx * delta) / dr2,
                              (-D * dx + std::copysign(1.0, dd) * dy * delta) / dr2);
        i_points.emplace_back((D * dy - std::copysign(1.0, dd) * dx * delta) / dr2,
                              (-D * dx - std::copysign(1.0, dd) * dy * delta) / dr2);
      }
    }

    return i_points;
  }
} // namespace helper