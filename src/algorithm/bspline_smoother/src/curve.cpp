#include <cassert>
#include "bspline_smoother/curve.h"

namespace trajectory_generation
{
  /**
   * @brief 构造一个新的 Curve 对象
   * @param step  模拟或插值步长
   */
  Curve::Curve(double step) : step_(step)
  {
  }

  /**
   * @brief 计算给定路径的长度
   * @param path    轨迹
   * @return length 路径的长度
   */
  double Curve::len(Points2d path)
  {
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i)
      length += helper::dist(path[i - 1], path[i]);
    return length;
  }

  /**
   * @brief 配置模拟步长
   * @param step    模拟或插值步长
   */
  void Curve::setStep(double step)
  {
    assert(step > 0);
    step_ = step;
  }
} // namespace trajectory_generation