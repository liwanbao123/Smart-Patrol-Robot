#include <Eigen/Dense>
#include <cassert>
#include "bspline_smoother/bspline_curve.h"

namespace trajectory_generation
{
  /**
   * @brief 构造一个新的 B-Spline 生成对象
   * @param step        模拟或插值步长（默认值：0.01）
   * @param order       曲线的阶数（默认值：3）
   * @param param_mode  参数化模式（默认值：PARAM_MODE_CHORDLENGTH）
   * @param spline_mode B-Spline 生成模式（默认值：SPLINE_MODE_INTERPOLATION）
   */
  BSpline::BSpline(double step, int order, int param_mode, int spline_mode)
      : Curve(step), order_(order), param_mode_(param_mode), spline_mode_(spline_mode)
  {
  }
  BSpline::BSpline()
      : Curve(0.01), order_(3), param_mode_(PARAM_MODE_CENTRIPETAL), spline_mode_(SPLINE_MODE_INTERPOLATION)
  {
  }

  /**
   * @brief 销毁 B-Spline 生成对象
   */
  BSpline::~BSpline()
  {
  }

  /**
   * @brief 使用 Cox-deBoor 函数计算基函数
   * @param i       基函数的索引
   * @param k       曲线的阶数
   * @param t       参数
   * @param knot    节点向量
   * @return  Nik_t 基函数 Nik(t) 的值
   */
  double BSpline::baseFunction(int i, int k, double t, std::vector<double> knot)
  {
    double Nik_t = 0;

    // 一阶 B-Spline
    if (k == 0)
      Nik_t = ((t >= knot[i]) && (t < knot[i + 1])) ? 1.0 : 0.0;
    // 二阶及更高阶 B-Spline
    else
    {
      double length1 = double(knot[i + k]) - knot[i];
      double length2 = double(knot[i + k + 1]) - knot[i + 1];

      // 处理分母为 0 的情况，将其替换为 1，并定义 0/0 为 0
      if ((length1 == 0) && (length2 == 0))
        Nik_t = 0;
      else if (length1 == 0)
        Nik_t = (knot[i + k + 1] - t) / length2 * baseFunction(i + 1, k - 1, t, knot);
      else if (length2 == 0)
        Nik_t = (t - knot[i]) / length1 * baseFunction(i, k - 1, t, knot);
      else
        Nik_t = (t - knot[i]) / length1 * baseFunction(i, k - 1, t, knot) +
                (knot[i + k + 1] - t) / length2 * baseFunction(i + 1, k - 1, t, knot);
    }
    return Nik_t;
  }

  /**
   * @brief 使用 `均匀间隔`、`弦长` 或 `向心` 方法计算参数
   * @param points      路径点
   * @return parameters 给定点的参数
   */
  std::vector<double> BSpline::paramSelection(const Points2d points)
  {
    size_t n = points.size();
    std::vector<double> parameters(n);

    if (param_mode_ == PARAM_MODE_UNIFORMSPACED)
    {
      for (size_t i = 0; i < n; i++)
        parameters[i] = (double)(i) / (double)(n - 1);
    }
    else
    {
      parameters[0] = 0.0;
      std::vector<double> s(n - 1);

      double d_cumsum = 0.0;
      for (size_t i = 0; i < n - 1; i++)
      {
        double d;
        if (param_mode_ == PARAM_MODE_CHORDLENGTH)
          d = helper::dist(points[i], points[i + 1]);
        else
        {
          double alpha = 0.5;
          d = std::pow(helper::dist(points[i], points[i + 1]), alpha);
        }
        d_cumsum += d;
        s[i] = d_cumsum;
      }
      for (size_t i = 1; i < n; i++)
        parameters[i] = s[i - 1] / s[n - 2];
    }
    return parameters;
  }

  /**
   * @brief 生成节点向量
   * @param parameters 给定点的参数
   * @param n          数据点的数量
   * @return knot 节点向量
   */
  std::vector<double> BSpline::knotGeneration(const std::vector<double> param, int n)
  {
    int m = n + order_ + 1;
    std::vector<double> knot(m);

    for (int i = 0; i < n; i++)
      knot[i] = 0.0;
    for (int i = n; i < m; i++)
      knot[i] = 1.0;
    for (int i = order_ + 1; i < n; i++)
    {
      for (int j = i - order_; j < i; j++)
        knot[i] += param[j];
      knot[i] /= order_;
    }
    return knot;
  }

  /**
   * @brief 给定 N 个数据点 D0, D1, ..., Dn 和一个阶数 k，找到一个通过所有数据点的 B-Spline 曲线
   * @param points          路径点
   * @param parameters      给定点的参数
   * @param knot            节点向量
   * @return control_points 控制点
   */
  Points2d BSpline::interpolation(const Points2d points, const std::vector<double> param, const std::vector<double> knot)
  {
    size_t n = points.size();
    Eigen::MatrixXd N = Eigen::MatrixXd::Zero(n, n);
    Eigen::MatrixXd D(n, 2);

    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < n; j++)
        N(i, j) = baseFunction(j, order_, param[i], knot);
    N(n - 1, n - 1) = 1;

    for (size_t i = 0; i < n; i++)
    {
      D(i, 0) = points[i].first;
      D(i, 1) = points[i].second;
    }

    Eigen::MatrixXd C = N.inverse() * D;

    std::vector<std::pair<double, double>> control_points(n);

    for (size_t i = 0; i < n; i++)
      control_points[i] = {C(i, 0), C(i, 1)};

    return control_points;
  }

  /**
   * @brief 给定 N 个数据点 D0, D1, ..., Dn，一个阶数 k 和一个控制点数量 H，找到一个满足条件的 B-Spline 曲线
   * @param points          路径点
   * @param parameters      给定点的参数
   * @param knot            节点向量
   * @return control_points 控制点
   */
  Points2d BSpline::approximation(const Points2d points, const std::vector<double> param, const std::vector<double> knot)
  {
    size_t n = points.size();
    Eigen::MatrixXd D(n, 2);
    for (size_t i = 0; i < n; i++)
    {
      D(i, 0) = points[i].first;
      D(i, 1) = points[i].second;
    }

    // 根据经验设置控制点数量
    size_t h = n - 1;
    Eigen::MatrixXd N = Eigen::MatrixXd::Zero(n, h);
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < h; j++)
        N(i, j) = baseFunction(j, order_, param[i], knot);

    Eigen::MatrixXd N_ = Eigen::MatrixXd::Zero(n - 2, h - 2);
    for (size_t i = 1; i < n - 1; i++)
      for (size_t j = 1; j < h - 1; j++)
        N_(i - 1, j - 1) = N(i, j);

    Eigen::MatrixXd qk = Eigen::MatrixXd::Zero(n - 2, 2);
    for (size_t i = 1; i < n - 1; i++)
    {
      qk(i - 1, 0) = D(i, 0) - N(i, 0) * D(0, 0) - N(i, h - 1) * D(n - 1, 0);
      qk(i - 1, 1) = D(i, 1) - N(i, 0) * D(0, 1) - N(i, h - 1) * D(n - 1, 1);
    }

    Eigen::MatrixXd Q = N_.transpose() * qk;
    Eigen::MatrixXd P = (N_.transpose() * N_).inverse() * Q;

    Points2d control_points(h);
    control_points[0] = {D(0, 0), D(0, 1)};
    control_points[h - 1] = {D(n - 1, 0), D(n - 1, 1)};
    for (size_t i = 1; i < h - 1; i++)
      control_points[i] = {P(i - 1, 0), P(i - 1, 1)};

    return control_points;
  }

  /**
   * @brief 生成路径
   * @param k               曲线的阶数
   * @param knot            节点向量
   * @param control_points  控制点
   * @return path 平滑的轨迹点
   */
  Points2d BSpline::generation(int /**k**/, const std::vector<double> knot, Points2d control_pts)
  {
    size_t n = (int)(1.0 / step_);
    std::vector<double> t(n);
    for (size_t i = 0; i < n; i++)
      t[i] = (double)(i) / (double)(n - 1);

    Eigen::MatrixXd N(n, control_pts.size());
    for (size_t i = 0; i < n; i++)
      for (size_t j = 0; j < control_pts.size(); j++)
        N(i, j) = baseFunction(j, order_, t[i], knot);

    N(n - 1, control_pts.size() - 1) = 1.0;

    Eigen::MatrixXd C(control_pts.size(), 2);
    for (size_t i = 0; i < control_pts.size(); i++)
    {
      C(i, 0) = control_pts[i].first;
      C(i, 1) = control_pts[i].second;
    }

    Eigen::MatrixXd P = N * C;
    Points2d points(n);
    for (size_t i = 0; i < n; i++)
      points[i] = {P(i, 0), P(i, 1)};

    return points;
  }

  /**
   * @brief 运行轨迹生成
   * @param points 路径点
   * @param path 生成的轨迹
   * @return 如果生成成功返回 true，否则返回 false
   */
  bool BSpline::run(const Points2d points, Points2d &path)
  {
    if (points.size() < 4)
      return false;
    else
    {
      Points2d control_pts;
      std::vector<double> params = paramSelection(points);
      std::vector<double> knot = knotGeneration(params, points.size());
      if (spline_mode_ == SPLINE_MODE_INTERPOLATION)
        control_pts = interpolation(points, params, knot);
      else if (spline_mode_ == SPLINE_MODE_APPROXIMATION)
      {
        control_pts = approximation(points, params, knot);
        params = paramSelection(control_pts);
        knot = knotGeneration(params, control_pts.size());
      }
      else
        return false;

      path = generation(order_, knot, control_pts);

      return !path.empty();
    }
  }

  /**
   * @brief 运行轨迹生成
   * @param points 路径点 <x, y, theta>
   * @param path 生成的轨迹
   * @return 如果生成成功返回 true，否则返回 false
   */
  bool BSpline::run(const Poses2d points, Points2d &path)
  {
    Points2d points_pair;
    for (const auto &p : points)
      points_pair.emplace_back(std::get<0>(p), std::get<1>(p));
    return run(points_pair, path);
  }

  /**
   * @brief 配置曲线的阶数
   * @param order  曲线的阶数
   */
  void BSpline::setSplineOrder(int order)
  {
    assert(order > 0);
    order_ = order;
  }

  /**
   * @brief 配置参数化模式
   * @param param_mode  参数化模式
   */
  void BSpline::setParamMode(int param_mode)
  {
    assert((param_mode == PARAM_MODE_CENTRIPETAL) || (param_mode == PARAM_MODE_CHORDLENGTH) ||
           (param_mode == PARAM_MODE_UNIFORMSPACED));
    param_mode_ = param_mode;
  }

  /**
   * @brief 配置 B-Spline 生成模式
   * @param spline_mode  B-Spline 生成模式
   */
  void BSpline::setSPlineMode(int spline_mode)
  {
    assert((spline_mode == SPLINE_MODE_APPROXIMATION) || (spline_mode == SPLINE_MODE_INTERPOLATION));
    spline_mode_ = spline_mode;
  }
} // namespace trajectory_generation
