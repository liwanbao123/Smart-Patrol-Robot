#include "global_planner.h"

namespace global_planner
{
/**
 * @brief 构造一个新的全局规划器对象
 * @param nx         代价地图x方向的像素数量
 * @param ny         代价地图y方向的像素数量
 * @param resolution 代价地图分辨率
 */
GlobalPlanner::GlobalPlanner(int nx, int ny, double resolution)
  : lethal_cost_(LETHAL_COST), neutral_cost_(NEUTRAL_COST), factor_(OBSTACLE_FACTOR)
{
  setSize(nx, ny);
  setResolution(resolution);
}

/**
 * @brief 设置或重置代价地图的大小
 * @param nx 代价地图x方向的像素数量
 * @param ny 代价地图y方向的像素数量
 */
void GlobalPlanner::setSize(int nx, int ny)
{
  nx_ = nx;
  ny_ = ny;
  ns_ = nx * ny;
}

/**
 * @brief 设置或重置代价地图的分辨率
 * @param resolution 代价地图分辨率
 */
void GlobalPlanner::setResolution(double resolution)
{
  resolution_ = resolution;
}

void GlobalPlanner::setLethalCost(unsigned char lethal_cost)
{
  lethal_cost_ = lethal_cost;
}

/**
 * @brief 设置或重置中性代价
 * @param neutral_cost 中性代价
 */
void GlobalPlanner::setNeutralCost(unsigned char neutral_cost)
{
  neutral_cost_ = neutral_cost;
}

/**
 * @brief 设置或重置障碍因子
 * @param factor 障碍因子
 */
void GlobalPlanner::setFactor(double factor)
{
  factor_ = factor;
}

/**
 * @brief 设置或重置代价地图的原点
 * @param origin_x  代价地图x方向的原点
 * @param origin_y  代价地图y方向的原点
 */
void GlobalPlanner::setOrigin(double origin_x, double origin_y)
{
  origin_x_ = origin_x;
  origin_y_ = origin_y;
}

/**
 * @brief 设置转换偏移量
 * @param convert_offset 转换偏移量
 */
void GlobalPlanner::setConvertOffset(double convert_offset)
{
  convert_offset_ = convert_offset;
}

/**
 * @brief 从网格地图(x, y)转换为网格索引(i)
 * @param x 网格地图x
 * @param y 网格地图y
 * @return 索引
 */
int GlobalPlanner::grid2Index(int x, int y)
{
  return x + nx_ * y;
}

/**
 * @brief 从网格索引(i)转换为网格地图(x, y)
 * @param i 网格索引i
 * @param x 网格地图x
 * @param y 网格地图y
 */
void GlobalPlanner::index2Grid(int i, int& x, int& y)
{
  x = i % nx_;
  y = i / nx_;
}

/**
 * @brief 从网格地图(x, y)转换为代价地图(x, y)
 * @param gx 网格地图x
 * @param gy 网格地图y
 * @param mx 代价地图x
 * @param my 代价地图y
 */
void GlobalPlanner::map2Grid(double mx, double my, int& gx, int& gy)
{
  gx = (int)mx;
  gy = (int)my;
}

/**
 * @brief 从代价地图(x, y)转换为网格地图(x, y)
 * @param gx 网格地图x
 * @param gy 网格地图y
 * @param mx 代价地图x
 * @param my 代价地图y
 */
void GlobalPlanner::grid2Map(int gx, int gy, double& mx, double& my)
{
  mx = resolution_ * (gx + 0.5);
  my = resolution_ * (gy + 0.5);
}

/**
 * @brief 从世界地图(x, y)转换为代价地图(x, y)
 * @param mx 代价地图x
 * @param my 代价地图y
 * @param wx 世界地图x
 * @param wy 世界地图y
 * @return 如果成功返回true，否则返回false
 */
bool GlobalPlanner::world2Map(double wx, double wy, double& mx, double& my)
{
  if (wx < origin_x_ || wy < origin_y_)
    return false;

  mx = (wx - origin_x_) / resolution_ - convert_offset_;
  my = (wy - origin_y_) / resolution_ - convert_offset_;
  if (mx < nx_ && my < ny_)
    return true;

  return false;
}

/**
 * @brief 从代价地图(x, y)转换为世界地图(x, y)
 * @param mx 代价地图x
 * @param my 代价地图y
 * @param wx 世界地图x
 * @param wy 世界地图y
 */
void GlobalPlanner::map2World(double mx, double my, double& wx, double& wy)
{
  wx = origin_x_ + (mx + convert_offset_) * resolution_;
  wy = origin_y_ + (my + convert_offset_) * resolution_;
}

/**
 * @brief 将代价地图的边界膨胀为障碍物以防止路径穿越
 * @param costarr 代价地图指针
 */
void GlobalPlanner::outlineMap(unsigned char* costarr)
{
  unsigned char* pc = costarr;
  for (int i = 0; i < nx_; i++)
    *pc++ = nav2_costmap_2d::LETHAL_OBSTACLE;
  pc = costarr + (ny_ - 1) * nx_;
  for (int i = 0; i < nx_; i++)
    *pc++ = nav2_costmap_2d::LETHAL_OBSTACLE;
  pc = costarr;
  for (int i = 0; i < ny_; i++, pc += nx_)
    *pc = nav2_costmap_2d::LETHAL_OBSTACLE;
  pc = costarr + nx_ - 1;
  for (int i = 0; i < ny_; i++, pc += nx_)
    *pc = nav2_costmap_2d::LETHAL_OBSTACLE;
}

/**
 * @brief 将关闭列表转换为路径
 * @param closed_list 关闭列表
 * @param start       起始节点
 * @param goal        目标节点
 * @return 包含路径节点的向量
 */
std::vector<Node> GlobalPlanner::_convertClosedListToPath(std::unordered_map<int, Node>& closed_list, const Node& start,
                                                          const Node& goal)
{
  std::vector<Node> path;
  auto current = closed_list.find(goal.id_);
  while (current->second != start)
  {
    path.emplace_back(current->second.x_, current->second.y_);
    auto it = closed_list.find(current->second.pid_);
    if (it != closed_list.end())
      current = it;
    else
      return {};
  }
  path.push_back(start);
  return path;
}

}  // namespace global_planner
