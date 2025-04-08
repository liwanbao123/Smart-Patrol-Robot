#include "a_star.h"

#include <queue>
#include <unordered_set>
#include <vector>

namespace global_planner
{
/**
 * @brief 构造一个新的AStar对象
 * @param nx         costmap在x方向上的像素数量
 * @param ny         costmap在y方向上的像素数量
 * @param resolution costmap分辨率
 * @param dijkstra   是否使用Dijkstra算法
 * @param gbfs       是否使用GBFS算法
 */
AStar::AStar(int nx, int ny, double resolution, bool dijkstra, bool gbfs) : GlobalPlanner(nx, ny, resolution)
{
  // 不能同时使用Dijkstra和GBFS
  if (!(dijkstra && gbfs))
  {
    is_dijkstra_ = dijkstra;
    is_gbfs_ = gbfs;
  }
  else
  {
    is_dijkstra_ = false;
    is_gbfs_ = false;
  }
  factor_ = 0.25;
};

/**
 * @brief A*算法实现
 * @param global_costmap 全局costmap
 * @param start          起始节点
 * @param goal           目标节点
 * @param path           最优路径，由Node组成
 * @param expand         包含搜索过程中访问的节点
 * @return 如果找到路径返回true，否则返回false
 */
bool AStar::plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
                 std::vector<Node>& expand)
{
  // 清空向量
  path.clear();
  expand.clear();
  
  // RCLCPP_INFO(logger_, "size: %i", global_costmap[100000]);

  // 打开列表和关闭列表
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  std::unordered_map<int, Node> closed_list;

  open_list.push(start);

  // 获取所有可能的运动
  const std::vector<Node> motions = Node::getMotion();

  // 主过程
  while (!open_list.empty())
  {
    // 从打开列表中弹出当前节点
    Node current = open_list.top();
    open_list.pop();

    // 当前节点不存在于关闭列表中
    if (closed_list.find(current.id_) != closed_list.end())
      continue;

    closed_list.insert(std::make_pair(current.id_, current));
    expand.push_back(current);
    // RCLCPP_INFO(logger_, "cx: %i, cy: %i, cid: %i, pid: %i", current.x_, current.y_, current.id_, current.pid_);
    // 找到目标
    if (current == goal)
    {
      path = _convertClosedListToPath(closed_list, start, goal);
      return true;
    }

    // 探索当前节点的邻居
    for (const auto& motion : motions)
    {
      // 探索一个新节点
      Node node_new = current + motion;
      node_new.id_ = grid2Index(node_new.x_, node_new.y_);

      // 新节点在关闭列表中
      if (closed_list.find(node_new.id_) != closed_list.end())
        continue;

      node_new.pid_ = current.id_;

      // 下一个节点碰到边界或障碍物
      // 防止当前节点在膨胀区域内时规划失败
      if ((node_new.id_ < 0) || (node_new.id_ >= ns_) ||
          (global_costmap[node_new.id_] >= lethal_cost_ * 0.8 &&
           global_costmap[node_new.id_] >= global_costmap[current.id_]))
        continue;

      // 如果使用Dijkstra算法，不考虑启发式代价
      if (!is_dijkstra_)
        node_new.h_ = helper::dist(node_new, goal);

      // 如果使用GBFS算法，仅考虑启发式代价
      if (is_gbfs_)
        node_new.g_ = 0.0;
      // 否则，g通过node_new = current + m计算

      open_list.push(node_new);
    }
  }

  return false;
}
}  // namespace global_planner
