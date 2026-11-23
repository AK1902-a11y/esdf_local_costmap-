// EsdfLayer: read-only Nav2 costmap layer that keeps a rolling-window incremental distance field
// of the lethal cells of the master costmap. Put it LAST in the plugin list so the master grid is final.
//
// Each costmap update:  (a) origin moved? -> RingEsdf::shift   (b) diff window vs stored occupancy
//                       (c) RingEsdf::update (waves)           (d) rebuild on resize / reset / big jump
// It never writes to the costmap. Consumers query it through distanceAt() / nearestObstacle().
#pragma once
#include <fstream>
#include <memory>
#include <shared_mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "nav2_esdf/ring_esdf.hpp"

namespace nav2_esdf
{

class EsdfLayer : public nav2_costmap_2d::Layer
{
public:
  EsdfLayer();
  ~EsdfLayer() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;
  void matchSize() override;
  bool isClearable() override {return false;}

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j) override;

  // ---- query API (thread safe; world coordinates in the costmap's global frame, metres) ----
  // Distance to the nearest lethal cell, in metres.
  //   NaN  : point outside the window, or the field is not built yet
  //   +inf : inside the window but no obstacle within dmax
  double distanceAt(double wx, double wy) const;
  // World position (cell centre) of the nearest obstacle; false if none within dmax / outside window.
  bool nearestObstacle(double wx, double wy, double & obstacle_x, double & obstacle_y) const;
  double dmaxMeters() const {return dmax_m_;}
  // Copy of the current squared-distance window, row-major [y][x], in cells^2; -1 = no obstacle within dmax.
  // (ox, oy) is the integer world-cell origin of the window. Returns false if the field is not built yet.
  // Used by the FH baseline package to check this field against an exact transform.
  bool snapshotD2(std::vector<int> & d2, int & ox, int & oy, int & nx, int & ny) const;

private:
  struct Timing
  {
    double shift_us = 0, diff_us = 0, waves_us = 0, rebuild_us = 0;
    int dx = 0, dy = 0, entering = 0, orphans = 0, changed = 0;
    bool rebuilt = false;
  };

  void rebuild(int oxc, int oyc, unsigned nx, unsigned ny, double res, Timing & t);
  void publishField(const nav2_costmap_2d::Costmap2D & master, int oxc, int oyc);
  void logStats(const Timing & t, long long pops, long long self_check, double total_us);

  mutable std::shared_mutex mutex_;
  std::unique_ptr<RingEsdf> field_;
  double res_ = 0.05;
  bool need_rebuild_ = true;
  // Window origin tracked as integer cells. Anchored at (re)build, then advanced by the rounded
  // metric delta of the master origin each update, so float noise can never cause an off-by-one.
  int ocx_ = 0, ocy_ = 0;
  double last_origin_x_ = 0.0, last_origin_y_ = 0.0;
  double off_x_ = 0.0, off_y_ = 0.0;     // world = cell * res + off (non-zero only if the origin is not a multiple of res)

  // parameters
  double dmax_m_ = 2.0;
  int lethal_ = 254;
  bool unknown_is_obstacle_ = false;
  int self_check_every_ = 0;
  bool publish_field_ = true;
  double publish_period_ = 0.5;
  std::string stats_file_;

  // bookkeeping
  std::ofstream stats_;
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};
  long long updates_ = 0;
  double sum_total_us_ = 0, max_total_us_ = 0;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
};

}  // namespace nav2_esdf
