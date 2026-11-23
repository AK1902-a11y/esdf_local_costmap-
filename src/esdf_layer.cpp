#include "nav2_esdf/esdf_layer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <mutex>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_esdf::EsdfLayer, nav2_costmap_2d::Layer)

namespace nav2_esdf
{
using Clock = std::chrono::steady_clock;
static double usSince(Clock::time_point t0)
{
  return std::chrono::duration<double, std::micro>(Clock::now() - t0).count();
}

EsdfLayer::EsdfLayer() {}
EsdfLayer::~EsdfLayer() {if (stats_.is_open()) {stats_.close();}}

void EsdfLayer::onInitialize()
{
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("dmax", rclcpp::ParameterValue(2.0));                  // distance cap [m]
  declareParameter("lethal_threshold", rclcpp::ParameterValue(254));      // cost >= this is an obstacle
  declareParameter("unknown_is_obstacle", rclcpp::ParameterValue(false)); // treat NO_INFORMATION (255) as obstacle
  declareParameter("self_check_every", rclcpp::ParameterValue(0));        // debug: bookkeeping check every k updates
  declareParameter("publish_field", rclcpp::ParameterValue(true));
  declareParameter("publish_rate", rclcpp::ParameterValue(2.0));          // [Hz]
  declareParameter("stats_file", rclcpp::ParameterValue(std::string("")));// per-update timing CSV ("" = off)

  const auto node = node_.lock();
  if (!node) {throw std::runtime_error("EsdfLayer::onInitialize: failed to lock node");}
  node->get_parameter(name_ + ".enabled", enabled_);
  node->get_parameter(name_ + ".dmax", dmax_m_);
  node->get_parameter(name_ + ".lethal_threshold", lethal_);
  node->get_parameter(name_ + ".unknown_is_obstacle", unknown_is_obstacle_);
  node->get_parameter(name_ + ".self_check_every", self_check_every_);
  node->get_parameter(name_ + ".publish_field", publish_field_);
  double rate = 2.0;
  node->get_parameter(name_ + ".publish_rate", rate);
  publish_period_ = rate > 0.0 ? 1.0 / rate : 0.0;
  node->get_parameter(name_ + ".stats_file", stats_file_);

  if (!stats_file_.empty()) {
    stats_.open(stats_file_, std::ios::out | std::ios::trunc);
    if (stats_.is_open()) {
      stats_ << "t_sec,origin_x,origin_y,dx,dy,entering,orphans,changed,rebuilt,pops,"
             << "t_shift_us,t_diff_us,t_waves_us,t_rebuild_us,t_total_us,self_check,seq\n";
    } else {
      RCLCPP_WARN(logger_, "EsdfLayer: cannot open stats_file '%s'", stats_file_.c_str());
    }
  }
  if (publish_field_) {
    pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      name_ + "/field", rclcpp::QoS(1).transient_local());
  }
#ifndef NDEBUG
  RCLCPP_WARN(
    logger_, "EsdfLayer was built WITHOUT optimisation (NDEBUG not set): its timings are NOT representative. "
    "Rebuild with -DCMAKE_BUILD_TYPE=Release.");
#endif
  current_ = true;   // otherwise the costmap never reports "current" and Nav2 waits forever
  RCLCPP_INFO(
    logger_, "EsdfLayer '%s': dmax %.2f m, lethal >= %d, unknown_is_obstacle %d, stats '%s'",
    name_.c_str(), dmax_m_, lethal_, static_cast<int>(unknown_is_obstacle_), stats_file_.c_str());
}

void EsdfLayer::activate() {if (pub_) {pub_->on_activate();}}
void EsdfLayer::deactivate() {if (pub_) {pub_->on_deactivate();}}

void EsdfLayer::reset()
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  need_rebuild_ = true;
}

void EsdfLayer::matchSize()
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  need_rebuild_ = true;     // costmap resized: rebuild on the next update
}

void EsdfLayer::updateBounds(
  double robot_x, double robot_y, double, double * min_x, double * min_y, double * max_x, double * max_y)
{
  // Read-only layer: it never writes costs. It only touches the robot's own position so the combined
  // bounds are never empty; LayeredCostmap::updateMap returns before calling any updateCosts() when they
  // are, which would make us miss a window shift. (ObstacleLayer already touches the footprint every
  // cycle, so this is a safeguard, not something the current config relies on.)
  if (!enabled_) {return;}
  *min_x = std::min(*min_x, robot_x); *min_y = std::min(*min_y, robot_y);
  *max_x = std::max(*max_x, robot_x); *max_y = std::max(*max_y, robot_y);
}

void EsdfLayer::rebuild(int oxc, int oyc, unsigned nx, unsigned ny, double res, Timing & t)
{
  auto t0 = Clock::now();
  res_ = res;
  const float dcells = dmax_m_ > 0.0 ? static_cast<float>(dmax_m_ / res) : 1e6f;
  field_ = std::make_unique<RingEsdf>(static_cast<int>(nx), static_cast<int>(ny), dcells);
  t.rebuilt = true;
  (void)oxc; (void)oyc;
  t.rebuild_us = usSince(t0);   // allocation only; the build itself is added by the caller
}

void EsdfLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master, int, int, int, int)
{
  if (!enabled_) {return;}
  const auto t_all = Clock::now();
  std::unique_lock<std::shared_mutex> lock(mutex_);

  const unsigned nx = master.getSizeInCellsX(), ny = master.getSizeInCellsY();
  const double res = master.getResolution();
  const bool rebuild_needed = need_rebuild_ || !field_ || field_->Nx != static_cast<int>(nx) ||
    field_->Ny != static_cast<int>(ny) || std::fabs(res - res_) > 1e-9;
  if (rebuild_needed) {
    // anchor: absolute integer origin, plus the (normally zero) offset of the origin from the cell grid
    ocx_ = static_cast<int>(std::lround(master.getOriginX() / res));
    ocy_ = static_cast<int>(std::lround(master.getOriginY() / res));
    off_x_ = master.getOriginX() - ocx_ * res;
    off_y_ = master.getOriginY() - ocy_ * res;
  } else {
    // Nav2 moves the origin by whole cells: advance our integer origin by the rounded metric delta
    ocx_ += static_cast<int>(std::lround((master.getOriginX() - last_origin_x_) / res));
    ocy_ += static_cast<int>(std::lround((master.getOriginY() - last_origin_y_) / res));
  }
  last_origin_x_ = master.getOriginX(); last_origin_y_ = master.getOriginY();
  const int oxc = ocx_, oyc = ocy_;
  const unsigned char * m = master.getCharMap();
  const int lethal = lethal_;
  const bool unk = unknown_is_obstacle_;
  auto occAt = [&](int x, int y) -> uint8_t {
      const unsigned char v = m[static_cast<size_t>(y - oyc) * nx + static_cast<size_t>(x - oxc)];
      if (v == nav2_costmap_2d::NO_INFORMATION) {return unk ? 1 : 0;}
      return v >= lethal ? 1 : 0;
    };

  Timing t;
  if (rebuild_needed) {
    rebuild(oxc, oyc, nx, ny, res, t);
    auto t0 = Clock::now();
    field_->reset(oxc, oyc, occAt);
    t.rebuild_us += usSince(t0);
    t.rebuilt = true;
    need_rebuild_ = false;
  } else {
    // (a) did the window move?
    if (oxc != field_->ox || oyc != field_->oy) {
      auto t0 = Clock::now();
      t.dx = oxc - field_->ox; t.dy = oyc - field_->oy;
      field_->shift(oxc, oyc, occAt);
      t.entering = field_->last_entering; t.orphans = field_->last_orphans;
      t.rebuilt = field_->last_full_rebuild;
      t.shift_us = usSince(t0);
    }
    // (b) which kept cells flipped?
    auto t1 = Clock::now();
    t.changed = field_->syncOccupancy(occAt);
    t.diff_us = usSince(t1);
    // (c) waves
    auto t2 = Clock::now();
    field_->update();
    t.waves_us = usSince(t2);
  }

  long long sc = -1;
  if (self_check_every_ > 0 && (updates_ % self_check_every_) == 0) {
    sc = field_->selfCheck();
    if (sc != 0) {RCLCPP_ERROR(logger_, "EsdfLayer self-check: %lld violations", sc);}
  }
  ++updates_;
  const double total = usSince(t_all);
  logStats(t, field_->pops, sc, total);
  if (publish_field_) {publishField(master, oxc, oyc);}
  current_ = true;
}

void EsdfLayer::logStats(const Timing & t, long long pops, long long self_check, double total_us)
{
  sum_total_us_ += total_us; max_total_us_ = std::max(max_total_us_, total_us);
  if (stats_.is_open()) {
    const double now = clock_ ? clock_->now().seconds() : 0.0;
    stats_ << now << ',' << field_->ox << ',' << field_->oy << ',' << t.dx << ',' << t.dy << ','
           << t.entering << ',' << t.orphans << ',' << t.changed << ',' << (t.rebuilt ? 1 : 0) << ','
           << pops << ',' << t.shift_us << ',' << t.diff_us << ',' << t.waves_us << ','
           << t.rebuild_us << ',' << total_us << ',' << self_check << ',' << updates_ << '\n';
    if ((updates_ % 20) == 0) {stats_.flush();}
  }
  if ((updates_ % 100) == 0) {
    RCLCPP_INFO(
      logger_, "EsdfLayer: %lld updates, mean %.1f us, max %.1f us (window %dx%d)",
      updates_, sum_total_us_ / static_cast<double>(updates_), max_total_us_, field_->Nx, field_->Ny);
  }
}

void EsdfLayer::publishField(const nav2_costmap_2d::Costmap2D & master, int oxc, int oyc)
{
  if (!pub_ || !pub_->is_activated() || pub_->get_subscription_count() == 0) {return;}
  const rclcpp::Time now = clock_->now();
  if (publish_period_ > 0.0 && (now - last_publish_).seconds() < publish_period_) {return;}
  last_publish_ = now;

  const int nx = field_->Nx, ny = field_->Ny;
  auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
  msg->header.stamp = now;
  msg->header.frame_id = layered_costmap_->getGlobalFrameID();
  msg->info.resolution = static_cast<float>(res_);
  msg->info.width = nx; msg->info.height = ny;
  msg->info.origin.position.x = master.getOriginX();
  msg->info.origin.position.y = master.getOriginY();
  msg->info.origin.orientation.w = 1.0;
  msg->data.assign(static_cast<size_t>(nx) * ny, -1);          // -1: no obstacle within dmax
  const double dmax_cells = std::max(1.0, dmax_m_ / res_);
  for (int y = 0; y < ny; ++y) {
    for (int x = 0; x < nx; ++x) {
      const int d2 = field_->d2At(oxc + x, oyc + y);
      if (d2 >= RingEsdf::INF2) {continue;}
      const double v = 100.0 * (1.0 - std::sqrt(static_cast<double>(d2)) / dmax_cells);  // 100 = obstacle
      msg->data[static_cast<size_t>(y) * nx + x] = static_cast<int8_t>(std::clamp(v, 0.0, 100.0));
    }
  }
  pub_->publish(std::move(msg));
}

bool EsdfLayer::snapshotD2(std::vector<int> & d2, int & ox, int & oy, int & nx, int & ny) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  if (!field_) {return false;}
  ox = field_->ox; oy = field_->oy; nx = field_->Nx; ny = field_->Ny;
  d2.resize(static_cast<size_t>(nx) * ny);
  for (int y = 0; y < ny; ++y) {
    for (int x = 0; x < nx; ++x) {
      const int v = field_->d2At(ox + x, oy + y);
      d2[static_cast<size_t>(y) * nx + x] = v >= RingEsdf::INF2 ? -1 : v;
    }
  }
  return true;
}

double EsdfLayer::distanceAt(double wx, double wy) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  if (!field_) {return std::numeric_limits<double>::quiet_NaN();}
  const int cx = static_cast<int>(std::floor((wx - off_x_) / res_)), cy = static_cast<int>(std::floor((wy - off_y_) / res_));
  if (!field_->inWin(cx, cy)) {return std::numeric_limits<double>::quiet_NaN();}
  const int d2 = field_->d2At(cx, cy);
  return d2 >= RingEsdf::INF2 ? std::numeric_limits<double>::infinity() : std::sqrt(static_cast<double>(d2)) * res_;
}

bool EsdfLayer::nearestObstacle(double wx, double wy, double & obstacle_x, double & obstacle_y) const
{
  std::shared_lock<std::shared_mutex> lock(mutex_);
  if (!field_) {return false;}
  int ox = 0, oy = 0;
  if (!field_->closest(static_cast<int>(std::floor((wx - off_x_) / res_)), static_cast<int>(std::floor((wy - off_y_) / res_)), ox, oy)) {
    return false;
  }
  obstacle_x = (ox + 0.5) * res_ + off_x_; obstacle_y = (oy + 0.5) * res_ + off_y_;
  return true;
}

}  // namespace nav2_esdf
