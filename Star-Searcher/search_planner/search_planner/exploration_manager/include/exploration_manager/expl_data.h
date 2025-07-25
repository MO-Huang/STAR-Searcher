#ifndef _EXPL_DATA_H_
#define _EXPL_DATA_H_

#include "active_perception/frontier_finder.h"
#include "quadrotor_msgs/PositionCommand.h"
#include <Eigen/Eigen>
#include <bspline/Bspline.h>
#include <string>
#include <vector>
using Eigen::Vector3d;
using std::pair;
using std::string;
using std::vector;
namespace fast_planner {
struct checkPoint;
struct FSMData {
  // FSM data
  bool trigger_, have_odom_, static_state_;
  vector<string> state_str_;

  Eigen::Vector3d odom_pos_, odom_vel_; // odometry state
  Eigen::Quaterniond odom_orient_;
  double odom_yaw_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_; // start state
  vector<Eigen::Vector3d> start_poss;
  bspline::Bspline newest_traj_;
  quadrotor_msgs::PositionCommand cmd;
};

struct FSMParam {
  double replan_thresh1_;
  double replan_thresh2_;
  double replan_thresh3_;
  double replan_time_; // second
  bool show_tag;
};

struct ExplorationData {
  ros::Time start_time_, finish_time_;
  vector<vector<Vector3d>> frontiers_;
  vector<vector<Vector3d>> dead_frontiers_;
  vector<pair<Vector3d, Vector3d>> frontier_boxes_;
  vector<Vector3d> points_;
  vector<Vector3d> averages_;
  vector<Vector3d> views_;
  vector<double> yaws_;
  vector<Vector3d> global_tour_;
  vector<int> global_tour_idx_;
  vector<Vector3d> inertia_tour_;
  vector<Vector3d> frontier_tour_;
  Vector3d blocked_seg_end_;

  vector<int> refined_ids_;
  vector<vector<Vector3d>> n_points_;
  vector<Vector3d> unrefined_points_;
  vector<Vector3d> refined_points_;
  vector<Vector3d> refined_views_; // points + dir(yaw)
  vector<Vector3d> refined_views1_, refined_views2_;
  vector<Vector3d> refined_tour_;

  Vector3d next_goal_;
  vector<Vector3d> path_next_goal_, kino_path_;
  Vector3d next_pos_;
  double next_yaw_;

  // viewpoint planning
  // vector<Vector4d> views_;
  vector<Vector3d> views_vis1_, views_vis2_;
  vector<Vector3d> centers_, scales_;

  // Coverage planning
  vector<Vector3d> grid_tour_;
  vector<int> grid_ids_;
  vector<checkPoint> local_tour_;
  vector<Eigen::Vector3d> local_tour_vis_;
  vector<vector<Eigen::Vector3d>> division_clusters;
  int plan_num_;
};

struct ExplorationParam {
  // params
  double feature_max_dist_;
  double inertial_cost_offset_;
  bool refine_local_;
  int refined_num_;
  double refined_radius_;
  int top_view_num_;
  double max_decay_;
  string tsp_dir_; // resource dir of tsp solver
  double relax_time_;
  int init_plan_num_;
  int alc_cp_search_range_;

  bool enable_fixed_hgrid_;
  bool perception_aware_local_;
  // Debug infomation switches
  bool verbose_active_loop_;
};

} // namespace fast_planner

#endif