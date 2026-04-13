// #include <fstream>
#include <active_perception/frontier_finder.h>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <exploration_manager/expl_data.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <lkh_tsp_solver/lkh_interface.h>
#include <plan_env/edt_environment.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/planner_manager.h>
#include <thread>
#include <traj_utils/planning_visualization.h>
#include <unordered_map>
#include <sstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;

namespace fast_planner {
// SECTION interfaces for setup and query

FastExplorationManager::FastExplorationManager() {}

FastExplorationManager::~FastExplorationManager() {
  ViewNode::astar_.reset();
  ViewNode::caster_.reset();
  ViewNode::map_.reset();
  ViewNode::router_.reset();
}

void FastExplorationManager::initialize(ros::NodeHandle &nh) {
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  edt_environment_ = planner_manager_->edt_environment_;
  sdf_map_ = edt_environment_->sdf_map_;
  frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
  // view_finder_.reset(new ViewFinder(edt_environment_, nh));
  router_ = planner_manager_->router_;

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);
  visualization_.reset(new PlanningVisualization(nh));
  voronoi_partition_marker_pub_ =
      nh.advertise<visualization_msgs::Marker>("/planning_vis/voronoi_partition", 1);

  nh.param("exploration/refine_local", ep_->refine_local_, true);
  nh.param("exploration/refined_num", ep_->refined_num_, -1);
  nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
  nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
  nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh.param("exploration/relax_time", ep_->relax_time_, 1.0);
  nh.param("exploration/init_plan_num", ep_->init_plan_num_, 2);
  nh.param("exploration/alc_cp_search_range", ep_->alc_cp_search_range_, 10);
  nh.param("exploration/enable_fixed_hgrid", ep_->enable_fixed_hgrid_, false);
  nh.param("exploration/perception_aware_local", ep_->perception_aware_local_,
           false);
  nh.param("exploration/feature_max_dist", ep_->feature_max_dist_, -1.0);
  nh.param("exploration/inertial_cost_offset", ep_->inertial_cost_offset_,
           -1.0);
  nh.param("exploration/vm", ViewNode::vm_, -1.0);
  nh.param("exploration/am", ViewNode::am_, -1.0);
  nh.param("exploration/yd", ViewNode::yd_, -1.0);
  nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
  nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);
  nh.param("exploration/verbose_active_loop", ep_->verbose_active_loop_, false);
  nh.param("exploration/drone_num", ep_->drone_num_, 1);
  nh.param("exploration/drone_id", ep_->drone_id_, 1);
  nh.param("exploration/voronoi_local_range", ep_->local_range_, 8.0);
  nh.param("exploration/voronoi_connection_cache_resolution", ep_->connection_cache_resolution_, 0.2);
  nh.param("exploration/voronoi_state_timeout", ep_->state_timeout_, 1.0);
  nh.param("exploration/voronoi_debug", ep_->voronoi_debug_, false);
  nh.param("segment_length", ep_->voronoi_segment_length_, 1.0);
  nh.param("exploration/voronoi_cluster_r1_scale", ep_->voronoi_cluster_r1_scale_, 2.5);

  ed_->swarm_state_.resize(ep_->drone_num_);
  for (int i = 0; i < ep_->drone_num_; ++i) {
    ed_->swarm_state_[i].stamp_ = 0.0;
  }

  ViewNode::astar_.reset(new Astar);
  ViewNode::astar_->init(nh, edt_environment_);
  ViewNode::map_ = sdf_map_;
  ViewNode::router_ = router_;

  double resolution_ = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(resolution_, origin);
  voronoi_graph_resolution_ = resolution_;
  voronoi_graph_origin_ = origin;
  voronoi_graph_meta_initialized_ = true;

  // planner_manager_->path_finder_->lambda_heu_ = 1.0;
  // // planner_manager_->path_finder_->max_search_time_ = 0.05;
  // planner_manager_->path_finder_->max_search_time_ = 1.0;

  // Initialize TSP par file
  ofstream par_file(ep_->tsp_dir_ + "/single.par");
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
  par_file << "RUNS = 1\n";

  ofstream par_file_hgrid(ep_->tsp_dir_ + "/single_hgrid.par");
  par_file_hgrid << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single_hgrid.tsp\n";
  par_file_hgrid << "GAIN23 = NO\n";
  par_file_hgrid << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_
                 << "/single_hgrid.txt\n";
  par_file_hgrid << "RUNS = 1\n";

  ofstream par_file_frontier(ep_->tsp_dir_ + "/single_frontier.par");
  par_file_frontier << "PROBLEM_FILE = " << ep_->tsp_dir_
                    << "/single_frontier.tsp\n";
  par_file_frontier << "GAIN23 = NO\n";
  par_file_frontier << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_
                    << "/single_frontier.txt\n";
  par_file_frontier << "RUNS = 1\n";

  ofstream par_file_cluster(ep_->tsp_dir_ + "/cluster.par");
  par_file_cluster << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/cluster.tsp\n";
  par_file_cluster << "GAIN23 = NO\n";
  par_file_cluster << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/cluster.txt\n";
  par_file_cluster << "RUNS = 1\n";
}

int FastExplorationManager::planExploreMotionCluster(const Vector3d &pos,
                                                     const Vector3d &vel,
                                                     const Vector3d &acc,
                                                     const Vector3d &yaw) {
  ros::Time t1 = ros::Time::now();
  auto t2 = t1;
  ed_->views_.clear();
  std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
            << ", acc: " << acc.transpose() << std::endl;

  // Search frontiers and group them into clusters
  // Find viewpoints (x,y,z,yaw) for all frontier clusters and get visible ones'
  // info
  frontier_finder_->searchFrontiers(pos);

  frontier_finder_->computeFrontiersToVisit(pos);
  // frontier_finder_->updateFrontierCostMatrix();
  bool neighbor;
  frontier_finder_->clusterFrontiers(pos, neighbor); 
  //若此处代码执行后得到的neighbor为true，说明无人机正在某个边界聚类中。这个neighbor影响findNextCluster，使得优先搜索无人机所在聚类

  frontier_finder_->getFrontiers(ed_->frontiers_);
  // frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
  // frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);
  frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_,
                                         ed_->averages_);
  double frt_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("[planExploreMotionCluster] frt_time:%lf", frt_time);

  if (ed_->frontiers_.empty()) {
    ROS_WARN("No coverable frontier.");
    return NO_FRONTIER;
  }
  voronoiPartition(pos, vel);
  vector<vector<Eigen::Vector3d>> division_clusters;
  frontier_finder_->getFrontierDivision(division_clusters);
  if (division_clusters.empty()) {
    ROS_WARN("No assigned frontier cluster after voronoi partition.");
    return NO_FRONTIER;
  }
  Eigen::Vector3d next_cluster_pos;
  while(division_clusters.size() > 0){
    if (division_clusters.size() > 1) {
      findNextCluster(pos, vel, yaw, ed_->local_tour_, next_cluster_pos,
                      neighbor);
    } else {
      vector<int> indices;
      indices.push_back(0);
      frontier_finder_->getClusterTour(indices, ed_->global_tour_);
      frontier_finder_->getClusterTourIdx(indices, ed_->global_tour_idx_);
      frontier_finder_->getCheckTour(0, ed_->local_tour_);
      next_cluster_pos = ed_->global_tour_[0];
    }

    // for visualize
    ed_->grid_tour_.clear();
    ed_->grid_tour_.push_back(pos);
    ed_->inertia_tour_.insert(ed_->inertia_tour_.begin(), pos);
    for (auto p : ed_->global_tour_)
      ed_->grid_tour_.push_back(p);

    // Do global and local tour planning and retrieve the next viewpoint
    Vector3d next_pos;
    vector<double> next_yaw;
    if (ed_->local_tour_.size() > 1) {
      vector<int> indices;
      findLocalTour(pos, vel, yaw, next_cluster_pos, indices);
      next_pos = ed_->local_tour_[indices[0]].pos_;
      next_yaw = ed_->local_tour_[indices[0]].yaws_;
      ed_->local_tour_vis_.clear();
      ed_->local_tour_vis_.push_back(pos);
      for (int i = 0; i < indices.size() - 1; i++) {
        ed_->local_tour_vis_.push_back(ed_->local_tour_[indices[i]].pos_);
      }
      ed_->local_tour_vis_.push_back(next_cluster_pos);
    } else if (ed_->local_tour_.size() == 1) {
      next_pos = ed_->local_tour_[0].pos_;
      next_yaw = ed_->local_tour_[0].yaws_;
      ed_->local_tour_vis_.clear();
      ed_->local_tour_vis_.push_back(pos);
      ed_->local_tour_vis_.push_back(ed_->local_tour_[0].pos_);

    } else
      ROS_ERROR("Empty destination.");

    std::cout << "Next view: " << next_pos.transpose() << std::endl;

    // Plan trajectory (position and yaw) to the next viewpoint
    t1 = ros::Time::now();

    auto minElementIterator = std::min_element(next_yaw.begin(), next_yaw.end());
    double min_yaw = *minElementIterator;
    auto maxElementIterator = std::max_element(next_yaw.begin(), next_yaw.end());
    double max_yaw = *maxElementIterator;

    // Generate trajectory of x,y,z
    bool go_wait_trav = false;
    planner_manager_->path_finder_->reset();
    planner_manager_->path_finder_->setMaxSearchTime(0.001);
    if (planner_manager_->path_finder_->search(pos, next_pos) !=
        Astar::REACH_END) {
      planner_manager_->path_finder_->setMaxSearchTime(0.008);
      if (planner_manager_->path_finder_->search(pos, next_pos) !=
          Astar::REACH_END) {
        // ROS_ERROR("No path to next viewpoint");
        // return FAIL;
        ROS_ERROR("No path to next viewpoint (%f, %f, %f).", next_pos[0], next_pos[1], next_pos[2]);
        frontier_finder_->removeUnreachableCluster(ed_->global_tour_idx_[0]);
        frontier_finder_->getFrontierDivision(division_clusters);
        if(division_clusters.size() > 0)
            continue;
        else {
          // bool found_blocked = false;
          printf("\033[33mTry to find blocked seg.\033[0m\n");
          vector<Eigen::Vector3d> unreachable_centers;
          frontier_finder_->getUnreachableClusterCenters(unreachable_centers);
          if (router_->search(pos, unreachable_centers[0]) == multi_robot_router::Router_Node::REACH_END){
            vector<Eigen::Vector3d> path = router_->getPath(); //得到的路径z轴值是从起点到目标点均匀变化的，有可能会不符合避障要求
            vector<Eigen::Vector3d> blocked_seg = router_->getBlockedPathSeg();
            if(blocked_seg.size() > 0) {
              // printf("\033[33mThe size of found blocked seg is %lu.\033[0m\n", blocked_seg.size());
              printf("\033[33mfound blocked seg: [%f, %f, %f] to [%f, %f, %f].\033[0m\n", 
                      blocked_seg[0][0], blocked_seg[0][1], blocked_seg[0][2], blocked_seg[1][0], blocked_seg[1][1], blocked_seg[1][2]);
              // return FAIL;
              bool found = false;
              // double n = 1.5;
              // next_pos = (n + 1) * blocked_seg[0] - n * blocked_seg[1];
              next_pos = 2 * blocked_seg[0] -  blocked_seg[1];
              next_pos[2] = pos[2];
              while(!found){
                bool safe = true;
                Vector3i idx;
                Vector3i unsafe_idx;
                planner_manager_->caster_->input(next_pos, blocked_seg[0]);
                while (planner_manager_->caster_->nextId(idx)) {
                  if (sdf_map_->getInflateOccupancy(idx) == 1) 
                  {
                    safe = false;
                    unsafe_idx = idx;
                    break;
                  }
                }
                if(!safe){
                  Eigen::Vector3d unsafe_pos;
                  sdf_map_->indexToPos(unsafe_idx, unsafe_pos);
                  next_pos[0] = (unsafe_pos[0] + blocked_seg[0][0]) / 2;
                  next_pos[1] = (unsafe_pos[1] + blocked_seg[0][1]) / 2;
                  continue;
                }
                Eigen::Vector3i next_idx;
                sdf_map_->posToIndex(next_pos, next_idx);
                if (safe && sdf_map_->getInflateOccupancy(next_idx) == 0) {
                  found  = true;
                  break;
                } else if(sdf_map_->getInflateOccupancy(next_idx) != 0) {
                  double n = (next_pos[0] - blocked_seg[0][0]) / (blocked_seg[0][0] - blocked_seg[1][0]);
                  n = std::max(0.0, n - 0.1);
                  next_pos = (n + 1) * blocked_seg[0] - n * blocked_seg[1];
                  if (n == 0.0) {
                    found = true;
                    break;
                  }
                }
              }
              printf("\033[33mcurrent pose:[%f, %f, %f], waiting point: [%f, %f, %f]\033[0m\n", pos[0], pos[1], pos[2], next_pos[0], next_pos[1], next_pos[2]);
              ed_->blocked_seg_end_ = blocked_seg[1];
              Eigen::Vector3d dir = (blocked_seg[1] - blocked_seg[0]).normalized();
              next_yaw = {atan2(dir[1], dir[0])};
              frontier_finder_->wrapYaw(next_yaw[0]);
              min_yaw = max_yaw = next_yaw[0];
              planner_manager_->path_finder_->reset();
              planner_manager_->path_finder_->setMaxSearchTime(0.001);
              if (planner_manager_->path_finder_->search(pos, next_pos) != Astar::REACH_END) {
                planner_manager_->path_finder_->setMaxSearchTime(0.008);
                if (planner_manager_->path_finder_->search(pos, next_pos) != Astar::REACH_END) {
                  printf("\033[31m No path to the blocked path seg.\033[0m\n");
                  return FAIL;
                }
              }
              printf("\033[32m Found path to the blocked path seg.\033[0m\n");
              go_wait_trav = true;
            }
            else {
              printf("\033[31m No active frontier clusters and blocked seg not found.\033[0m\n");
              return FAIL;
            }
          }
          else {
            printf("\033[31m router_->search(pos, unreachable_centers[0]) failed.\033[0m\n");
            return FAIL;
          }
        }
      }
    }

    ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
    shortenPath(ed_->path_next_goal_);

    // Compute time lower bound of yaw and use in trajectory generation
    double diff = fabs(min_yaw - yaw[0]);
    const double radius_far = 5.0;
    const double radius_close = 1.5;
    const double len = Astar::pathLength(ed_->path_next_goal_);
    double yaw_time_lb = min(diff, 2 * M_PI - diff) / ViewNode::yd_;
    double pos_time_lb = len / ViewNode::vm_;
    double min_time_lb = 0.2 * M_PI / ViewNode::yd_;
    double time_lb =
        max(yaw_time_lb,
            pos_time_lb); // max(max(yaw_time_lb, pos_time_lb), min_time_lb);

    std::cout << "Size of Path_Next_Goal:" << ed_->path_next_goal_.size() << ","
              << " Path length:" << len << std::endl;

    if (len < radius_far) {
      // Next viewpoint is very close, no need to search kinodynamic path, just
      // use waypoints-based optimization
      planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb);
      ed_->next_goal_ = next_pos;
    } else {
      // Next viewpoint is far away, select intermediate goal on geometric path
      // (this also deal with dead end)
      std::cout << "Far goal." << std::endl;
      double len2 = 0.0;
      vector<Eigen::Vector3d> truncated_path = {ed_->path_next_goal_.front()};
      for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i) {
        auto cur_pt = ed_->path_next_goal_[i];
        len2 += (cur_pt - truncated_path.back()).norm();
        truncated_path.push_back(cur_pt);
      }
      ed_->next_goal_ = truncated_path.back();
      planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
      // if (!planner_manager_->kinodynamicReplan(
      //         pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
      //   return FAIL;
      // ed_->kino_path_ = planner_manager_->kino_path_finder_->getKinoTraj(0.02);
    }

    if (planner_manager_->local_data_.position_traj_.getTimeSum() <
        time_lb - 0.1) {
      ROS_ERROR("Lower bound not satified!");
      ROS_ERROR("Yaw_tlb:%f, Pos_tlb:%f, Traj_time:%f", yaw_time_lb, pos_time_lb,
                planner_manager_->local_data_.position_traj_.getTimeSum());
    }

    planner_manager_->planYawExplore(yaw, min_yaw, false, ep_->relax_time_);
    ed_->next_yaw_ = min_yaw;

    double traj_plan_time = (ros::Time::now() - t1).toSec();
    t1 = ros::Time::now();

    double yaw_time = (ros::Time::now() - t1).toSec();
    ROS_WARN("Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);
    double total = (ros::Time::now() - t2).toSec();
    ROS_WARN("Total time: %lf", total);
    ROS_ERROR_COND(total > 0.1, "Total time too long!!!");
    if (next_yaw.size() > 1) {
      planner_manager_->local_data_.spiral_max_yaw_ = max_yaw;
      planner_manager_->local_data_.spiral_min_yaw_ = min_yaw;
      planner_manager_->local_data_.need_spiral_ = true;
    } else {
      planner_manager_->local_data_.need_spiral_ = false;
    }
    if(go_wait_trav)
      planner_manager_->local_data_.go_wait_trav_ = true;
    else
      planner_manager_->local_data_.go_wait_trav_ = false;
    return SUCCEED;
  }
}

void FastExplorationManager::shortenPath(vector<Vector3d> &path) {
  if (path.empty()) {
    ROS_ERROR("Empty path to shorten");
    return;
  }
  // Shorten the tour, only critical intermediate points are reserved.
  const double dist_thresh = 3.0;
  vector<Vector3d> short_tour = {path.front()};
  for (int i = 1; i < path.size() - 1; ++i) {
    if ((path[i] - short_tour.back()).norm() > dist_thresh)
      short_tour.push_back(path[i]);
    else {
      // Add waypoints to shorten path only to avoid collision
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (ViewNode::caster_->nextId(idx) && ros::ok()) {
        if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
            edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }
  if ((path.back() - short_tour.back()).norm() > 1e-3)
    short_tour.push_back(path.back());

  // Ensure at least three points in the path
  if (short_tour.size() == 1) {
    std::cout << "\033[33mshort_tour.size() == 1\033[0m" << std::endl;
    std::cout << "\033[33m The originial path is " << std::endl;
    for (int i = 0; i < path.size(); i++) {
      std::cout << path[i].transpose() << std::endl; 
    }
    std::cout << "\033[0m" << std::endl;
    if (path.size() == 2)
      path.insert(path.begin() + 1, 0.5 * (path[0] + path[1]));
    return;
  }
  if (short_tour.size() == 2)
    short_tour.insert(short_tour.begin() + 1,
                      0.5 * (short_tour[0] + short_tour[1]));
  path = short_tour;
}

void FastExplorationManager::findNextCluster(const Vector3d &cur_pos,
                                             const Vector3d &cur_vel,
                                             const Vector3d &cur_yaw,
                                             vector<checkPoint> &check_tour,
                                             Eigen::Vector3d &next_cluster_pos,
                                             const bool neighbor) {

  auto t1 = ros::Time::now();

  Eigen::MatrixXd cost_matrix;
  frontier_finder_->getClusterMatrix(cur_pos, cur_vel, cur_yaw, cost_matrix);
  if (neighbor)
    cost_matrix(0, 1) = 0.0;

  vector<Eigen::Vector3d> centers;
  frontier_finder_->getClusterCenter(centers);
  vector<int> inertial_indices, TSP_indices;
  double inertial_cost = std::numeric_limits<double>::max(),
         TSP_cost = std::numeric_limits<double>::max();
  updateInertialTour(cur_pos, ed_->global_tour_, centers, cost_matrix,
                     inertial_indices, inertial_cost);

  TSPConfig cluster_config;
  cluster_config.dimension_ = cost_matrix.rows();
  cluster_config.problem_name_ = "cluster";
  cluster_config.skip_first_ = true;
  cluster_config.skip_last_ = false;
  cluster_config.result_id_offset_ = 2;
  solveTSP(cost_matrix, cluster_config, TSP_indices, TSP_cost);

  vector<int> indices;

  // ROS_WARN("[findNextCluster]Inertial cost : %lf, TSP cost : %lf",
          //  inertial_cost * ViewNode::vm_, TSP_cost * ViewNode::vm_);
  if (inertial_cost * ViewNode::vm_ <
      TSP_cost * ViewNode::vm_ + ep_->inertial_cost_offset_) {
    indices = inertial_indices;
    frontier_finder_->getClusterTour(inertial_indices, ed_->global_tour_);
    frontier_finder_->getClusterTourIdx(inertial_indices, ed_->global_tour_idx_);
    // ROS_WARN("[findNextCluster] Using Inertial tour");
  } else {
    indices = TSP_indices;
    frontier_finder_->getClusterTour(TSP_indices, ed_->global_tour_);
    frontier_finder_->getClusterTourIdx(TSP_indices, ed_->global_tour_idx_);
    // ROS_WARN("[findNextCluster] Using TSP tour");
  }

  next_cluster_pos = centers[indices[1]];
  frontier_finder_->getCheckTour(indices[0], check_tour);

  double cal_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("[findNextCluster] Calculation Time: %f", cal_time);
}

void FastExplorationManager::findLocalTour(const Vector3d &cur_pos,
                                           const Vector3d &cur_vel,
                                           const Vector3d cur_yaw,
                                           const Vector3d &next_cluster_pos,
                                           vector<int> &indices) {
  auto t1 = ros::Time::now();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXd cost_mat;
  frontier_finder_->getCheckTourCostMatrix(cur_pos, cur_vel, cur_yaw,
                                           next_cluster_pos, cost_mat);
  const int dimension = cost_mat.rows();
  // double mat_time = (ros::Time::now() - t1).toSec();
  // t1 = ros::Time::now();
  ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
  string prob_spec =
      "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
  prob_file << prob_spec;
  const int scale = 100;
  // Use Asymmetric TSP
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = cost_mat(i, j) * scale;
      prob_file << int_cost << " ";
    }
    prob_file << "\n";
  }

  prob_file << "EOF";
  prob_file.close();

  // Call LKH TSP solver
  solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/single.txt");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }

  // Read path for ATSP formulation
  while (getline(res_file, res)) {
    // Read indices of frontiers in optimal tour
    int id = stoi(res);
    if (id == 1) // Ignore the current state
      continue;
    if (id == -1)
      break;
    indices.push_back(id - 2); // Idx of solver-2 == Idx of frontier
  }

  res_file.close();
  // double tsp_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("Mat_time: %lf, TSP_time: %lf", mat_time, tsp_time);
  double local_tour_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("[FindLocalTour] local_tour_time: %lf", local_tour_time);
}

void FastExplorationManager::findGlobalTour(const Vector3d &cur_pos,
                                            const Vector3d &cur_vel,
                                            const Vector3d cur_yaw,
                                            vector<int> &indices) {
  auto t1 = ros::Time::now();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXd cost_mat;
  frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
  const int dimension = cost_mat.rows();

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Write params and cost matrix to problem file
  ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
  // Problem specification part, follow the format of TSPLIB

  string prob_spec =
      "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

  // string prob_spec = "NAME : single\nTYPE : TSP\nDIMENSION : " +
  // to_string(dimension) +
  //     "\nEDGE_WEIGHT_TYPE : "
  //     "EXPLICIT\nEDGE_WEIGHT_FORMAT : LOWER_ROW\nEDGE_WEIGHT_SECTION\n";

  prob_file << prob_spec;
  // prob_file << "TYPE : TSP\n";
  // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
  // Problem data part
  const int scale = 100;
  // Use Asymmetric TSP
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = cost_mat(i, j) * scale;
      prob_file << int_cost << " ";
    }
    prob_file << "\n";
  }

  prob_file << "EOF";
  prob_file.close();

  // Call LKH TSP solver
  solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/single.txt");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }

  // Read path for ATSP formulation
  while (getline(res_file, res)) {
    // Read indices of frontiers in optimal tour
    int id = stoi(res);
    if (id == 1) // Ignore the current state
      continue;
    if (id == -1)
      break;
    indices.push_back(id - 2); // Idx of solver-2 == Idx of frontier
  }

  res_file.close();

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->global_tour_);

  double tsp_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);
}

int FastExplorationManager::updateFrontierStruct(const Eigen::Vector3d &pos) {

  auto t1 = ros::Time::now();
  ed_->views_.clear();

  // Search frontiers and group them into clusters
  frontier_finder_->searchFrontiers(pos);

  double frontier_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Find viewpoints (x,y,z,yaw) for all clusters; find the informative ones
  frontier_finder_->computeFrontiersToVisit(pos);

  // Retrieve the updated info
  frontier_finder_->getFrontiers(ed_->frontiers_);
  frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);
  frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);

  frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_,
                                         ed_->averages_);

  ROS_WARN("ed->points size: %ld", ed_->points_.size());
  for (int i = 0; i < ed_->points_.size(); ++i)
    ed_->views_.push_back(ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]),
                                                           sin(ed_->yaws_[i]),
                                                           0));

  if (ed_->frontiers_.empty()) {
    ROS_WARN("[ActiveExplorationManager] No frontier");
    return 0;
  }

  double view_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  frontier_finder_->updateFrontierCostMatrix();

  double mat_time = (ros::Time::now() - t1).toSec();
  double total_time = frontier_time + view_time + mat_time;
  // ROS_INFO("[ActiveExplorationManager] Frontier search t: %.4lf, viewpoint t: "
  //          "%.4lf, cost mat t: "
  //          "%.4lf, frontier update total t: "
  //          "%.4lf",
  //          frontier_time, view_time, mat_time, total_time);
  return ed_->frontiers_.size();
}

void FastExplorationManager::refineLocalTour(
    const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d &cur_yaw,
    const vector<vector<Vector3d>> &n_points,
    const vector<vector<double>> &n_yaws, vector<Vector3d> &refined_pts,
    vector<double> &refined_yaws) {
  double create_time, search_time, parse_time;
  auto t1 = ros::Time::now();

  // Create graph for viewpoints selection
  GraphSearch<ViewNode> g_search;
  vector<ViewNode::Ptr> last_group, cur_group;

  // Add the current state
  ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
  first->vel_ = cur_vel;
  g_search.addNode(first);
  last_group.push_back(first);
  ViewNode::Ptr final_node;

  // Add viewpoints
  std::cout << "Local tour graph: ";
  for (int i = 0; i < n_points.size(); ++i) {
    // Create nodes for viewpoints of one frontier
    for (int j = 0; j < n_points[i].size(); ++j) {
      ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
      g_search.addNode(node);
      // Connect a node to nodes in last group
      for (auto nd : last_group)
        g_search.addEdge(nd->id_, node->id_);
      cur_group.push_back(node);

      // Only keep the first viewpoint of the last local frontier
      if (i == n_points.size() - 1) {
        final_node = node;
        break;
      }
    }
    // Store nodes for this group for connecting edges
    std::cout << cur_group.size() << ", ";
    last_group = cur_group;
    cur_group.clear();
  }
  std::cout << "" << std::endl;
  create_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Search optimal sequence
  vector<ViewNode::Ptr> path;
  g_search.DijkstraSearch(first->id_, final_node->id_, path);

  search_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Return searched sequence
  for (int i = 1; i < path.size(); ++i) {
    refined_pts.push_back(path[i]->pos_);
    refined_yaws.push_back(path[i]->yaw_);
  }

  // Extract optimal local tour (for visualization)
  ed_->refined_tour_.clear();
  ed_->refined_tour_.push_back(cur_pos);
  // ViewNode::astar_->lambda_heu_ = 1.0;
  // ViewNode::astar_->setResolution(0.2);
  for (auto pt : refined_pts) {
    vector<Vector3d> path;
    if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
      ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(),
                                path.end());
    else
      ed_->refined_tour_.push_back(pt);
  }
  // ViewNode::astar_->lambda_heu_ = 10000;

  parse_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time,
  // search_time, parse_time);
}

void FastExplorationManager::updateInertialTour(
    const Vector3d cur_pos, const vector<Vector3d> &last_tour,
    const vector<Vector3d> &cluster_centers,
    const Eigen::MatrixXd &cluster_cost_matrix, vector<int> &indices,
    double &inertia_cost) {
  auto t1 = ros::Time::now();
  inertia_cost = 0.0;
  if (last_tour.empty() || cluster_centers.empty()) {
    inertia_cost = 10000.0;
    return;
  }

  indices.clear();
  for (int i = 0; i < cluster_centers.size(); i++) {
    indices.push_back(i);
  }

  // min cost from every cluster centers in present tour to last tour
  vector<double> classified_min_cost;
  // the id of the centers in this tour corresponding to ones in the last
  vector<int> classified_id;
  vector<int> classified_num(last_tour.size(), 0);
  for (int i = 0; i < cluster_centers.size(); i++) {
    double minDistance = std::numeric_limits<double>::max();
    int id = -1;
    double pos_cost;
    for (int j = 0; j < last_tour.size(); j++) {
      // avoid long distance astar
      if ((cluster_centers[i] - last_tour[j]).norm() < 8.0) {
        vector<Vector3d> path;
        pos_cost = ViewNode::computeCostPos(cluster_centers[i], last_tour[j],
                                            Vector3d::Zero(), path);
      } else {
        pos_cost = 500.0 + (cluster_centers[i] - last_tour[j]).norm();
      }

      if (pos_cost < minDistance) {
        minDistance = pos_cost;
        id = j;
      }
    }
    // if (id == -1) {
    //   id = last_tour.size() - 1;
    //   minDistance = 1000.0;
    // }
    classified_min_cost.push_back(minDistance);
    classified_id.push_back(id);
    classified_num[id]++;
  }

  auto compare = [=](int id1, int id2) {
    if (classified_id[id1] != classified_id[id2])
      return classified_id[id1] < classified_id[id2];
    else
      return classified_min_cost[id1] < classified_min_cost[id2];
  };
  // memory indices for cluster_centers according to the order of classified_id
  // and pos_cost
  sort(indices.begin(), indices.end(), compare);

  // calculate TSP from indices[begin_id] to indices[end_id]
  auto calculateLocalTSP = [&](const int begin_id, const int end_id) -> double {
    // consider cur_pos for the first segment
    if (end_id == 0 && begin_id == 0)
      return cluster_cost_matrix(0, indices[0] + 1);
    // case of two neighbor feature points
    if (end_id - begin_id < 2)
      return cluster_cost_matrix(indices[begin_id] + 1, indices[end_id] + 1);

    Eigen::MatrixXd local_cost_matrix;
    vector<int> local_indices;
    vector<int> indices_copy;
    for (int i = begin_id + 1; i < end_id; i++) {
      indices_copy.push_back(indices[i]);
    }

    int dimen;
    double local_cost;
    dimen = end_id - begin_id + 1;
    local_cost_matrix = Eigen::MatrixXd::Zero(dimen, dimen);
    // cost between different cluster_centers
    for (int i = 0; i < dimen; i++) {
      for (int j = i; j < dimen; j++) {
        local_cost_matrix(i, j) = local_cost_matrix(j, i) = cluster_cost_matrix(
            indices[begin_id + i] + 1, indices[begin_id + j] + 1);
      }
    }

    // set cost to fix start point and end point
    local_cost_matrix.leftCols<1>().setZero();
    for (int i = 1; i < dimen - 1; i++) {
      local_cost_matrix(i, 0) = 65536;
      local_cost_matrix(dimen-1, i) = 65536;
    }

    TSPConfig cluster_config;
    cluster_config.dimension_ = local_cost_matrix.rows();
    cluster_config.problem_name_ = "cluster";
    cluster_config.skip_first_ = true;
    cluster_config.skip_last_ = true;
    cluster_config.result_id_offset_ = 2;
    solveTSP(local_cost_matrix, cluster_config, local_indices, local_cost);

    for (int i = 0; i < local_indices.size(); i++) {
      indices[begin_id + i + 1] = indices_copy[local_indices[i]];
    }
    return local_cost;
  };

  // find the feature centers and calculate the local TSP
  int begin_id = 0, end_id = 0;
  while (end_id < cluster_centers.size()) {
    if (classified_min_cost[indices[end_id]] < ep_->feature_max_dist_) {
      inertia_cost += calculateLocalTSP(begin_id, end_id);
      begin_id = end_id;
      end_id += classified_num[classified_id[indices[end_id]]];
    } else {
      end_id += classified_num[classified_id[indices[end_id]]];
    }
  }
  inertia_cost += calculateLocalTSP(begin_id, indices.size() - 1);

  auto cal_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("[updateInertialTour] Calculation time:%f, inertia_cost:%f", cal_time,
  //          inertia_cost);
}

void FastExplorationManager::solveTSP(const Eigen::MatrixXd &cost_matrix,
                                      const TSPConfig &config,
                                      vector<int> &result_indices,
                                      double &total_cost) {
  // Write params and cost matrix to problem file
  ofstream prob_file(ep_->tsp_dir_ + "/" + config.problem_name_ + ".tsp");

  // Problem specification part, follow the format of TSPLIB
  string prob_spec =
      "NAME : single_frontier\nTYPE : ATSP\nDIMENSION : " +
      to_string(config.dimension_) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
  prob_file << prob_spec;

  // Use Asymmetric TSP
  const int scale = 100;
  for (int i = 0; i < config.dimension_; ++i) {
    for (int j = 0; j < config.dimension_; ++j) {
      int int_cost = cost_matrix(i, j) * scale;
      prob_file << int_cost << " ";
    }
    prob_file << "\n";
  }

  prob_file << "EOF";
  prob_file.close();

  // Call LKH TSP solver
  solveTSPLKH((ep_->tsp_dir_ + "/" + config.problem_name_ + ".par").c_str());

  // Read result indices from the tour section of result file
  ifstream fin(ep_->tsp_dir_ + "/" + config.problem_name_ + ".txt");
  string res;
  // Go to tour section
  while (getline(fin, res)) {
    // Read total cost
    if (res.find("COMMENT : Length") != std::string::npos) {
      int cost_res = stoi(res.substr(19));
      total_cost = (double)cost_res / 100.0;
      // ROS_INFO("[ActiveExplorationManager] TSP problem name: %s, total
      // cost:
      // %.2f",
      //          config.problem_name_.c_str(), cost);
      std::cout << "[ActiveExplorationManager] TSP problem name: "
                << config.problem_name_ << ", total cost: " << total_cost;
    }
    if (res.compare("TOUR_SECTION") == 0)
      break;
  }
  // Read indices
  while (getline(fin, res)) {
    int id = stoi(res);

    // Ignore the first state (current state)
    if (id == 1 && config.skip_first_) {
      continue;
    }

    // Ignore the last state (next grid or virtual depot)
    if (id == config.dimension_ && config.skip_last_) {
      break;
    }

    // EOF
    if (id == -1)
      break;

    result_indices.push_back(id - config.result_id_offset_);
  }
  fin.close();
}

// int FastExplorationManager::planTrajToViewInfo(
//     const Vector3d &pos, const Vector3d &vel, const Vector3d &acc,
//     const Vector3d &yaw, const Vector3d &next_pos, const double &next_yaw)
//     {
//   // Plan perception-aware trajectory (position and yaw) to the next
//   viewpoint TicToc tic_pos;
//   // [Todo] Compute time lower bound of yaw and use in trajectory
//   generation
//   // [Todo] May need to handle ed_->path_next_goal_, ed_->next_goal_
//   bool truncated = false;
//   int local_result =
//       planner_manager_->planLocalMotion(next_pos, pos, vel, acc,
//       truncated);
//   if (local_result == LOCAL_FAIL)
//     return FAIL;

//   ROS_WARN("[Local Planner] Plan path time: %fs", tic_pos.toc());

//   TicToc tic_yaw;
//   // planner_manager_->planYawExplore(yaw, next_yaw, true,
//   ep_->relax_time_); bool specify_end_yaw = (truncated) ? false : true;
//   planner_manager_->planYawInfo(yaw, next_yaw, specify_end_yaw,
//                                 ep_->relax_time_);

//   ROS_WARN("[Local Planner] Plan yaw time: %fs", tic_yaw.toc());

//   return SUCCEED;
// }

void FastExplorationManager::clearExplorationData() {
  ed_->frontier_tour_.clear();
  ed_->n_points_.clear();
  ed_->refined_ids_.clear();
  ed_->unrefined_points_.clear();
  ed_->refined_points_.clear();
  ed_->refined_views_.clear();
  ed_->refined_views1_.clear();
  ed_->refined_views2_.clear();
  ed_->refined_tour_.clear();
}

double FastExplorationManager::hausdorffDistance(
    const std::vector<Eigen::Vector3d> &set1,
    const std::vector<Eigen::Vector3d> &set2, vector<int> &indices) {
  indices.clear();
  double maxDistance = 0.0;
  int min_idx, idx = 0;

  // Calculate the maximum distance from set1 to set2
  for (int i = 0; i < set1.size(); i++) {
    double minDistance = std::numeric_limits<double>::max();

    for (int j = 0; j < set2.size(); j++) {
      double distance = (set1[i] - set2[j]).norm();
      if (distance < minDistance) {
        minDistance = distance;
        min_idx = j;
      }
    }
    if (minDistance > maxDistance) {
      maxDistance = minDistance;
    }
    if (indices.size() < set2.size()) {
      indices.push_back(min_idx);
    }
  }

  // Calculate the maximum distance from set2 to set1
  for (const auto &p2 : set2) {
    double minDistance = std::numeric_limits<double>::max();
    for (const auto &p1 : set1) {
      double distance = (p2 - p1).norm();
      if (distance < minDistance) {
        minDistance = distance;
      }
    }
    if (minDistance > maxDistance) {
      maxDistance = minDistance;
    }
  }
  return maxDistance;
}

void FastExplorationManager::voronoiPartition(const Vector3d &cur_pos,
                                              const Vector3d &cur_vel) {
  // Voronoi-style task partition:
  // 1) Build local topo graph from Voronoi segments.
  // 2) Bind clusters to reachable topo and select local clusters.
  // 3) Seed overlap nodes from local drones and run multi-source Dijkstra.
  // 4) Keep clusters owned by this drone (fallback keeps all selected).
  vector<Vector3d> centers;
  frontier_finder_->getClusterCenter(centers);
  const int cluster_num = static_cast<int>(centers.size());
  if (cluster_num == 0) return;

  // Voronoi graph segments (grid coords) from the topo structure.
  std::vector<tuw_graph::Segment> vg_segments;
  // Pull topology segments (no grid data needed here).
  if (!sdf_map_->getVoronoiGraph(vg_segments) || vg_segments.empty()) {
    return;
  }
  // Cache resolution/origin once for endpoint -> world conversion.
  if (!voronoi_graph_meta_initialized_) {
    Vector3d region_size;
    int z_layer;
    sdf_map_->getRegion(voronoi_graph_origin_, region_size);
    sdf_map_->getZLayer(z_layer);
    voronoi_graph_resolution_ = sdf_map_->getResolution();
    voronoi_graph_origin_[2] = voronoi_graph_origin_[2] + z_layer * voronoi_graph_resolution_;
    voronoi_graph_meta_initialized_ = true;
  }

  // Clamp cache quantization to avoid tiny bins.
  ep_->connection_cache_resolution_ = std::max(1e-3, ep_->connection_cache_resolution_);

  const int self_idx = ep_->drone_id_ - 1;
  const double now = ros::Time::now().toSec();

  // Minimal drone state for partitioning seeds.
  struct DroneSeed {
    int drone_idx_;
    Vector3d pos_;
    Vector3d vel_;
  };
  vector<DroneSeed> valid_drones;
  valid_drones.reserve(ep_->drone_num_);
  for (int drone_idx = 0; drone_idx < ep_->drone_num_; ++drone_idx) {
    // Keep self and recent teammates only.
    if (drone_idx == self_idx) {
      valid_drones.push_back({drone_idx, cur_pos, cur_vel});
    } else if (now - ed_->swarm_state_[drone_idx].stamp_ < ep_->state_timeout_) {
      valid_drones.push_back(
          {drone_idx, ed_->swarm_state_[drone_idx].pos_, ed_->swarm_state_[drone_idx].vel_});
    }
  }
  if (valid_drones.empty()) return;
  if (ep_->voronoi_debug_) {
    std::ostringstream ss;
    ss << "[voronoiPartition] self_id=" << ep_->drone_id_
       << ", total_clusters=" << cluster_num
       << ", vg_segments=" << vg_segments.size()
       << ", valid_drones=" << valid_drones.size();
    ROS_WARN_STREAM(ss.str());
  }

  // 1) Build topo graph nodes from segment endpoints.
  //    Topo-topo edge weights use Voronoi segment length.
  struct TopoNode {
    Vector3d pos_;
    vector<pair<int, double>> neighbors_;
  };
  vector<TopoNode> topo_nodes;

  auto endpointToWorld = [&](const Eigen::Vector2d &pt, const double z) -> Vector3d {
    // Segment endpoints are in grid coords; lift to world with current Z.
    Vector3d p = Vector3d::Zero();
    p[0] = pt[0] * voronoi_graph_resolution_ + voronoi_graph_origin_[0];
    p[1] = pt[1] * voronoi_graph_resolution_ + voronoi_graph_origin_[1];
    p[2] = z;
    return p;
  };

  // Query real path cost and cache by quantized endpoint pairs.
  std::unordered_map<std::string, double> path_cost_cache;
  auto makeQuantized = [&](const Vector3d &p) -> Vector3i {
    Vector3i q;
    q[0] = static_cast<int>(std::round(p[0] / ep_->connection_cache_resolution_));
    q[1] = static_cast<int>(std::round(p[1] / ep_->connection_cache_resolution_));
    q[2] = static_cast<int>(std::round(p[2] / ep_->connection_cache_resolution_));
    return q;
  };
  auto makeKey = [&](const Vector3d &a, const Vector3d &b) -> std::string {
    // Order-invariant key so (a,b) and (b,a) share the same cache entry.
    const Vector3i qa = makeQuantized(a);
    const Vector3i qb = makeQuantized(b);
    const bool swap = (qa[0] > qb[0]) ||
                      (qa[0] == qb[0] && qa[1] > qb[1]) ||
                      (qa[0] == qb[0] && qa[1] == qb[1] && qa[2] > qb[2]);
    const Vector3i &u = swap ? qb : qa;
    const Vector3i &v = swap ? qa : qb;
    return std::to_string(u[0]) + "_" + std::to_string(u[1]) + "_" +
           std::to_string(u[2]) + "|" + std::to_string(v[0]) + "_" +
           std::to_string(v[1]) + "_" + std::to_string(v[2]);
  };
  auto queryPathCost = [&](const Vector3d &a, const Vector3d &b, double &cost) -> bool {
    // Returns true if router can reach; cost is real path length.
    if ((a - b).norm() <= 1e-3) {
      cost = 0.0;
      return true;
    }
    const std::string key = makeKey(a, b);
    auto it = path_cost_cache.find(key);
    if (it != path_cost_cache.end()) {
      cost = it->second;
      return std::isfinite(cost);
    }
    if (router_->search(a, b) == multi_robot_router::Router_Node::REACH_END) {
      auto path = router_->getPath();
      if (path.size() < 2) {
        cost = (a - b).norm();
      } else {
        cost = router_->pathLength(path);
      }
      path_cost_cache[key] = cost;
      return true;
    }
    path_cost_cache[key] = std::numeric_limits<double>::infinity();
    return false;
  };
  auto logQueryFail = [&](const char *tag, const Vector3d &a, const Vector3d &b) {
    ROS_WARN("queryPathCost failed at %s. a=[%.3f %.3f %.3f], b=[%.3f %.3f %.3f]",
             tag,
             a[0], a[1], a[2],
             b[0], b[1], b[2]);
  };

  // Endpoint deduplication (quantized) to avoid duplicate topo nodes.
  std::unordered_map<std::string, int> endpoint_to_node;
  auto endpointKey = [&](const Vector3d &p) -> std::string {
    const Vector3i q = makeQuantized(p);
    return std::to_string(q[0]) + "_" + std::to_string(q[1]) + "_" +
           std::to_string(q[2]);
  };
  auto getOrCreateNode = [&](const Vector3d &p) -> int {
    const std::string key = endpointKey(p);
    auto it = endpoint_to_node.find(key);
    if (it != endpoint_to_node.end()) return it->second;
    topo_nodes.push_back({p, {}});
    const int id = static_cast<int>(topo_nodes.size()) - 1;
    endpoint_to_node[key] = id;
    return id;
  };

  // Build topo adjacency from Voronoi segments.
  for (const auto &seg : vg_segments) {
    const Vector3d s = endpointToWorld(seg.getStart(), cur_pos[2]);
    const Vector3d e = endpointToWorld(seg.getEnd(), cur_pos[2]);
    const int u = getOrCreateNode(s);
    const int v = getOrCreateNode(e);
    // Segment length is in grid units; scale to meters by resolution.
    const double w = std::max(1.0, static_cast<double>(seg.getLength())) *
                     voronoi_graph_resolution_;//这里的getLength()得到的值是什么，直接乘上分辨率是对的吗?
    topo_nodes[u].neighbors_.push_back({v, w});
    topo_nodes[v].neighbors_.push_back({u, w});
  }
  if (topo_nodes.empty()) return;

  // Candidate topo nodes by XY range (filtered by reachability later).
  vector<int> candidate_topo_ids;
  for (int i = 0; i < static_cast<int>(topo_nodes.size()); ++i) {
    if ((topo_nodes[i].pos_.head<2>() - cur_pos.head<2>()).norm() <= ep_->local_range_) {
      candidate_topo_ids.push_back(i);
    }
  }
  if (candidate_topo_ids.empty()) { // Fallback: keep nearest topo to avoid empty candidates.
    double best = std::numeric_limits<double>::max();
    int best_id = -1;
    for (int i = 0; i < static_cast<int>(topo_nodes.size()); ++i) {
      const double d = (topo_nodes[i].pos_.head<2>() - cur_pos.head<2>()).norm();
      if (d < best) {
        best = d;
        best_id = i;
      }
    }
    if (best_id >= 0) candidate_topo_ids.push_back(best_id);
  }

  // Keep only topo nodes that are path-reachable from self.
  vector<int> local_topo_ids;
  for (const int tid : candidate_topo_ids) {
    double c = 0.0;
    if (queryPathCost(cur_pos, topo_nodes[tid].pos_, c)) {
      local_topo_ids.push_back(tid);
    } else {
      logQueryFail("local_topo_ids:cur_to_topo", cur_pos, topo_nodes[tid].pos_);
    }
  }
  if (local_topo_ids.empty()) {
    int best_id = -1;
    double best_cost = std::numeric_limits<double>::max();
    for (int i = 0; i < static_cast<int>(topo_nodes.size()); ++i) {
      double c = 0.0;
      if (!queryPathCost(cur_pos, topo_nodes[i].pos_, c)) {
        logQueryFail("local_topo_ids_fallback:cur_to_topo", cur_pos, topo_nodes[i].pos_);
        continue;
      }
      if (c < best_cost) {
        best_cost = c;
        best_id = i;
      }
    }
    if (best_id >= 0) local_topo_ids.push_back(best_id);
  }
  if (local_topo_ids.empty()) return;
  if (ep_->voronoi_debug_) {
    std::ostringstream ss;
    ss << "[voronoiPartition] local_topo_ids(" << local_topo_ids.size() << "): ";
    for (const int tid : local_topo_ids) ss << tid << " ";
    ROS_WARN_STREAM(ss.str());
  }

  // 2) Bind each cluster to its nearest reachable topo (global topo set).
  //    Select clusters whose bound topo is in local_topo_ids.
  const double Lmax = std::max(1e-3, ep_->voronoi_segment_length_);
  const double R1 = ep_->voronoi_cluster_r1_scale_ * Lmax;

  vector<int> cluster_best_topo(cluster_num, -1);
  vector<double> cluster_best_cost(cluster_num, std::numeric_limits<double>::infinity());

  for (int cid = 0; cid < cluster_num; ++cid) {
    vector<pair<double, int>> topo_dists;
    topo_dists.reserve(topo_nodes.size());
    double max_dist = 0.0;
    for (int tid = 0; tid < static_cast<int>(topo_nodes.size()); ++tid) {
      const double d = (topo_nodes[tid].pos_.head<2>() - centers[cid].head<2>()).norm();
      topo_dists.push_back({d, tid});
      if (d > max_dist) max_dist = d;
    }
    std::sort(topo_dists.begin(), topo_dists.end(),
              [](const pair<double, int> &a, const pair<double, int> &b) {
                return a.first < b.first;
              });

    auto scanTier = [&](const double r, size_t &start_idx) -> bool {
      bool found = false;
      while (start_idx < topo_dists.size() && topo_dists[start_idx].first <= r) {
        const int tid = topo_dists[start_idx].second;
        double c = 0.0;
        if (queryPathCost(topo_nodes[tid].pos_, centers[cid], c)) {
          if (c < cluster_best_cost[cid]) {
            cluster_best_cost[cid] = c;
            cluster_best_topo[cid] = tid;
          }
          found = true;
        } else {
          logQueryFail("cluster_bind:topo_to_center", topo_nodes[tid].pos_, centers[cid]);
        }
        ++start_idx;
      }
      return found;
    };

    size_t idx = 0;
    double r = std::min(R1, max_dist);
    bool found = scanTier(r, idx);
    if (!found && r < max_dist) {
      r = std::min(2.0 * r, max_dist);
      found = scanTier(r, idx);
    }
    if (!found && r < max_dist) {
      r = std::min(2.0 * r, max_dist);
      found = scanTier(r, idx);
    }
    if (!found && r < max_dist) {
      scanTier(max_dist, idx);
    }
  }

  vector<char> is_local_topo(topo_nodes.size(), false);
  for (const int tid : local_topo_ids) is_local_topo[tid] = true;

  vector<int> selected_clusters;
  selected_clusters.reserve(cluster_num);
  for (int cid = 0; cid < cluster_num; ++cid) {
    const int tid = cluster_best_topo[cid];
    if (tid >= 0 && is_local_topo[tid]) selected_clusters.push_back(cid);
  }
  if (selected_clusters.empty()) return;
  if (ep_->voronoi_debug_) {
    std::ostringstream ss;
    ss << "[voronoiPartition] selected_clusters(" << selected_clusters.size() << "): ";
    for (const int cid : selected_clusters) ss << cid << " ";
    ROS_WARN_STREAM(ss.str());
  }

  // 3) Keep drones whose local range overlaps self local range.
  //    Overlap: XY range filter + reachability via queryPathCost.
  struct LocalDroneInfo {
    DroneSeed dr_;
    vector<int> overlap_topo_ids_;
    vector<int> overlap_cluster_ids_;
  };
  vector<LocalDroneInfo> local_drones;
  local_drones.reserve(valid_drones.size());
  for (const auto &dr : valid_drones) {
    LocalDroneInfo info;
    info.dr_ = dr;

    // Range pre-filter for topo nodes.
    vector<int> topo_candidates;
    topo_candidates.reserve(local_topo_ids.size());
    for (const int tid : local_topo_ids) {
      if ((topo_nodes[tid].pos_.head<2>() - dr.pos_.head<2>()).norm() <= ep_->local_range_) {
        topo_candidates.push_back(tid);
      }
    }
    // Reachability check for topo candidates.
    for (const int tid : topo_candidates) {
      double c = 0.0;
      if (queryPathCost(dr.pos_, topo_nodes[tid].pos_, c)) {
        info.overlap_topo_ids_.push_back(tid);
      } else {
        logQueryFail("local_drones:dr_to_topo", dr.pos_, topo_nodes[tid].pos_);
      }
    }

    // Range pre-filter for frontier clusters.
    vector<int> cluster_candidates;
    cluster_candidates.reserve(selected_clusters.size());
    for (const int cid : selected_clusters) {
      if ((centers[cid].head<2>() - dr.pos_.head<2>()).norm() <= ep_->local_range_) {
        cluster_candidates.push_back(cid);
      }
    }
    // Reachability check for cluster candidates.
    for (const int cid : cluster_candidates) {
      double c = 0.0;
      if (queryPathCost(dr.pos_, centers[cid], c)) {
        info.overlap_cluster_ids_.push_back(cid);
      } else {
        logQueryFail("local_drones:dr_to_center", dr.pos_, centers[cid]);
      }
    }

    // Keep drone only if it overlaps with self local range.
    if (dr.drone_idx_ == self_idx ||
        !info.overlap_topo_ids_.empty() ||
        !info.overlap_cluster_ids_.empty()) {
      local_drones.push_back(info);
    }
  }
  if (local_drones.empty()) return;
  if (ep_->voronoi_debug_) {
    std::ostringstream ss;
    ss << "[voronoiPartition] local_drones(" << local_drones.size() << "): ";
    for (const auto &info : local_drones) ss << (info.dr_.drone_idx_ + 1) << " ";
    ROS_WARN_STREAM(ss.str());
  }

  // Build local graph: topo nodes + selected clusters.
  // Node layout: [0..topo_n-1]=topo, [topo_n..topo_n+clu_n-1]=clusters.
  const int topo_n = static_cast<int>(local_topo_ids.size());
  const int clu_n = static_cast<int>(selected_clusters.size());
  const int total_n = topo_n + clu_n;
  vector<vector<pair<int, double>>> adj(total_n);
  std::unordered_map<int, int> topo_local_idx;
  for (int i = 0; i < topo_n; ++i) topo_local_idx[local_topo_ids[i]] = i; // topo_id -> local_idx

  // topo-topo edges (from Voronoi segments).
  for (int i = 0; i < topo_n; ++i) {
    const int tid = local_topo_ids[i];
    for (const auto &nei : topo_nodes[tid].neighbors_) {
      const int nei_tid = nei.first;
      auto it_local = topo_local_idx.find(nei_tid);
      if (it_local == topo_local_idx.end()) continue;
      const int j = it_local->second;
      adj[i].push_back({j, nei.second});
    }
  }

  // topo-cluster edges: connect each cluster only to its bound topo.
  for (int j = 0; j < clu_n; ++j) {
    const int cid = selected_clusters[j];
    const int tid = cluster_best_topo[cid];
    auto it_local = topo_local_idx.find(tid);
    if (it_local == topo_local_idx.end()) continue;
    const int i = it_local->second;
    const double w = cluster_best_cost[cid];
    if (!std::isfinite(w)) continue;
    const int ci = topo_n + j;
    adj[i].push_back({ci, w});
    adj[ci].push_back({i, w});
  }

  // Multi-source Dijkstra over local graph (seeded by drones).
  struct QueueNode {
    double dist_;
    int node_id_;
    int owner_;
    bool operator<(const QueueNode &other) const { return dist_ > other.dist_; }
  };
  std::priority_queue<QueueNode> q;
  vector<double> best_dist(total_n, std::numeric_limits<double>::max());
  vector<int> owner(total_n, -1);

  // Keep best dist; tie-break by smaller drone index for deterministic ownership.
  auto tryPush = [&](const int node_id, const double dist, const int drone_owner) {
    if (dist + 1e-6 < best_dist[node_id] ||
        (std::abs(dist - best_dist[node_id]) <= 1e-6 &&
         (owner[node_id] == -1 || drone_owner < owner[node_id]))) {
      best_dist[node_id] = dist;
      owner[node_id] = drone_owner;
      q.push({dist, node_id, drone_owner});
    }
  };

  // Seed only overlapping topo/cluster nodes from each local drone.
  for (const auto &info : local_drones) {
    for (const int tid : info.overlap_topo_ids_) {
      double d = 0.0;
      if (queryPathCost(info.dr_.pos_, topo_nodes[tid].pos_, d)) {
        auto it_local = topo_local_idx.find(tid);
        if (it_local != topo_local_idx.end()) {
          tryPush(it_local->second, d, info.dr_.drone_idx_);
        }
      } else {
        logQueryFail("seed_dist:dr_to_topo", info.dr_.pos_, topo_nodes[tid].pos_);
      }
    }
    for (const int cid : info.overlap_cluster_ids_) {
      double d = 0.0;
      if (queryPathCost(info.dr_.pos_, centers[cid], d)) {
        int local_cluster_idx = -1;
        for (int j = 0; j < clu_n; ++j) {
          if (selected_clusters[j] == cid) {
            local_cluster_idx = j;
            break;
          }
        }
        if (local_cluster_idx >= 0) {
          tryPush(topo_n + local_cluster_idx, d, info.dr_.drone_idx_);
        }
      } else {
        logQueryFail("seed_dist:dr_to_center", info.dr_.pos_, centers[cid]);
      }
    }
  }

  if (q.empty()) return;

  // Standard Dijkstra relaxation with owner propagation.
  while (!q.empty()) {
    const QueueNode cur = q.top();
    q.pop();
    if (cur.owner_ != owner[cur.node_id_] ||
        std::abs(cur.dist_ - best_dist[cur.node_id_]) > 1e-6) {
      continue;
    }
    for (const auto &e : adj[cur.node_id_]) {
      const double nd = cur.dist_ + e.second;
      if (nd + 1e-6 < best_dist[e.first] ||
          (std::abs(nd - best_dist[e.first]) <= 1e-6 && cur.owner_ < owner[e.first])) {
        best_dist[e.first] = nd;
        owner[e.first] = cur.owner_;
        q.push({nd, e.first, cur.owner_});
      }
    }
  }

  // Map per-cluster owner/dist for logging and visualization.
  vector<int> cluster_owner(cluster_num, -2); // -2: not in local graph
  vector<double> cluster_dist(cluster_num, std::numeric_limits<double>::infinity());
  for (int j = 0; j < clu_n; ++j) {
    const int cid = selected_clusters[j];
    const int nid = topo_n + j;
    cluster_owner[cid] = owner[nid];
    cluster_dist[cid] = best_dist[nid];
  }

  if (voronoi_partition_marker_pub_) {
    // Visualize selected clusters with color by owner (RViz).
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp = ros::Time::now();
    mk.ns = "voronoi_partition";
    mk.id = 0;
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.45;
    mk.scale.y = 0.45;
    mk.scale.z = 0.45;

    auto ownerColor = [&](const int oid) -> std_msgs::ColorRGBA {
      // Fixed palette; unassigned -> gray.
      std_msgs::ColorRGBA c;
      c.a = 1.0;
      if (oid < 0) {
        c.r = 0.6; c.g = 0.6; c.b = 0.6;
        return c;
      }
      const int m = oid % 6;
      if (m == 0) { c.r = 0.95; c.g = 0.20; c.b = 0.20; }
      if (m == 1) { c.r = 0.20; c.g = 0.95; c.b = 0.20; }
      if (m == 2) { c.r = 0.20; c.g = 0.45; c.b = 0.95; }
      if (m == 3) { c.r = 0.95; c.g = 0.95; c.b = 0.20; }
      if (m == 4) { c.r = 0.95; c.g = 0.45; c.b = 0.20; }
      if (m == 5) { c.r = 0.65; c.g = 0.20; c.b = 0.95; }
      return c;
    };

    mk.points.reserve(selected_clusters.size());
    mk.colors.reserve(selected_clusters.size());
    for (const int cid : selected_clusters) {
      geometry_msgs::Point p;
      p.x = centers[cid][0];
      p.y = centers[cid][1];
      p.z = centers[cid][2];
      mk.points.push_back(p);
      mk.colors.push_back(ownerColor(cluster_owner[cid]));
    }
    voronoi_partition_marker_pub_.publish(mk);
  }

  // Keep clusters owned by self; fallback keeps all selected if none.
  vector<int> keep_ids;
  keep_ids.reserve(clu_n);
  for (int j = 0; j < clu_n; ++j) {
    const int node_id = topo_n + j;
    if (owner[node_id] == self_idx) keep_ids.push_back(selected_clusters[j]);
  }
  bool fallback_keep_all_selected = false;
  if (keep_ids.empty()) {
    keep_ids = selected_clusters;
    fallback_keep_all_selected = true;
  }

  if (ep_->voronoi_debug_) {
    ROS_WARN_STREAM("[voronoiPartition] assignment detail:");
    for (int cid = 0; cid < cluster_num; ++cid) {
      std::ostringstream line;
      line << "  cluster[" << cid << "] center=("
           << centers[cid][0] << "," << centers[cid][1] << "," << centers[cid][2] << ")";
      if (cluster_owner[cid] == -2) {
        line << " owner=NA(local graph excluded)";
      } else if (cluster_owner[cid] < 0) {
        line << " owner=UNASSIGNED";
      } else {
        line << " owner_drone_id=" << (cluster_owner[cid] + 1)
             << " dist=" << cluster_dist[cid];
      }
      ROS_WARN_STREAM(line.str());
    }

    std::ostringstream summary;
    summary << "[voronoiPartition] keep_ids(" << keep_ids.size() << "): ";
    for (const int id : keep_ids) summary << id << " ";
    if (fallback_keep_all_selected) {
      summary << " [fallback: no self-owned cluster, keep all selected]";
    }
    ROS_WARN_STREAM(summary.str());
  }
  frontier_finder_->retainClusterByIds(keep_ids);
}

} // namespace fast_planner
