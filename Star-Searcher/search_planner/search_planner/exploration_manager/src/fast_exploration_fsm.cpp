
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Int32.h"
#include <active_perception/perception_utils.h>
#include <exploration_manager/expl_data.h>
#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <visualization_msgs/Marker.h>
using Eigen::Vector4d;

namespace fast_planner {
void FastExplorationFSM::init(ros::NodeHandle &nh) {
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  /*  Fsm param  */
  nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
  nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
  nh.param("fsm/show_tag", fp_->show_tag, false);

  /* Initialize main modules */
  expl_manager_.reset(new FastExplorationManager);
  expl_manager_->initialize(nh);
  this->visualization_ = expl_manager_->visualization_;
  router_ = expl_manager_->router_;

  planner_manager_ = expl_manager_->planner_manager_;
  percep_utils_.reset(new PerceptionUtils(nh));
  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = {"INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ",
                     "EXEC_TRAJ", "FINISH", "SPIRAL", "WAIT_FRONTIER", "WAIT_TRAV"};
  fd_->static_state_ = true;
  fd_->trigger_ = false;
  fd_->cmd.kx = {5.7, 5.7, 6.2};
  fd_->cmd.kv = {3.4, 3.4, 4.0};
  fd_->cmd.header.frame_id = "world";
  fd_->cmd.trajectory_flag =
      quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;

  tag_poses.clear();
  tf_listener_ptr_ = std::unique_ptr<tf2_ros::TransformListener>(new tf2_ros::TransformListener(tf_buffer_));

  /* Ros sub, pub and timer */
  exec_timer_ = nh.createTimer(ros::Duration(0.01),
                               &FastExplorationFSM::FSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05),
                                 &FastExplorationFSM::safetyCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(0.3),
                                   &FastExplorationFSM::frontierCallback, this);
  tag_timer_ = nh.createTimer(ros::Duration(0.5),
                                   &FastExplorationFSM::tagCallback, this);                                

  trigger_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1,
                              &FastExplorationFSM::triggerCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1,
                           &FastExplorationFSM::odometryCallback, this);

  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);
  start_flag_pub = nh.advertise<std_msgs::Int32>("/start_flag", 10);
  spiral_pub_ =
      nh.advertise<quadrotor_msgs::PositionCommand>("/planning/pos_cmd", 50);

  std::thread vis_thread([this]() {
    ros::Rate rate(30);       // 30 Hz 可视化
    while (ros::ok()) {
      this->visualize();
      rate.sleep();
    }
  });
  vis_thread.detach();
}

void FastExplorationFSM::FSMCallback(const ros::TimerEvent &e) {
  std_msgs::Int32 state_flag;
  ROS_INFO_STREAM_THROTTLE(1.0,
                           "[FSM]: state: " << fd_->state_str_[int(state_)]);

  switch (state_) {
  case INIT: {
    state_flag.data = INIT;
    // Wait for odometry ready
    if (!fd_->have_odom_) {
      ROS_WARN_THROTTLE(1.0, "no odom.");
      return;
    }
    // Go to wait trigger when odom is ok
    transitState(WAIT_TRIGGER, "FSM");
    break;
  }

  case WAIT_TRIGGER: {
    state_flag.data = WAIT_TRIGGER;

    // Do nothing but wait for trigger
    ROS_WARN_THROTTLE(1.0, "wait for trigger.");
    break;
  }

  case FINISH: {
    state_flag.data = FINISH;
    ROS_INFO_THROTTLE(1.0, "finish exploration.");
    break;
  }

  case PLAN_TRAJ: {
    state_flag.data = PLAN_TRAJ;
    if (fd_->static_state_) {
      // Plan from static state (hover)
      fd_->start_pt_ = fd_->odom_pos_;
      fd_->start_vel_ = fd_->odom_vel_;
      fd_->start_acc_.setZero();

      fd_->start_yaw_(0) = fd_->odom_yaw_;
      fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
    } else {
      // Replan from non-static state, starting from 'replan_time' seconds later
      LocalTrajData *info = &planner_manager_->local_data_;
      double t_r =
          (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;

      fd_->start_pt_ =
          info->need_spiral_
              ? info->position_traj_.evaluateDeBoorT(info->duration_)
              : info->position_traj_.evaluateDeBoorT(t_r);
      fd_->start_vel_ =
          info->need_spiral_
              ? info->velocity_traj_.evaluateDeBoorT(info->duration_)
              : info->velocity_traj_.evaluateDeBoorT(t_r);
      fd_->start_acc_ =
          info->need_spiral_
              ? info->acceleration_traj_.evaluateDeBoorT(info->duration_)
              : info->acceleration_traj_.evaluateDeBoorT(t_r);
      fd_->start_yaw_(0) = info->need_spiral_
                               ? info->spiral_max_yaw_
                               : info->yaw_traj_.evaluateDeBoorT(t_r)[0];
      fd_->start_yaw_(1) =
          info->need_spiral_
              ? info->yawdot_traj_.evaluateDeBoorT(info->duration_)[0]
              : info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
      fd_->start_yaw_(2) =
          info->need_spiral_
              ? info->yawdotdot_traj_.evaluateDeBoorT(info->duration_)[0]
              : info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
    }

    // Inform traj_server the replanning
    replan_pub_.publish(std_msgs::Empty());
    int res = callExplorationPlanner();
    if (res == SUCCEED) {
      transitState(PUB_TRAJ, "FSM");
      // visualize();
      // thread vis_thread(&FastExplorationFSM::visualize, this);
      // vis_thread.detach();
    } else if (res == NO_FRONTIER) {
      transitState(FINISH, "FSM");
      fd_->static_state_ = true;
    } else if (res == FAIL) {
      // Still in PLAN_TRAJ state, keep replanning
      ROS_WARN("plan fail");
      fd_->static_state_ = true;
    }
    break;
  }

  case PUB_TRAJ: {
    state_flag.data = PUB_TRAJ;
    double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
    if (dt > 0) {
      bspline_pub_.publish(fd_->newest_traj_);
      fd_->static_state_ = false;
      transitState(EXEC_TRAJ, "FSM");
    }
    break;
  }

  case EXEC_TRAJ: {
    state_flag.data = EXEC_TRAJ;
    LocalTrajData *info = &planner_manager_->local_data_;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();

    // Replan if traj is almost fully executed
    double time_to_end = info->duration_ - t_cur;
    // if (info->need_spiral_ == true && time_to_end < fp_->replan_thresh2_) {
    //   ROS_WARN("Replan: prepare to spiral");
    //   transitState(SPIRAL, "FSM");
    // }
    // std::cout << "\033[32mduration_ = " << info->duration_ << ", t_cur = \033[0m" << t_cur << std::endl;

    Eigen::Vector3d pos = fd_->odom_pos_;
    double yaw = fd_->odom_yaw_;
    bool reach_goal = (pos - expl_manager_->ed_->next_goal_).norm() < 0.5 && abs(yaw - expl_manager_->ed_->next_yaw_)< 1.0;
    // std::cout << "\033[32minfo->go_wait_trav_ = " << info->go_wait_trav_ << ", time_to_end = \033[0m" << time_to_end << std::endl;
    if (info->go_wait_trav_ && reach_goal) {
      transitState(WAIT_TRAV, "FSM");
      fd_->static_state_ = true;
      break;
    }
    if (info->go_wait_trav_ && time_to_end < fp_->replan_thresh1_) {
      ROS_WARN("Reach waiting point for traversability.");
      transitState(WAIT_TRAV, "FSM");
      fd_->static_state_ = true;
      break;
    }

    bool need_replan = false;
    if (time_to_end < fp_->replan_thresh1_) {
      ROS_WARN("Replan: traj fully executed");
      need_replan = true;
    }
    // Replan if next frontier to be visited is covered
    if (t_cur > fp_->replan_thresh2_ &&
        expl_manager_->frontier_finder_->isFrontierCovered()) {
      ROS_WARN("Replan: cluster covered");
      need_replan = true;
    }

    // Replan after some time
    if (t_cur > fp_->replan_thresh3_) {
      ROS_WARN("Replan: periodic call");
      need_replan = true;
    }

    // if (need_replan) {
    //   if (expl_manager_->updateFrontierStruct(fd_->odom_pos_) != 0) {

    //     transitState(PLAN_TRAJ, "FSM");
    //   } else {
    //     transitState(FINISH, "FSM");
    //     ROS_WARN("[FSM] Finish exploration: No frontier detected");
    //     fd_->static_state_ = true;
    //     replan_pub_.publish(std_msgs::Empty());
    //   }
    // }
    if (need_replan) {
      // visualize();
      transitState(PLAN_TRAJ, "FSM");
    }

    break;
  }

  case WAIT_FRONTIER: {
    state_flag.data = WAIT_FRONTIER;
    expl_manager_->frontier_finder_->searchFrontiers(fd_->odom_pos_);
    expl_manager_->frontier_finder_->computeFrontiersToVisit(fd_->odom_pos_);
    bool neighbor;
    expl_manager_->frontier_finder_->clusterFrontiers(fd_->odom_pos_, neighbor);
    expl_manager_->frontier_finder_->getFrontiers(expl_manager_->ed_->frontiers_);
    if (!(expl_manager_->ed_->frontiers_.empty()))
      transitState(PLAN_TRAJ, "FSM");
    break;
  }

  case WAIT_TRAV: {
    state_flag.data = WAIT_TRAV;
    expl_manager_->frontier_finder_->searchFrontiers(fd_->odom_pos_);
    expl_manager_->frontier_finder_->computeFrontiersToVisit(fd_->odom_pos_);
    bool neighbor;
    expl_manager_->frontier_finder_->clusterFrontiers(fd_->odom_pos_, neighbor);
    vector<Eigen::Vector3d> centers;
    expl_manager_->frontier_finder_->getClusterCenter(centers);
    Eigen::Vector3d pos = fd_->odom_pos_;
    bool traversable = false;
    planner_manager_->path_finder_->reset();
    for (auto center : centers) {
      if(planner_manager_->path_finder_->search(pos, center) == Astar::REACH_END) {
        traversable = true;
        printf("\033[32mTured traversable, traversable center:[%f, %f, %f].\033[0m\n", center[0], center[1], center[2]);
        transitState(PLAN_TRAJ, "FSM");
        break;
      }
    }
    if(traversable)
      break;
    planner_manager_->path_finder_->reset();
    if(planner_manager_->path_finder_->search(pos, expl_manager_->ed_->blocked_seg_end_) == Astar::REACH_END) {
      printf("\033[32mThe blocked seg end [%f, %f, %f] tured traversable.\033[0m\n", expl_manager_->ed_->blocked_seg_end_[0], expl_manager_->ed_->blocked_seg_end_[1], expl_manager_->ed_->blocked_seg_end_[2]);
      transitState(PLAN_TRAJ, "FSM");
      break;
    }
    // printf("\033[33mStill blocked.\033[0m\n");
    ROS_WARN_THROTTLE(1.0, "\033[33mStill blocked.\033[0m\n");
    break;
  }
  }

  // thread vis_thread(&FastExplorationFSM::visualize, this);
  // vis_thread.detach();
  start_flag_pub.publish(state_flag);
  // std::cout<<"The State of the drone is: "<<state_flag.data<<endl;
}

int FastExplorationFSM::callExplorationPlanner() {
  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

  // int res = expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_,
  //                                            fd_->start_acc_,
  //                                            fd_->start_yaw_);
  int res = expl_manager_->planExploreMotionCluster(
      fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_);
  // int res = expl_manager_->planExploreMotionHGrid(
  //     fd_->start_pt_, fd_->start_vel_, fd_->start_acc_, fd_->start_yaw_);
  classic_ = false;

  // int res = expl_manager_->classicFrontier(fd_->start_pt_,
  // fd_->start_yaw_[0]); classic_ = true;

  // int res = expl_manager_->rapidFrontier(fd_->start_pt_, fd_->start_vel_,
  // fd_->start_yaw_[0], classic_);

  if (res == SUCCEED) {
    auto info = &planner_manager_->local_data_;
    info->start_time_ =
        (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
    fd_->newest_traj_ = bspline;
  }
  return res;
}

void FastExplorationFSM::visualize() {
  auto info = &planner_manager_->local_data_;
  auto plan_data = &planner_manager_->plan_data_;
  auto ed_ptr = expl_manager_->ed_;
  auto ft = expl_manager_->frontier_finder_;

  visualization_->drawLines(ed_ptr->local_tour_vis_, 0.1,
                            PlanningVisualization::Color::Red(), "local_tour",
                            0, PlanningVisualization::PUBLISHER::HGRID);
  visualization_->drawLines(
      ed_ptr->grid_tour_, 0.1, PlanningVisualization::Color::Purple(),
      "global_tour", 0, PlanningVisualization::PUBLISHER::HGRID);
  // visualization_->drawLines(
  //     ed_ptr->inertia_tour_, 0.1, PlanningVisualization::Color::DeepGreen(),
  //     "inertial_tour", 0, PlanningVisualization::PUBLISHER::HGRID);
  // Draw a line from frontier top view point with its view direction
  ed_ptr->views_.clear();
  for (int i = 0; i < ed_ptr->points_.size(); ++i)
    ed_ptr->views_.push_back(
        ed_ptr->points_[i] +
        2.0 * Vector3d(cos(ed_ptr->yaws_[i]), sin(ed_ptr->yaws_[i]), 0));

  visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05,
                            PlanningVisualization::Color::SpringGreen(),
                            "frontier_view", 0,
                            PlanningVisualization::PUBLISHER::VIEWPOINT);
  visualization_->drawBspline(info->position_traj_, 0.1,
                              Vector4d(1.0, 0.0, 0.0, 1), false, 0.15,
                              Vector4d(1, 1, 0, 1));

  vector<vector<Eigen::Vector3d>> frontier_division;
  ft->getFrontierDivision(frontier_division);
  for (int i = 0; i < frontier_division.size(); i++) {
    Eigen::Vector4d color =
        visualization_->getColor(double(i) / frontier_division.size(), 1.0);
    visualization_->drawLines({}, 0.2, color, "frt_division", i,
                              PlanningVisualization::PUBLISHER::VIEWPOINT);

    visualization_->drawSpheres(frontier_division[i], 0.2, color,
                                "frt_division", i,
                                PlanningVisualization::PUBLISHER::VIEWPOINT);
  }

  int frts_pts_num = 0;
  for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
    frts_pts_num += ed_ptr->frontiers_[i].size();
  }
  static pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  cloud.points.clear();
  cloud.points.reserve(frts_pts_num);
  cloud.width = frts_pts_num;
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = "world";
  int i = 0;
  for (auto ft : ed_ptr->frontiers_) {
    i++;
    int j = 0;
    while (j < ft.size()) {
      pcl::PointXYZRGBA pt;
      Eigen::Vector4d color = 
          visualization_->getColor(0.0, 0.3);
      pt.x = ft[j].x();
      pt.y = ft[j].y();
      pt.z = ft[j].z();
      pt.r = color(0) * 255;
      pt.g = color(1) * 255;
      pt.b = color(2) * 255;
      pt.a = 50;
      cloud.push_back(pt);
      j++;
    }
  }
  sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
  //std::cout << "\033[42mcloud.width = " << cloud.width << ", cloud.points.size() = " << cloud.points.size() << "\033[0m" << std::endl;
  pcl::toROSMsg(cloud, *cloud_msg);
  visualization_->frts_pub_.publish(cloud_msg);
  
  // int dormant_frts_pts_num = 0;
  // vector<vector<Vector3d>> dormant_frontiers;
  // ft->getDormantFrontiers(dormant_frontiers);
  // for (int i = 0; i < dormant_frontiers.size(); ++i) {
  //   dormant_frts_pts_num += dormant_frontiers[i].size();
  // }
  // pcl::PointCloud<pcl::PointXYZRGBA> dormant_cloud;
  // dormant_cloud.width = dormant_frts_pts_num;
  // dormant_cloud.height = 1;
  // dormant_cloud.is_dense = true;
  // dormant_cloud.header.frame_id = "world";
  // for (auto ft : dormant_frontiers) {
  //   int j = 0;
  //   while (j < ft.size()) {
  //     pcl::PointXYZRGBA pt;
  //     Eigen::Vector4d color =
  //         visualization_->getColor(2.0 / 6, 0.3);
  //     pt.x = ft[j].x();
  //     pt.y = ft[j].y();
  //     pt.z = ft[j].z();
  //     pt.r = color(0) * 255;
  //     pt.g = color(1) * 255;
  //     pt.b = color(2) * 255;
  //     pt.a = 50;
  //     dormant_cloud.push_back(pt);
  //     j++;
  //   }
  // }
  // sensor_msgs::PointCloud2::Ptr dormant_cloud_msg(new sensor_msgs::PointCloud2);
  // pcl::toROSMsg(dormant_cloud, *dormant_cloud_msg);
  // visualization_->dormant_frts_pub_.publish(dormant_cloud_msg);
}

void FastExplorationFSM::clearVisMarker() {
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0,
  // 6); visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1),
  // "global_tour", 0, 6); visualization_->drawSpheres({}, 0.2, Vector4d(0, 0,
  // 1, 1), "refined_pts", 0, 6); visualization_->drawLines({}, {}, 0.05,
  // Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour",
  // 0, 6); visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1),
  // "B-Spline", 0, 0);

  // visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1),
  // "current_pose", 0, 6);
}

void FastExplorationFSM::frontierCallback(const ros::TimerEvent &e) {
  static int delay = 0;
  if (++delay < 5)
    return;

  if (state_ == WAIT_TRIGGER || state_ == FINISH) {
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;
    // Eigen::Vector3d debug_update_min, debug_update_max;
    // ft->edt_env_->sdf_map_->getUpdatedBox(debug_update_min, debug_update_max,
    //                                       false);
    bool neighbor;
    ft->searchFrontiers(fd_->start_pt_);
    ft->computeFrontiersToVisit(fd_->start_pt_);
    // ft->updateFrontierCostMatrix();
    ft->clusterFrontiers(fd_->start_pt_, neighbor);
    ft->getFrontiers(expl_manager_->ed_->frontiers_);
    ft->getDormantFrontiers(expl_manager_->ed_->dead_frontiers_);
    ft->getFrontierBoxes(expl_manager_->ed_->frontier_boxes_);

    ft->getTopViewpointsInfo(fd_->odom_pos_, expl_manager_->ed_->points_,
                             expl_manager_->ed_->yaws_,
                             expl_manager_->ed_->averages_);

    expl_manager_->ed_->views_.clear();
    for (int i = 0; i < expl_manager_->ed_->points_.size(); ++i)
      expl_manager_->ed_->views_.push_back(
          expl_manager_->ed_->points_[i] +
          2.0 * Vector3d(cos(expl_manager_->ed_->yaws_[i]),
                         sin(expl_manager_->ed_->yaws_[i]), 0));
    visualization_->drawLines(
        expl_manager_->ed_->points_, expl_manager_->ed_->views_, 0.05,
        PlanningVisualization::Color::SpringGreen(), "frontier_view", 0,
        PlanningVisualization::PUBLISHER::VIEWPOINT);
    // int frts_pts_num = 0, i = 0;
    // pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    // for (auto ft : ed->frontiers_) {
    //   int j = 0;
    //   while (j < ft.size()) {
    //     pcl::PointXYZRGBA pt;
    //     Eigen::Vector4d color =
    //         visualization_->getColor(double(i) / ed->frontiers_.size(), 0.3);
    //     pt.x = ft[j].x();
    //     pt.y = ft[j].y();
    //     pt.z = ft[j].z();
    //     pt.r = color(0) * 255;
    //     pt.g = color(1) * 255;
    //     pt.b = color(2) * 255;
    //     pt.a = 50;
    //     cloud.push_back(pt);
    //     j++;
    //     frts_pts_num++;
    //   }
    //   i++;
    // }
    // cloud.width = frts_pts_num;
    // cloud.height = 1;
    // cloud.is_dense = true;
    // cloud.header.frame_id = "world";
    // sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
    // pcl::toROSMsg(cloud, *cloud_msg);
  }
}

void FastExplorationFSM::tagCallback(const ros::TimerEvent &e) {
  // 定义要监听的 AprilTag 名称范围（apriltag_0 到 apriltag_7）
    const int num_tags = 8;
    for (int tag_id = 0; tag_id < num_tags; ++tag_id) {
        std::string tag_frame = "apriltag_" + std::to_string(tag_id);
        std::string target_frame = "world";

        try {
            // 获取从 world 到 tag 的变换（时间戳为最新可用）
            geometry_msgs::TransformStamped transform = 
                tf_buffer_.lookupTransform(
                    target_frame, 
                    tag_frame,
                    ros::Time(0)  // 获取最新可用的变换
                );

            // 更新位姿数据
            tag_poses[tag_id] = transform;
        } catch (tf2::TransformException &ex) {
            // 处理异常（例如标签未检测到）
            // ROS_WARN("Failed to get transform for %s: %s", 
            //     tag_frame.c_str(), ex.what());
            continue;
        }
    }
    if(fp_->show_tag)
    {
      // 统一打印当前检测到的所有标签位姿
      ROS_INFO("===== Current AprilTag Poses in World Frame =====");
      for (const auto& pair : tag_poses) {
          int tag_id = pair.first;
          const geometry_msgs::TransformStamped& transform = pair.second;
          ROS_INFO_STREAM(
              "[Tag " << tag_id << "]\n"
              << "  Position:    [" 
              << transform.transform.translation.x << ", "
              << transform.transform.translation.y << ", "
              << transform.transform.translation.z << "]\n"
              << "  Orientation: ["
              << transform.transform.rotation.x << ", "
              << transform.transform.rotation.y << ", "
              << transform.transform.rotation.z << ", "
              << transform.transform.rotation.w << "]"
          );
      }
      ROS_INFO("================================================\n");
    }
    return;
}

void FastExplorationFSM::triggerCallback(const nav_msgs::PathConstPtr &msg) {
  if (msg->poses[0].pose.position.z < -0.1)
    return;
  if (state_ != WAIT_TRIGGER)
    return;
  fd_->trigger_ = true;
  cout << "Triggered!" << endl;
  transitState(PLAN_TRAJ, "triggerCallback");
}

void FastExplorationFSM::safetyCallback(const ros::TimerEvent &e) {
  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    // Check safety and trigger replan if necessary
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      fd_->static_state_ = true;
      ROS_WARN("Replan: collision detected==================================");
      transitState(PLAN_TRAJ, "safetyCallback");
    }
  }
}

void FastExplorationFSM::odometryCallback(
    const nav_msgs::OdometryConstPtr &msg) {
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x =
      fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  fd_->have_odom_ = true;
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " +
              fd_->state_str_[int(new_state)]
       << endl;
}
} // namespace fast_planner
