#ifndef _SDF_MAP_H
#define _SDF_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <memory>
#include <queue>
#include <ros/ros.h>
#include <tuple>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <nav_msgs/OccupancyGrid.h>
#include <tuw_voronoi_graph/voronoi_graph_node.h>
#include <plan_env/map_ros.h>

using namespace std;

namespace cv {
class Mat;
}

class RayCaster;

namespace fast_planner {
struct MapParam;
struct MapData;
class MapROS;
class camlidFusion;
class SDFMap {
public:
  SDFMap();
  ~SDFMap();

  enum OCCUPANCY {
    UNKNOWN,
    FREE,
    UNDEROBSERVED,
    OCCUPIED,
  };

  enum FRESHNESS {
    NOTINTERESTED,
    FRESH,
    NOTFRESH,
  };

  void initMap(ros::NodeHandle &nh);
  void inputCamLidarPointCloud(const Eigen::MatrixXd &points,
                               const Eigen::MatrixXd &pts_uv,
                               const Eigen::MatrixXd &lidar_pts,
                               const Eigen::Vector3d &lidar_pos);
  void inputCamLidarSemanticPointCloud(const Eigen::MatrixXd &points,
                                       const Eigen::MatrixXd &pts_uv,
                                       const Eigen::MatrixXd &lidar_pts,
                                       const Eigen::Vector3d &lidar_pos,
                                       const Eigen::VectorXi &semantic_label);
  void inputPointCloud(const pcl::PointCloud<pcl::PointXYZ> &points,
                       const int &point_num, const Eigen::Vector3d &camera_pos);

  void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
  void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
  void boundIndex(Eigen::Vector3i &id);
  int toAddress(const Eigen::Vector3i &id);
  int toAddress(const int &x, const int &y, const int &z);
  bool isInMap(const Eigen::Vector3d &pos);
  bool isInMap(const Eigen::Vector3i &idx);
  bool isInBox(const Eigen::Vector3i &id);
  bool isInBox(const Eigen::Vector3d &pos);
  void boundBox(Eigen::Vector3d &low, Eigen::Vector3d &up);
  double getObserverDist(const Eigen::Vector3d &pos);
  double getObserverDist(const Eigen::Vector3i &id);
  int getOccupancy(const Eigen::Vector3d &pos);
  int getOccupancy(const Eigen::Vector3i &id);
  int getFreshness(const Eigen::Vector3d &pos);
  int getFreshness(const Eigen::Vector3i &id);
  void setOccupied(const Eigen::Vector3d &pos, const int &occ = 1);
  int getInflateOccupancy(const Eigen::Vector3d &pos);
  int getInflateOccupancy(const Eigen::Vector3i &id);
  double getDistance(const Eigen::Vector3d &pos);
  double getDistance(const Eigen::Vector3i &id);
  void getDistanceAndGradient(const Eigen::Vector3d &pos, double &distance,
                              Eigen::Vector3d &gradient);
  void getDistanceAndGradient(const Eigen::Vector3i &idx, double &distance,
                              Eigen::Vector3d &gradient);
  double getDistWithGrad(const Eigen::Vector3d &pos, Eigen::Vector3d &grad);
  void updateESDF3d();
  void resetBuffer();
  void resetBuffer(const Eigen::Vector3d &min, const Eigen::Vector3d &max);

  void getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size);
  void getBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax);
  void getUpdatedBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax,
                     bool reset = false);
  double getResolution();
  int getVoxelNum();
  double getZ();
  double getBeliefDist();
  MapROS* getMapROS() {return mr_.get();};

private:
  void clearAndInflateLocalMap();
  void clearAndInflateLocalMapSemantic();
  void inflatePoint(const Eigen::Vector3i &pt, int step,
                    vector<Eigen::Vector3i> &pts);
  void setCacheOccupancy(const int &adr, const int &occ);
  void setCacheSemanticOccupancy(const int &adr, const int &occ);
  void setObservedDist(const int &adr, const double &dist);
  void fadingCallback(const ros::TimerEvent & /*event*/);
  nav_msgs::OccupancyGrid convert3DMapLayerToOccupancyGrid(int z_layer);
  Eigen::Vector3d closetPointInMap(const Eigen::Vector3d &pt,
                                   const Eigen::Vector3d &camera_pt);
  template <typename F_get_val, typename F_set_val>
  void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end,
                int dim);
  vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i &voxel);
  bool isInObstacle(const Eigen::Vector3i idx);
  ros::Publisher debug_pts_pub_;
  std::unique_ptr<MapParam> mp_;
  std::unique_ptr<MapData> md_;
  std::unique_ptr<tuw_graph::VoronoiGeneratorNode> vg_;
  // std::unique_ptr<MapROS> mr_;
  std::unique_ptr<RayCaster> caster_;
  std::unique_ptr<MapROS> mr_;
  ros::Timer fading_timer_;
  friend MapROS;
  friend camlidFusion;

public:
  typedef std::shared_ptr<SDFMap> Ptr;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct MapParam {
  // map properties
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;
  Eigen::Vector3i map_voxel_num_;
  double belief_dist_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  double virtual_ceil_height_, ground_height_;
  Eigen::Vector3i box_min_, box_max_;
  Eigen::Vector3d box_mind_, box_maxd_;
  double default_dist_;
  bool optimistic_, signed_dist_;
  double fading_time_;
  // map fusion
  double p_hit_, p_miss_, p_min_, p_max_, p_occ_; // occupancy probability
  double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_,
      min_occupancy_log_; // logit
  double max_ray_length_;
  double local_bound_inflate_;
  int local_map_margin_;
  double unknown_flag_;
};

struct MapData {
  // main map data, occupancy of each voxel and Euclidean distance
  std::vector<double> occupancy_buffer_;
  std::vector<char> occupancy_buffer_inflate_;
  std::vector<double> semantic_occupancy_buffer_;
  std::vector<char> semantic_occupancy_buffer_inflate_;
  std::vector<double> freshness_value_;
  std::vector<double> min_observed_dist_;
  std::vector<double> distance_buffer_neg_;
  std::vector<double> distance_buffer_;
  std::vector<double> tmp_buffer1_;
  std::vector<double> tmp_buffer2_;
  // data for updating
  vector<short> count_hit_, count_miss_, count_hit_and_miss_;
  vector<short> semantic_count_hit_, semantic_count_miss_, semantic_count_hit_and_miss_;
  vector<char> flag_rayend_, flag_visited_;
  char raycast_num_;
  queue<int> cache_voxel_;
  queue<int> cache_semantic_voxel_;
  Eigen::Vector3i local_bound_min_, local_bound_max_;
  Eigen::Vector3d update_min_, update_max_;
  bool reset_updated_box_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline void SDFMap::posToIndex(const Eigen::Vector3d &pos,
                               Eigen::Vector3i &id) {
  for (int i = 0; i < 3; ++i)
    id(i) = floor((pos(i) - mp_->map_origin_(i)) * mp_->resolution_inv_);
}

inline void SDFMap::indexToPos(const Eigen::Vector3i &id,
                               Eigen::Vector3d &pos) {
  for (int i = 0; i < 3; ++i)
    pos(i) = (id(i) + 0.5) * mp_->resolution_ + mp_->map_origin_(i);
}

inline void SDFMap::boundIndex(Eigen::Vector3i &id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_->map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_->map_voxel_num_(1) - 1), 0);
  id1(2) = max(min(id(2), mp_->map_voxel_num_(2) - 1), 0);
  id = id1;
}

inline int SDFMap::toAddress(const int &x, const int &y, const int &z) {
  return x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) +
         y * mp_->map_voxel_num_(2) + z;
}

inline int SDFMap::toAddress(const Eigen::Vector3i &id) {
  return toAddress(id[0], id[1], id[2]);
}

inline bool SDFMap::isInMap(const Eigen::Vector3d &pos) {
  if (pos(0) < mp_->map_min_boundary_(0) + 1e-4 ||
      pos(1) < mp_->map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_->map_min_boundary_(2) + 1e-4)
    return false;
  if (pos(0) > mp_->map_max_boundary_(0) - 1e-4 ||
      pos(1) > mp_->map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_->map_max_boundary_(2) - 1e-4)
    return false;
  return true;
}

inline bool SDFMap::isInMap(const Eigen::Vector3i &idx) {
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
    return false;
  if (idx(0) > mp_->map_voxel_num_(0) - 1 ||
      idx(1) > mp_->map_voxel_num_(1) - 1 ||
      idx(2) > mp_->map_voxel_num_(2) - 1)
    return false;
  return true;
}

inline bool SDFMap::isInBox(const Eigen::Vector3i &id) {
  for (int i = 0; i < 3; ++i) {
    if (id[i] < mp_->box_min_[i] || id[i] >= mp_->box_max_[i]) {
      return false;
    }
  }
  return true;
}

inline bool SDFMap::isInBox(const Eigen::Vector3d &pos) {
  for (int i = 0; i < 3; ++i) {
    if (pos[i] <= mp_->box_mind_[i] || pos[i] >= mp_->box_maxd_[i]) {
      return false;
    }
  }
  return true;
}

inline void SDFMap::boundBox(Eigen::Vector3d &low, Eigen::Vector3d &up) {
  for (int i = 0; i < 3; ++i) {
    low[i] = max(low[i], mp_->box_mind_[i]);
    up[i] = min(up[i], mp_->box_maxd_[i]);
  }
}
inline double SDFMap::getObserverDist(const Eigen::Vector3d &pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getOccupancy(id);
}
inline double SDFMap::getObserverDist(const Eigen::Vector3i &id) {
  if (!isInMap(id))
    return -1;
  double min_dist = md_->min_observed_dist_[toAddress(id)];
  return min_dist;
}
inline int SDFMap::getOccupancy(const Eigen::Vector3i &id) {
  if (!isInMap(id))
    return -1;
  double occ = md_->occupancy_buffer_[toAddress(id)];
  double observed_dist = md_->min_observed_dist_[toAddress(id)];
  if (occ < mp_->clamp_min_log_ - 1e-3)
    return UNKNOWN;

  if (occ > mp_->min_occupancy_log_) {
    if (observed_dist > mp_->belief_dist_)
      return UNDEROBSERVED;
    else
      return OCCUPIED;
  }

  return FREE;
}

inline int SDFMap::getOccupancy(const Eigen::Vector3d &pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getOccupancy(id);
}

inline int SDFMap::getFreshness(const Eigen::Vector3i &id) {
  if (!isInMap(id))
    return -1;

  double reduce = 0;
  if (mp_->fading_time_ > 0)
    reduce = (mp_->clamp_max_log_ - mp_->min_occupancy_log_) / mp_->fading_time_;
  const double low_thres = mp_->clamp_min_log_ + reduce;
  
  if(md_->freshness_value_[toAddress(id)] == -5)
    return NOTINTERESTED;
  else if(md_->freshness_value_[toAddress(id)] > low_thres)
    return FRESH;
  else
    return NOTFRESH;
}

inline int SDFMap::getFreshness(const Eigen::Vector3d &pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getFreshness(id);
}

inline void SDFMap::setOccupied(const Eigen::Vector3d &pos, const int &occ) {
  if (!isInMap(pos))
    return;
  Eigen::Vector3i id;
  posToIndex(pos, id);
  md_->occupancy_buffer_inflate_[toAddress(id)] = occ;
}

inline int SDFMap::getInflateOccupancy(const Eigen::Vector3i &id) {
  if (!isInMap(id))
    return -1;
  return int(md_->occupancy_buffer_inflate_[toAddress(id)]);
}

inline int SDFMap::getInflateOccupancy(const Eigen::Vector3d &pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getInflateOccupancy(id);
}

inline double SDFMap::getDistance(const Eigen::Vector3i &id) {
  if (!isInMap(id))
    return -1;
  return md_->distance_buffer_[toAddress(id)];
}

inline double SDFMap::getDistance(const Eigen::Vector3d &pos) {
  Eigen::Vector3i id;
  posToIndex(pos, id);
  return getDistance(id);
}

inline void SDFMap::inflatePoint(const Eigen::Vector3i &pt, int step,
                                 vector<Eigen::Vector3i> &pts) {
  int num = 0;

  /* ---------- + shape inflate ---------- */
  // for (int x = -step; x <= step; ++x)
  // {
  //   if (x == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1), pt(2));
  // }
  // for (int y = -step; y <= step; ++y)
  // {
  //   if (y == 0)
  //     continue;
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1) + y, pt(2));
  // }
  // for (int z = -1; z <= 1; ++z)
  // {
  //   pts[num++] = Eigen::Vector3i(pt(0), pt(1), pt(2) + z);
  // }

  /* ---------- all inflate ---------- */
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z) {
        pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
      }
}
} // namespace fast_planner
#endif