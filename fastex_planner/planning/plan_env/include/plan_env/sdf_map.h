#ifndef _SDF_MAP_H
#define _SDF_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <memory>
#include <unordered_set>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <mutex>

using namespace std;

namespace cv
{
  class Mat;
}

class RayCaster;

namespace fast_planner
{
  struct MapParam;
  struct MapData;
  class MapROS;

  class SDFMap
  {
  public:
    SDFMap();
    ~SDFMap();

    struct CollisionCheckResult
    {
      bool is_collision_free;
      bool crosses_unknown;
    };

    enum OCCUPANCY
    {
      UNKNOWN,
      FREE,
      OCCUPIED
    };

    enum CHANGE_COLLECTION_MODE
    {
      ALL_IN_PERCEPTION_RANGE,
      ONLY_UPDATED
    };

    void initMap(ros::NodeHandle &nh);
    void inputPointCloud(const pcl::PointCloud<pcl::PointXYZ> &points, const int &point_num,
                         const Eigen::Vector3d &sensor_pos);

    // convert double (position) to int (voxel index)
    void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
    // convert int (voxel index) to double (position); position is the voxel center
    void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos, const double offset = 0.5);
    // clamp each dimension of the voxel index to [0, map_voxel_num_-1)
    void boundIndex(Eigen::Vector3i &id);
    // convert to flat address (linear storage)
    int toAddress(const Eigen::Vector3i &id);
    int toAddress(const int &x, const int &y, const int &z);
    // convert flat address back to voxel id
    Eigen::Vector3i AddressToIndex(const int &addr);
    Eigen::Vector3d AddressToPos(const int &addr);
    // check if a point (double coordinates) is inside the map
    bool isInMap(const Eigen::Vector3d &pos);
    // check if a point (int voxel coordinates) is inside the map
    bool isInMap(const Eigen::Vector3i &idx);
    // check if a point (double coordinates) is inside the bounding box
    bool isInBox(const Eigen::Vector3i &id);
    // check if a point (int voxel coordinates) is inside the bounding box
    bool isInBox(const Eigen::Vector3d &pos);
    // re-determine the box boundary
    void boundBox(Eigen::Vector3d &low, Eigen::Vector3d &up);
    // get state information for the current position
    int getOccupancy(const Eigen::Vector3d &pos);
    // get state information for the current voxel index
    int getOccupancy(const Eigen::Vector3i &id);
    // set state for the current voxel index (occupancy_buffer_inflate_)
    void setOccupied(const Eigen::Vector3d &pos, const int &occ = 1);
    // get occupancy_buffer_inflate_ value for the current position
    int getInflateOccupancy(const Eigen::Vector3d &pos);
    int getInflateOccupancy(const Eigen::Vector3i &id);
    // get distance value from distance_buffer_
    double getDistance(const Eigen::Vector3d &pos);
    double getDistance(const Eigen::Vector3i &id);

    bool isNeighborUnknown(const Eigen::Vector3d &pos, const double range);
    bool isNeighborOccupied(const Eigen::Vector3d &pos, const double range);

    bool isFree(const Eigen::Vector3d &pos);
    bool isFree(const Eigen::Vector3i &id);
    bool isUnknown(const Eigen::Vector3d &pos);
    bool isUnknown(const Eigen::Vector3i &id);
    bool isOccupied(const Eigen::Vector3d &pos);
    bool isOccupied(const Eigen::Vector3i &id);
    bool isInflatedFree(const Eigen::Vector3d &pos);
    bool isInflatedFree(const Eigen::Vector3i &id);
    bool isInflatedUnknown(const Eigen::Vector3d &pos);
    bool isInflatedUnknown(const Eigen::Vector3i &id);
    bool isInflatedOccupied(const Eigen::Vector3d &pos);
    bool isInflatedOccupied(const Eigen::Vector3i &id);

    void resetChangeDetection();
    void enableChangeDetection();
    void disableChangeDetection();
    std::vector<int>::iterator changesBegin();
    std::vector<int>::iterator changesEnd();
    double getMaxRayLength();
    CollisionCheckResult isStrictCollisionFreeStraight(const Eigen::Vector3d &source, const Eigen::Vector3d &target);
    CollisionCheckResult isOptimisticCollisionFreeStraight(const Eigen::Vector3d &source, const Eigen::Vector3d &target);
    CollisionCheckResult isStrictInflatedCollisionFreeStraight(const Eigen::Vector3d &source, const Eigen::Vector3d &target);
    CollisionCheckResult isOptimisticInflatedCollisionFreeStraight(const Eigen::Vector3d &source, const Eigen::Vector3d &target);

    double getDistWithGrad(const Eigen::Vector3d &pos, Eigen::Vector3d &grad);
    void updateESDF3d();
    void resetBuffer();
    void resetBuffer(const Eigen::Vector3d &min, const Eigen::Vector3d &max);

    void getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size);
    void getBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax);
    void getUpdatedBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax, bool reset = false);
    double getResolution();
    int getVoxelNum();
    double getInflateRadius();

    void setChangeCollectionMode(CHANGE_COLLECTION_MODE mode);

    bool isSDFMapGot();

    // Find the nearest free cell using BFS within a search box
    bool bfsNearestFree(const Eigen::Vector3d &search_box_size,
                        const Eigen::Vector3d &center_pos,
                        Eigen::Vector3d &nearest_valid_pos);

    // Round a position to the nearest grid cell center
    void roundPosition(const Eigen::Vector3d &pos, Eigen::Vector3d &rounded_pos);

  private:
    void clearAndInflateLocalMap();
    // inflate a point (dilate occupied region)
    void inflatePoint(const Eigen::Vector3i &pt, int step, vector<Eigen::Vector3i> &pts);
    void setCacheOccupancy(const int &adr, const int &occ);
    // find the closest point to pt along the ray from sensor_pt, within map bounds
    Eigen::Vector3d closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &sensor_pt);
    template <typename F_get_val, typename F_set_val>
    void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

    unique_ptr<MapParam> mp_;
    unique_ptr<MapData> md_;
    unique_ptr<MapROS> mr_;
    unique_ptr<RayCaster> caster_;

    friend MapROS;

  public:
    typedef std::shared_ptr<SDFMap> Ptr;
    std::mutex sdf_mutex_;
    std::mutex change_mutex_;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  struct MapParam
  {
    // map properties
    Eigen::Vector3d map_origin_, map_size_;
    Eigen::Vector3d map_min_boundary_, map_max_boundary_;
    Eigen::Vector3i map_voxel_num_;
    double resolution_, resolution_inv_;
    double obstacles_inflation_;
    double virtual_ceil_height_, ground_height_;
    Eigen::Vector3i box_min_, box_max_;
    Eigen::Vector3d box_mind_, box_maxd_;
    double default_dist_;
    bool optimistic_, signed_dist_;
    // map fusion
    double p_hit_, p_miss_, p_min_, p_max_, p_occ_;                                           // occupancy probability
    double prob_hit_log_, prob_miss_log_, clamp_min_log_, clamp_max_log_, min_occupancy_log_; // logit
    double max_ray_length_;
    double local_bound_inflate_;
    int local_map_margin_;
    double unknown_flag_;
    int change_colletion_mode_;
    bool enable_change_detection_;
  };

  struct MapData
  {
    // main map data, occupancy of each voxel and Euclidean distance
    std::vector<double> occupancy_buffer_;
    std::vector<char> occupancy_buffer_inflate_;
    std::vector<double> distance_buffer_neg_;
    std::vector<double> distance_buffer_;
    std::vector<double> tmp_buffer1_;
    std::vector<double> tmp_buffer2_;
    std::vector<int> change_buffer_;
    // data for updating
    vector<short> count_hit_, count_miss_, count_hit_and_miss_;
    vector<char> flag_rayend_, flag_visited_;
    char raycast_num_;
    queue<int> cache_voxel_;
    Eigen::Vector3i local_bound_min_, local_bound_max_;
    Eigen::Vector3d update_min_, update_max_;
    Eigen::Vector3d occupancy_min_, occupancy_max_;
    Eigen::Vector3i occupancy_min_i_, occupancy_max_i_;
    bool sdf_map_got_;
    bool reset_updated_box_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  inline void SDFMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
  {
    for (int i = 0; i < 3; ++i)
      id(i) = floor((pos(i) - mp_->map_origin_(i)) * mp_->resolution_inv_);
  }

  inline void SDFMap::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos, const double offset)
  {
    for (int i = 0; i < 3; ++i)
      pos(i) = (id(i) + offset) * mp_->resolution_ + mp_->map_origin_(i);
  }

  inline void SDFMap::boundIndex(Eigen::Vector3i &id)
  {
    Eigen::Vector3i id1;
    id1(0) = max(min(id(0), mp_->map_voxel_num_(0) - 1), 0);
    id1(1) = max(min(id(1), mp_->map_voxel_num_(1) - 1), 0);
    id1(2) = max(min(id(2), mp_->map_voxel_num_(2) - 1), 0);
    id = id1;
  }

  inline int SDFMap::toAddress(const int &x, const int &y, const int &z)
  {
    return x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) + y * mp_->map_voxel_num_(2) + z;
  }

  inline int SDFMap::toAddress(const Eigen::Vector3i &id)
  {
    return toAddress(id[0], id[1], id[2]);
  }

  inline Eigen::Vector3i SDFMap::AddressToIndex(const int &addr)
  {
    return Eigen::Vector3i(addr / (mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2)),
                           addr % (mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2)) / mp_->map_voxel_num_(2),
                           addr % (mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2)) % mp_->map_voxel_num_(2));
  }

  inline Eigen::Vector3d SDFMap::AddressToPos(const int &addr)
  {
    Eigen::Vector3d pos;
    indexToPos(AddressToIndex(addr), pos);
    return pos;
  }

  inline bool SDFMap::isInMap(const Eigen::Vector3d &pos)
  {
    if (pos(0) < mp_->map_min_boundary_(0) + 1e-4 || pos(1) < mp_->map_min_boundary_(1) + 1e-4 ||
        pos(2) < mp_->map_min_boundary_(2) + 1e-4)
      return false;
    if (pos(0) > mp_->map_max_boundary_(0) - 1e-4 || pos(1) > mp_->map_max_boundary_(1) - 1e-4 ||
        pos(2) > mp_->map_max_boundary_(2) - 1e-4)
      return false;
    return true;
  }

  inline bool SDFMap::isInMap(const Eigen::Vector3i &idx)
  {
    if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
      return false;
    if (idx(0) > mp_->map_voxel_num_(0) - 1 || idx(1) > mp_->map_voxel_num_(1) - 1 ||
        idx(2) > mp_->map_voxel_num_(2) - 1)
      return false;
    return true;
  }

  inline bool SDFMap::isInBox(const Eigen::Vector3i &id)
  {
    for (int i = 0; i < 3; ++i)
    {
      if (id[i] < mp_->box_min_[i] || id[i] >= mp_->box_max_[i])
      {
        return false;
      }
    }
    return true;
  }

  inline bool SDFMap::isInBox(const Eigen::Vector3d &pos)
  {
    for (int i = 0; i < 3; ++i)
    {
      if (pos[i] <= mp_->box_mind_[i] || pos[i] >= mp_->box_maxd_[i])
      {
        return false;
      }
    }
    return true;
  }

  inline void SDFMap::boundBox(Eigen::Vector3d &low, Eigen::Vector3d &up)
  {
    for (int i = 0; i < 3; ++i)
    {
      low[i] = max(low[i], mp_->box_mind_[i]);
      up[i] = min(up[i], mp_->box_maxd_[i]);
    }
  }

  inline int SDFMap::getOccupancy(const Eigen::Vector3i &id)
  {
    if (!isInMap(id))
      return -1;
    double occ = md_->occupancy_buffer_[toAddress(id)];
    if (occ < mp_->clamp_min_log_ - 1e-3)
      return UNKNOWN;
    if (occ > mp_->min_occupancy_log_)
      return OCCUPIED;
    return FREE;
  }

  inline int SDFMap::getOccupancy(const Eigen::Vector3d &pos)
  {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return getOccupancy(id);
  }

  inline void SDFMap::setOccupied(const Eigen::Vector3d &pos, const int &occ)
  {
    if (!isInMap(pos))
      return;
    Eigen::Vector3i id;
    posToIndex(pos, id);
    md_->occupancy_buffer_inflate_[toAddress(id)] = occ;
  }

  inline int SDFMap::getInflateOccupancy(const Eigen::Vector3i &id)
  {
    if (!isInMap(id))
      return -1;
    return int(md_->occupancy_buffer_inflate_[toAddress(id)]);
  }

  inline int SDFMap::getInflateOccupancy(const Eigen::Vector3d &pos)
  {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return getInflateOccupancy(id);
  }

  inline double SDFMap::getDistance(const Eigen::Vector3i &id)
  {
    if (!isInMap(id))
      return -1;
    return md_->distance_buffer_[toAddress(id)];
  }

  inline double SDFMap::getDistance(const Eigen::Vector3d &pos)
  {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return getDistance(id);
  }

  inline bool SDFMap::isNeighborUnknown(const Eigen::Vector3d &pos, const double range)
  {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    int step = ceil(range / mp_->resolution_);
    for (int x = -step; x <= step; ++x)
      for (int y = -step; y <= step; ++y)
        for (int z = -step; z <= step; ++z)
        {
          Eigen::Vector3i id1 = id + Eigen::Vector3i(x, y, z);
          if (getOccupancy(id1) == UNKNOWN)
            return true;
        }
    return false;
  }

  inline bool SDFMap::isNeighborOccupied(const Eigen::Vector3d &pos, const double range)
  {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    int step = ceil(range / mp_->resolution_);
    for (int x = -step; x <= step; ++x)
      for (int y = -step; y <= step; ++y)
        for (int z = -step; z <= step; ++z)
        {
          Eigen::Vector3i id1 = id + Eigen::Vector3i(x, y, z);
          if (getInflateOccupancy(id1) == 1)
            return true;
        }
    return false;
  }

  inline bool SDFMap::isFree(const Eigen::Vector3d &pos)
  {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return getOccupancy(id) == FREE;
  }

  inline bool SDFMap::isFree(const Eigen::Vector3i &id)
  {
    return getOccupancy(id) == FREE;
  }

  inline bool SDFMap::isUnknown(const Eigen::Vector3d &pos)
  {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return getOccupancy(id) == UNKNOWN;
  }

  inline bool SDFMap::isUnknown(const Eigen::Vector3i &id)
  {
    return getOccupancy(id) == UNKNOWN;
  }

  inline bool SDFMap::isOccupied(const Eigen::Vector3d &pos)
  {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return getOccupancy(id) == OCCUPIED;
  }

  inline bool SDFMap::isOccupied(const Eigen::Vector3i &id)
  {
    return getOccupancy(id) == OCCUPIED;
  }

  inline bool SDFMap::isInflatedFree(const Eigen::Vector3d &pos)
  {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return getInflateOccupancy(id) == 0 && getOccupancy(id) == FREE;
  }

  inline bool SDFMap::isInflatedFree(const Eigen::Vector3i &id)
  {
    return getInflateOccupancy(id) == 0 && getOccupancy(id) == FREE;
  }

  inline bool SDFMap::isInflatedUnknown(const Eigen::Vector3d &pos)
  {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return getInflateOccupancy(id) == 0 && getOccupancy(id) == UNKNOWN;
  }

  inline bool SDFMap::isInflatedUnknown(const Eigen::Vector3i &id)
  {
    return getInflateOccupancy(id) == 0 && getOccupancy(id) == UNKNOWN;
  }

  inline bool SDFMap::isInflatedOccupied(const Eigen::Vector3d &pos)
  {
    Eigen::Vector3i id;
    posToIndex(pos, id);
    return getInflateOccupancy(id) == 1;
  }

  inline bool SDFMap::isInflatedOccupied(const Eigen::Vector3i &id)
  {
    return getInflateOccupancy(id) == 1;
  }

  inline void SDFMap::resetChangeDetection()
  {
    md_->change_buffer_.clear();
  }

  inline void SDFMap::enableChangeDetection()
  {
    mp_->enable_change_detection_ = true;
  }

  inline void SDFMap::disableChangeDetection()
  {
    mp_->enable_change_detection_ = false;
  }

  inline std::vector<int>::iterator SDFMap::changesBegin()
  {
    return md_->change_buffer_.begin();
  }

  inline std::vector<int>::iterator SDFMap::changesEnd()
  {
    return md_->change_buffer_.end();
  }

  inline double SDFMap::getMaxRayLength()
  {
    return mp_->max_ray_length_;
  }

  inline double SDFMap::getInflateRadius()
  {
    return mp_->obstacles_inflation_;
  }

  inline void SDFMap::roundPosition(const Eigen::Vector3d &pos, Eigen::Vector3d &rounded_pos)
  {
    Eigen::Vector3i ijk;
    posToIndex(pos, ijk);
    indexToPos(ijk, rounded_pos, 0.5);
  }

  inline void SDFMap::inflatePoint(const Eigen::Vector3i &pt, int step,
                                   vector<Eigen::Vector3i> &pts)
  {
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
        for (int z = -step; z <= step; ++z)
        {
          pts[num++] = Eigen::Vector3i(pt(0) + x, pt(1) + y, pt(2) + z);
        }
  }
}
#endif
