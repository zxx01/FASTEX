#ifndef _VDB_HELPER_H_
#define _VDB_HELPER_H_

#include <vector>

#include <eigen3/Eigen/Core>

#include <openvdb/openvdb.h>
#include <openvdb/math/DDA.h>
#include <openvdb/math/Ray.h>

namespace vdb_utils
{
enum class DIMENSION
{
  X = 0,
  Y,
  Z,
  XY,
  YZ,
  XZ,
  XYZ
};

class VDBUtil
{
public:
  static void setVoxelSize(openvdb::GridBase &grid, double vs,
                           const openvdb::math::Vec3d &offset = openvdb::math::Vec3d(0.0, 0.0, 0.0));

  static void setVoxelSize(openvdb::GridBase &grid, const openvdb::Vec3d &vs,
                           const openvdb::math::Vec3d &offset = openvdb::math::Vec3d(0.0, 0.0, 0.0));

  static void getVoxelCenter(const openvdb::GridBase::Ptr &grid, const openvdb::math::Coord &coord,
                             openvdb::Vec3d &center);

  static void getVoxelBBoxd(const openvdb::GridBase::Ptr &grid, const openvdb::math::Coord &coord,
                            openvdb::BBoxd &bboxd);

  static void convertCoarseIndexToFineIndex(const int &from_level, const int &to_level,
                                            const openvdb::math::Coord &coarse_index,
                                            openvdb::math::CoordBBox &fine_index_box,
                                            const DIMENSION &dimensions = DIMENSION::XYZ);

  static void convertFineIndexToCoarseIndex(const int &from_level, const int &to_level,
                                            const openvdb::math::Coord &fine_index,
                                            openvdb::math::Coord &coarse_index,
                                            const DIMENSION &dimensions = DIMENSION::XYZ);

  static void convertBBoxdToCoordBox(const openvdb::GridBase::Ptr &grid, const openvdb::BBoxd &bboxd,
                                     const bool is_needed_pruned, openvdb::math::CoordBBox &cbbox);

  static void getCoordInCoordBBox(const openvdb::math::CoordBBox &cbbox,
                                  std::vector<openvdb::math::Coord> &coords);

  template <typename EigenVecType, typename OpenVDBVecType>
  static OpenVDBVecType convertEigenVecToOpenVDBVec(const EigenVecType &input_vec);

  template <typename OpenVDBVecType, typename EigenVecType>
  static EigenVecType convertOpenVDBVecToEigenVec(const OpenVDBVecType &input_vec);
};

} // namespace vdb_utils

#endif
