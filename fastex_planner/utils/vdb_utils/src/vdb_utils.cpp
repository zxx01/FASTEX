#include <array>
#include <cmath>

#include "vdb_utils/vdb_utils.h"

namespace vdb_utils
{

void VDBUtil::setVoxelSize(openvdb::GridBase &grid, double vs, const openvdb::math::Vec3d &offset)
{
  openvdb::math::Transform::Ptr tf = openvdb::math::Transform::createLinearTransform(vs);
  tf->postTranslate(offset);
  grid.setTransform(tf);
}

void VDBUtil::setVoxelSize(openvdb::GridBase &grid, const openvdb::Vec3d &vs, const openvdb::math::Vec3d &offset)
{
  openvdb::math::Mat4d mat = openvdb::math::Mat4d::identity();
  mat.preScale(vs);
  openvdb::math::Transform::Ptr tf = openvdb::math::Transform::createLinearTransform(mat);
  tf->postTranslate(offset);
  grid.setTransform(tf);
}

void VDBUtil::getVoxelCenter(const openvdb::GridBase::Ptr &grid, const openvdb::math::Coord &coord, openvdb::Vec3d &center)
{
  const openvdb::math::Transform &grid_tf(grid->transform());
  center = grid_tf.indexToWorld(coord.asVec3d() + openvdb::Vec3d(0.5, 0.5, 0.5));
}

void VDBUtil::getVoxelBBoxd(const openvdb::GridBase::Ptr &grid, const openvdb::math::Coord &coord, openvdb::BBoxd &bboxd)
{
  const openvdb::math::Transform &grid_tf(grid->transform());
  openvdb::Vec3d min_world = grid_tf.indexToWorld(coord);
  openvdb::Vec3d max_world = grid_tf.indexToWorld(coord.offsetBy(1, 1, 1));
  bboxd = openvdb::BBoxd(min_world, max_world);
}

void VDBUtil::convertCoarseIndexToFineIndex(const int &from_level, const int &to_level,
                                            const openvdb::math::Coord &coarse_index,
                                            openvdb::math::CoordBBox &fine_index_box,
                                            const DIMENSION &dimensions)
{
  int scalar = std::pow(2, to_level - from_level);

  std::array<bool, 3> dims_to_convert = {false, false, false};
  switch (dimensions)
  {
  case DIMENSION::X:
    dims_to_convert[0] = true;
    break;
  case DIMENSION::Y:
    dims_to_convert[1] = true;
    break;
  case DIMENSION::Z:
    dims_to_convert[2] = true;
    break;
  case DIMENSION::XY:
    dims_to_convert[0] = true;
    dims_to_convert[1] = true;
    break;
  case DIMENSION::XZ:
    dims_to_convert[0] = true;
    dims_to_convert[2] = true;
    break;
  case DIMENSION::YZ:
    dims_to_convert[1] = true;
    dims_to_convert[2] = true;
    break;
  case DIMENSION::XYZ:
    dims_to_convert[0] = true;
    dims_to_convert[1] = true;
    dims_to_convert[2] = true;
    break;
  default:
    break;
  }

  int x = dims_to_convert[0] ? coarse_index.x() * scalar : coarse_index.x();
  int y = dims_to_convert[1] ? coarse_index.y() * scalar : coarse_index.y();
  int z = dims_to_convert[2] ? coarse_index.z() * scalar : coarse_index.z();

  openvdb::math::Coord start_coord(x, y, z);
  openvdb::math::Coord end_coord(dims_to_convert[0] ? (x + scalar - 1) : x,
                                 dims_to_convert[1] ? (y + scalar - 1) : y,
                                 dims_to_convert[2] ? (z + scalar - 1) : z);

  fine_index_box.reset(start_coord, end_coord);
}

void VDBUtil::convertFineIndexToCoarseIndex(const int &from_level, const int &to_level,
                                            const openvdb::math::Coord &fine_index,
                                            openvdb::math::Coord &coarse_index,
                                            const DIMENSION &dimensions)
{
  int scalar = std::pow(2, from_level - to_level);

  std::array<bool, 3> dims_to_convert = {false, false, false};
  switch (dimensions)
  {
  case DIMENSION::X:
    dims_to_convert[0] = true;
    break;
  case DIMENSION::Y:
    dims_to_convert[1] = true;
    break;
  case DIMENSION::Z:
    dims_to_convert[2] = true;
    break;
  case DIMENSION::XY:
    dims_to_convert[0] = true;
    dims_to_convert[1] = true;
    break;
  case DIMENSION::XZ:
    dims_to_convert[0] = true;
    dims_to_convert[2] = true;
    break;
  case DIMENSION::YZ:
    dims_to_convert[1] = true;
    dims_to_convert[2] = true;
    break;
  case DIMENSION::XYZ:
    dims_to_convert[0] = true;
    dims_to_convert[1] = true;
    dims_to_convert[2] = true;
    break;
  default:
    break;
  }

  int x = dims_to_convert[0] ? std::floor(static_cast<float>(fine_index.x()) / scalar) : fine_index.x();
  int y = dims_to_convert[1] ? std::floor(static_cast<float>(fine_index.y()) / scalar) : fine_index.y();
  int z = dims_to_convert[2] ? std::floor(static_cast<float>(fine_index.z()) / scalar) : fine_index.z();

  coarse_index = openvdb::math::Coord(x, y, z);
}

void VDBUtil::convertBBoxdToCoordBox(const openvdb::GridBase::Ptr &grid, const openvdb::BBoxd &bboxd,
                                     const bool is_needed_pruned, openvdb::math::CoordBBox &cbbox)
{
  const openvdb::math::Transform &grid_tf(grid->transform());
  cbbox = grid_tf.worldToIndexNodeCentered(bboxd);

  if (is_needed_pruned)
  {
    cbbox.max() -= openvdb::Coord(1, 1, 1);
  }
}

void VDBUtil::getCoordInCoordBBox(const openvdb::math::CoordBBox &cbbox, std::vector<openvdb::math::Coord> &coords)
{
  for (auto coord_iter = cbbox.beginXYZ(); coord_iter; ++coord_iter)
  {
    coords.push_back(*coord_iter);
  }
}

template <>
openvdb::Vec3d VDBUtil::convertEigenVecToOpenVDBVec<Eigen::Vector3d, openvdb::Vec3d>(const Eigen::Vector3d &input_vec)
{
  return openvdb::Vec3d(input_vec.x(), input_vec.y(), input_vec.z());
}

template <>
openvdb::math::Coord VDBUtil::convertEigenVecToOpenVDBVec<Eigen::Vector3i, openvdb::math::Coord>(
    const Eigen::Vector3i &input_vec)
{
  return openvdb::math::Coord(input_vec.x(), input_vec.y(), input_vec.z());
}

template <>
Eigen::Vector3d VDBUtil::convertOpenVDBVecToEigenVec<openvdb::Vec3d, Eigen::Vector3d>(const openvdb::Vec3d &input_vec)
{
  return Eigen::Vector3d(input_vec.x(), input_vec.y(), input_vec.z());
}

template <>
Eigen::Vector3i VDBUtil::convertOpenVDBVecToEigenVec<openvdb::math::Coord, Eigen::Vector3i>(
    const openvdb::math::Coord &input_vec)
{
  return Eigen::Vector3i(input_vec.x(), input_vec.y(), input_vec.z());
}

} // namespace vdb_utils
