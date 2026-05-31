/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-05-05 21:27:48
 * @LastEditTime: 2024-09-06 15:54:44
 * @Description:
 */

#ifndef _EIGEN_UTILS_H_
#define _EIGEN_UTILS_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <list>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <sstream>

#include <cmath>      // For std::round
#include <functional> // For std::hash
#include <numeric>    // For std::accumulate

namespace eigen_utils
{
  /// alias for eigen
  using Vec2i = Eigen::Vector2i;
  using Vec3i = Eigen::Vector3i;
  using Vec4i = Eigen::Vector4i;

  using Vec2d = Eigen::Vector2d;
  using Vec2f = Eigen::Vector2f;
  using Vec3d = Eigen::Vector3d;
  using Vec3f = Eigen::Vector3f;
  using Vec4d = Eigen::Vector4d;
  using Vec4f = Eigen::Vector4f;
  using Vec5d = Eigen::Matrix<double, 5, 1>;
  using Vec5f = Eigen::Matrix<float, 5, 1>;
  using Vec6d = Eigen::Matrix<double, 6, 1>;
  using Vec6f = Eigen::Matrix<float, 6, 1>;
  using Vec15d = Eigen::Matrix<double, 15, 15>;

  using Mat1d = Eigen::Matrix<double, 1, 1>;
  using Mat1f = Eigen::Matrix<float, 1, 1>;
  using Mat2d = Eigen::Matrix2d;
  using Mat2f = Eigen::Matrix2f;
  using Mat3d = Eigen::Matrix3d;
  using Mat3f = Eigen::Matrix3f;
  using Mat4d = Eigen::Matrix4d;
  using Mat4f = Eigen::Matrix4f;
  using Mat5d = Eigen::Matrix<double, 5, 5>;
  using Mat5f = Eigen::Matrix<float, 5, 5>;
  using Mat6d = Eigen::Matrix<double, 6, 6>;
  using Mat6f = Eigen::Matrix<float, 6, 6>;
  using Mat15d = Eigen::Matrix<double, 15, 15>;

  using Quatd = Eigen::Quaterniond;
  using Quatf = Eigen::Quaternionf;

  /// less of vector
  template <int N>
  struct less_veci
  {
    inline bool operator()(const Eigen::Matrix<int, N, 1> &vec1, const Eigen::Matrix<int, N, 1> &vec2) const;
  };

  /// equal of vector
  template <int N>
  struct equal_veci
  {
    inline bool operator()(const Eigen::Matrix<int, N, 1> &vec1, const Eigen::Matrix<int, N, 1> &vec2) const
    {
      return vec1 == vec2;
    }
  };

  template <int N, int Precision = -1>
  struct equal_vecd
  {
    inline bool operator()(const Eigen::Matrix<double, N, 1> &vec1, const Eigen::Matrix<double, N, 1> &vec2) const
    {
      if (Precision < 0)
        return vec1.isApprox(vec2, 1e-8);
      else
      {
        for (int i = 0; i < N; ++i)
        {
          if (std::abs(vec1[i] - vec2[i]) > std::pow(10.0, -Precision))
            return false;
        }
        return true;
      }
    }
  };

  template <int N, int Precision = -1>
  struct equal_vecf
  {
    inline bool operator()(const Eigen::Matrix<float, N, 1> &vec1, const Eigen::Matrix<float, N, 1> &vec2) const
    {
      if (Precision < 0)
        return vec1.isApprox(vec2, 1e-6);
      else
      {
        for (int i = 0; i < N; ++i)
        {
          if (std::abs(vec1[i] - vec2[i]) > std::pow(10.0f, -Precision))
            return false;
        }
        return true;
      }
    }
  };

  /// hash of vector
  template <int N>
  struct hash_veci
  {
    inline size_t operator()(const Eigen::Matrix<int, N, 1> &vec) const;
  };

  template <int N, int Precision = -1>
  struct hash_vecd
  {
    double factor = 1.0;

    hash_vecd()
    {
      if (Precision >= 0)
        factor = std::pow(10.0, Precision);
    }

    size_t operator()(const Eigen::Matrix<double, N, 1> &vec) const
    {
      if (Precision < 0)
      {
        std::hash<double> hasher;
        return std::accumulate(vec.data(), vec.data() + N, size_t(0x9e3779b9), [&](size_t seed, double val)
                               {
                size_t h = hasher(val);
                return seed ^ (h + 0x9e3779b9 + (seed << 6) + (seed >> 2)); });
      }
      else
      {
        std::hash<long long> hasher;
        return std::accumulate(vec.data(), vec.data() + N, size_t(0x9e3779b9), [&](size_t seed, double val)
                               {
                long long val_rounded = static_cast<long long>(std::round(val * factor));
                size_t h = hasher(val_rounded);
                return seed ^ (h + 0x9e3779b9 + (seed << 6) + (seed >> 2)); });
      }
    }
  };

  template <int N, int Precision = -1>
  struct hash_vecf
  {
    float factor = 1.0f;

    hash_vecf()
    {
      if (Precision >= 0)
        factor = std::pow(10.0f, Precision);
    }

    size_t operator()(const Eigen::Matrix<float, N, 1> &vec) const
    {
      if (Precision < 0)
      {
        std::hash<float> hasher;
        return std::accumulate(vec.data(), vec.data() + N, size_t(0x9e3779b9), [&](size_t seed, float val)
                               {
                size_t h = hasher(val);
                return seed ^ (h + 0x9e3779b9 + (seed << 6) + (seed >> 2)); });
      }
      else
      {
        std::hash<int> hasher;
        return std::accumulate(vec.data(), vec.data() + N, size_t(0x9e3779b9), [&](size_t seed, float val)
                               {
                int val_rounded = static_cast<int>(std::round(val * factor));
                size_t h = hasher(val_rounded);
                return seed ^ (h + 0x9e3779b9 + (seed << 6) + (seed >> 2)); });
      }
    }
  };

  /// implementation
  template <>
  inline bool less_veci<2>::operator()(const Eigen::Matrix<int, 2, 1> &vec1, const Eigen::Matrix<int, 2, 1> &vec2) const
  {
    return vec1[0] < vec2[0] || (vec1[0] == vec2[0] && vec1[1] < vec2[1]);
  }

  template <>
  inline bool less_veci<3>::operator()(const Eigen::Matrix<int, 3, 1> &vec1, const Eigen::Matrix<int, 3, 1> &vec2) const
  {
    return (vec1[0] < vec2[0] || (vec1[0] == vec2[0] && vec1[1] < vec2[1])) && (vec1[0] == vec2[0] && vec1[1] == vec2[1] && vec1[2] < vec2[2]);
  }

  /// vec 2 hash
  /// @see Optimized Spatial Hashing for Collision Detection of Deformable Objects, Matthias Teschner et. al., VMV 2003
  template <>
  inline size_t hash_veci<2>::operator()(const Eigen::Matrix<int, 2, 1> &vec) const
  {
    return size_t(((vec[0]) * 73856093) ^ ((vec[1]) * 471943)) % 10000000;
  }

  /// vec 3 hash
  template <>
  inline size_t hash_veci<3>::operator()(const Eigen::Matrix<int, 3, 1> &vec) const
  {
    return size_t(((vec[0]) * 73856093) ^ ((vec[1]) * 471943) ^ ((vec[2]) * 83492791)) % 10000000;
  }

  // Vector to string
  template <typename Derived>
  std::string vec_to_string(const Eigen::MatrixBase<Derived> &vec)
  {
    std::stringstream ss;
    ss << "(";
    for (int i = 0; i < vec.size(); ++i)
    {
      ss << vec[i];
      if (i < vec.size() - 1)
        ss << ", ";
    }
    ss << ")";
    return ss.str();
  }

  // Vector for Vec
  using Vec_Vec2i = std::vector<Vec2i>;
  using Vec_Vec3i = std::vector<Vec3i>;
  using Vec_Vec4i = std::vector<Vec4i>;
  using Vec_Vec2f = std::vector<Vec2f>;
  using Vec_Vec3f = std::vector<Vec3f>;
  using Vec_Vec4f = std::vector<Vec4f>;
  using Vec_Vec2d = std::vector<Vec2d>;
  using Vec_Vec3d = std::vector<Vec3d>;
  using Vec_Vec4d = std::vector<Vec4d>;

  // List for Vec
  using List_Vec2i = std::list<Vec2i>;
  using List_Vec3i = std::list<Vec3i>;
  using List_Vec4i = std::list<Vec4i>;
  using List_Vec2f = std::list<Vec2f>;
  using List_Vec3f = std::list<Vec3f>;
  using List_Vec4f = std::list<Vec4f>;
  using List_Vec2d = std::list<Vec2d>;
  using List_Vec3d = std::list<Vec3d>;
  using List_Vec4d = std::list<Vec4d>;

  // unordered_map for Vec
  template <typename T>
  using Vec3iMap = std::unordered_map<eigen_utils::Vec3i, T, eigen_utils::hash_veci<3>, eigen_utils::equal_veci<3>>;
  template <typename T, int Precision = -1>
  using Vec3dMap = std::unordered_map<eigen_utils::Vec3d, T, eigen_utils::hash_vecd<3, Precision>, eigen_utils::equal_vecd<3, Precision>>;
  template <typename T, int Precision = -1>
  using Vec3fMap = std::unordered_map<eigen_utils::Vec3f, T, eigen_utils::hash_vecf<3, Precision>, eigen_utils::equal_vecf<3, Precision>>;

  // unordered_set for Vec
  using Vec3iSet = std::unordered_set<eigen_utils::Vec3i, eigen_utils::hash_veci<3>, eigen_utils::equal_veci<3>>;
  template <int Precision = -1>
  using Vec3dSet = std::unordered_set<eigen_utils::Vec3d, eigen_utils::hash_vecd<3, Precision>, eigen_utils::equal_vecd<3, Precision>>;
  template <int Precision = -1>
  using Vec3fSet = std::unordered_set<eigen_utils::Vec3f, eigen_utils::hash_vecf<3, Precision>, eigen_utils::equal_vecf<3, Precision>>;

} // namespace eigen_utils

#endif