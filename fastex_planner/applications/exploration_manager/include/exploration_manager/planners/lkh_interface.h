/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-05-30 21:29:33
 * @LastEditTime: 2026-05-30 21:57:28
 * @Description:
 */

#ifndef _LKH_INTERFACE_H_
#define _LKH_INTERFACE_H_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "lkh_solver/lkh_solver.h"

namespace fastex_explorer
{
class LkhInterface
{
  public:
    using UniquePtr = std::unique_ptr<LkhInterface>;

    LkhInterface();
    ~LkhInterface() = default;

    bool solveATSP(const std::string& file_name, const Eigen::MatrixXd& matrix,
                   std::vector<int>& optimal_indices);
    bool solveSOP(const std::string& file_name, const Eigen::MatrixXd& matrix,
                  std::vector<int>& optimal_indices);

  private:
    void writeATSPFiles(const std::string& file_name, const Eigen::MatrixXd& matrix) const;
    void writeSOPFiles(const std::string& file_name, const Eigen::MatrixXd& matrix) const;
    bool readOptimalTour(const std::string& file_name, std::vector<int>& optimal_indices) const;

    lkh_solver::LKHSlover::UniquePtr solver_;
};
} // namespace fastex_explorer

#endif // _LKH_INTERFACE_H_
