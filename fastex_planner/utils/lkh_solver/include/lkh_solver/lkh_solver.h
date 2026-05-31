/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-10-09 16:58:52
 * @LastEditTime: 2026-05-30 21:58:03
 * @Description:
 */

#include <memory>
#include <string>

#ifndef _LKH_SOLVER_H_
#define _LKH_SOLVER_H_

namespace lkh_solver
{
class LKHSlover
{
  public:
    using SharedPtr = std::shared_ptr<LKHSlover>;
    using UniquePtr = std::unique_ptr<LKHSlover>;

    LKHSlover();
    ~LKHSlover();

    bool solveProblem(const std::string& par_file);

  private:
    std::string lkh_executable_;
};
} // namespace lkh_solver

#endif // ! _LKH_SOLVER_H_