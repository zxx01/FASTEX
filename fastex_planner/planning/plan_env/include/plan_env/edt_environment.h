#ifndef _EDT_ENVIRONMENT_H_
#define _EDT_ENVIRONMENT_H_

#include <Eigen/Eigen>
#include <iostream>
#include <memory>
#include <utility>

using std::cout;
using std::endl;
using std::list;
using std::pair;
using std::shared_ptr;
using std::vector;

namespace fast_planner
{
  class SDFMap;

  class EDTEnvironment
  {
  private:
    /* data */
    double resolution_inv_;

  public:
    EDTEnvironment(/* args */)
    {
    }
    ~EDTEnvironment()
    {
    }

    shared_ptr<SDFMap> sdf_map_;

    void init();
    void setMap(shared_ptr<SDFMap> &map);
    void evaluateEDTWithGrad(const Eigen::Vector3d &pos, double &dist, Eigen::Vector3d &grad);
    double evaluateCoarseEDT(Eigen::Vector3d &pos);

    // deprecated
    void getSurroundDistance(Eigen::Vector3d pts[2][2][2], double dists[2][2][2]);
    void interpolateTrilinear(double values[2][2][2], const Eigen::Vector3d &diff, double &value,
                              Eigen::Vector3d &grad);

    typedef shared_ptr<EDTEnvironment> Ptr;
  };

} // namespace fast_planner

#endif
