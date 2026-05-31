#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <bspline/non_uniform_bspline.h>
#include <iostream>
#include <poly_traj/polynomial_traj.h>
#include <ros/ros.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>

using std::vector;
namespace fast_planner
{
  class PlanningVisualization
  {
  private:
    enum TRAJECTORY_PLANNING_ID
    {
      GOAL = 1,
      PATH = 200,
      BSPLINE = 300,
      BSPLINE_CTRL_PT = 400,
      POLY_TRAJ = 500
    };

    /* data */
    ros::NodeHandle node;
    ros::Publisher traj_pub_;              // 0
    ros::Publisher topo_pub_;              // 1
    ros::Publisher frontier_pub_;          // 4, frontier searching
    ros::Publisher yaw_pub_;               // 5, yaw trajectory
    ros::Publisher viewpoint_pub_;         // 6, viewpoint planning
    ros::Publisher dynamic_expanding_grid_pub_; // 7, dynamic expanding grid
    vector<ros::Publisher> pubs_;          //

    int last_bspline_phase1_num_;
    int last_bspline_phase2_num_;
    int last_frontier_num_;

  public:
    PlanningVisualization(/* args */)
    {
    }
    ~PlanningVisualization()
    {
    }
    PlanningVisualization(ros::NodeHandle &nh);

    // new interface
    void fillBasicInfo(visualization_msgs::Marker &mk, const Eigen::Vector3d &scale,
                       const Eigen::Vector4d &color, const string &ns, const int &id, const int &shape);
    void fillGeometryInfo(visualization_msgs::Marker &mk, const vector<Eigen::Vector3d> &list);
    void fillGeometryInfo(visualization_msgs::Marker &mk, const vector<Eigen::Vector3d> &list1,
                          const vector<Eigen::Vector3d> &list2);

    void drawSpheres(const vector<Eigen::Vector3d> &list, const double &scale,
                     const Eigen::Vector4d &color, const string &ns, const int &id, const int &pub_id);
    void drawCubes(const vector<Eigen::Vector3d> &list, const double &scale, const Eigen::Vector4d &color,
                   const string &ns, const int &id, const int &pub_id);
    void drawLines(const vector<Eigen::Vector3d> &list1, const vector<Eigen::Vector3d> &list2,
                   const double &scale, const Eigen::Vector4d &color, const string &ns, const int &id,
                   const int &pub_id);
    void drawLines(const vector<Eigen::Vector3d> &list, const double &scale, const Eigen::Vector4d &color,
                   const string &ns, const int &id, const int &pub_id);
    void drawBox(const Eigen::Vector3d &center, const Eigen::Vector3d &scale, const Eigen::Vector4d &color,
                 const string &ns, const int &id, const int &pub_id);

    void drawPose(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir, const double &scale,
                  const Eigen::Vector4d &color, const string &ns, const int &id, const int &pub_id);

    // Deprecated
    // draw basic shapes
    void displaySphereList(const vector<Eigen::Vector3d> &list, double resolution,
                           const Eigen::Vector4d &color, int id, int pub_id = 0);
    void displayCubeList(const vector<Eigen::Vector3d> &list, double resolution,
                         const Eigen::Vector4d &color, int id, int pub_id = 0);
    void displayLineList(const vector<Eigen::Vector3d> &list1, const vector<Eigen::Vector3d> &list2,
                         double line_width, const Eigen::Vector4d &color, int id, int pub_id = 0);
    // draw a piece-wise straight line path
    void drawGeometricPath(const vector<Eigen::Vector3d> &path, double resolution,
                           const Eigen::Vector4d &color, int id = 0);
    // draw a polynomial trajectory
    void drawPolynomialTraj(PolynomialTraj poly_traj, double resolution, const Eigen::Vector4d &color,
                            int id = 0);
    // draw a bspline trajectory
    void drawBspline(NonUniformBspline &bspline, double size, const Eigen::Vector4d &color,
                     bool show_ctrl_pts = false, double size2 = 0.1,
                     const Eigen::Vector4d &color2 = Eigen::Vector4d(1, 1, 0, 1), int id = 0);
    // draw a set of bspline trajectories generated in different phases
    void drawBsplinesPhase1(vector<NonUniformBspline> &bsplines, double size);
    void drawBsplinesPhase2(vector<NonUniformBspline> &bsplines, double size);
    void drawGoal(Eigen::Vector3d goal, double resolution, const Eigen::Vector4d &color, int id = 0);
    Eigen::Vector4d getColor(const double &h, double alpha = 1.0);

    typedef std::shared_ptr<PlanningVisualization> Ptr;

    // SECTION developing
    void drawFrontier(const vector<vector<Eigen::Vector3d>> &frontiers);
    void drawYawTraj(NonUniformBspline &pos, NonUniformBspline &yaw, const double &dt);
    void drawYawPath(NonUniformBspline &pos, const vector<double> &yaw, const double &dt);
  };
} // namespace fast_planner
#endif
