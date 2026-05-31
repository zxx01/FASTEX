#include <traj_utils/planning_visualization.h>
#include <vis_utils/marker_utils.h>

using std::cout;
using std::endl;
namespace fast_planner
{
  PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh)
  {
    node = nh;

    traj_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/trajectory", 100);
    pubs_.push_back(traj_pub_);

    topo_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/topo_path", 100);
    pubs_.push_back(topo_pub_);

    pubs_.push_back(ros::Publisher());
    pubs_.push_back(ros::Publisher());

    frontier_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/frontier", 10000);
    pubs_.push_back(frontier_pub_);

    yaw_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/yaw", 100);
    pubs_.push_back(yaw_pub_);

    viewpoint_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/viewpoints", 1000);
    pubs_.push_back(viewpoint_pub_);

    dynamic_expanding_grid_pub_ =
        node.advertise<visualization_msgs::Marker>("/planning_vis/dynamic_expanding_grid", 100);
    pubs_.push_back(dynamic_expanding_grid_pub_);

    last_bspline_phase1_num_ = 0;
    last_bspline_phase2_num_ = 0;
    last_frontier_num_ = 0;
  }

  void PlanningVisualization::fillBasicInfo(visualization_msgs::Marker &mk, const Eigen::Vector3d &scale,
                                            const Eigen::Vector4d &color, const string &ns, const int &id,
                                            const int &shape)
  {
    vis_utils::marker_utils::configureMarker(
        mk, "world", ns, id, shape,
        vis_utils::marker_utils::makeScale(scale[0], scale[1], scale[2]),
        vis_utils::marker_utils::toRosColor(color));
  }

  void PlanningVisualization::fillGeometryInfo(visualization_msgs::Marker &mk,
                                               const vector<Eigen::Vector3d> &list)
  {
    vis_utils::marker_utils::appendPoints(mk, list);
  }

  void PlanningVisualization::fillGeometryInfo(visualization_msgs::Marker &mk,
                                               const vector<Eigen::Vector3d> &list1,
                                               const vector<Eigen::Vector3d> &list2)
  {
    for (int i = 0; i < int(list1.size()); ++i)
    {
      vis_utils::marker_utils::appendLine(mk, list1[i], list2[i]);
    }
  }

  void PlanningVisualization::drawBox(const Eigen::Vector3d &center, const Eigen::Vector3d &scale,
                                      const Eigen::Vector4d &color, const string &ns, const int &id,
                                      const int &pub_id)
  {
    visualization_msgs::Marker mk;
    fillBasicInfo(mk, scale, color, ns, id, visualization_msgs::Marker::CUBE);
    mk.action = visualization_msgs::Marker::DELETE;
    pubs_[pub_id].publish(mk);

    mk.pose.position.x = center[0];
    mk.pose.position.y = center[1];
    mk.pose.position.z = center[2];
    mk.action = visualization_msgs::Marker::ADD;

    pubs_[pub_id].publish(mk);
    ros::Duration(0.0005).sleep();
  }

  void PlanningVisualization::drawPose(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir, const double &scale,
                                       const Eigen::Vector4d &color, const string &ns, const int &id, const int &pub_id)
  {
    visualization_msgs::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(scale, scale * 2, scale * 2), color, ns, id,
                  visualization_msgs::Marker::ARROW);

    // set start and goal points
    geometry_msgs::Point start_point;
    start_point.x = pos.x();
    start_point.y = pos.y();
    start_point.z = pos.z();

    geometry_msgs::Point end_point;
    end_point.x = pos.x() + dir.x();
    end_point.y = pos.y() + dir.y();
    end_point.z = pos.z() + dir.z();

    mk.points.push_back(start_point);
    mk.points.push_back(end_point);

    mk.action = visualization_msgs::Marker::ADD;
    pubs_[pub_id].publish(mk);
    ros::Duration(0.0005).sleep();
  }

  void PlanningVisualization::drawSpheres(const vector<Eigen::Vector3d> &list, const double &scale,
                                          const Eigen::Vector4d &color, const string &ns, const int &id,
                                          const int &pub_id)
  {
    visualization_msgs::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                  visualization_msgs::Marker::SPHERE_LIST);

    // clean old marker
    mk.action = visualization_msgs::Marker::DELETE;
    pubs_[pub_id].publish(mk);

    // pub new marker
    fillGeometryInfo(mk, list);
    mk.action = visualization_msgs::Marker::ADD;
    pubs_[pub_id].publish(mk);
    ros::Duration(0.0005).sleep();
  }

  void PlanningVisualization::drawCubes(const vector<Eigen::Vector3d> &list, const double &scale,
                                        const Eigen::Vector4d &color, const string &ns, const int &id,
                                        const int &pub_id)
  {
    visualization_msgs::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                  visualization_msgs::Marker::CUBE_LIST);

    // clean old marker
    mk.action = visualization_msgs::Marker::DELETE;
    pubs_[pub_id].publish(mk);

    // pub new marker
    fillGeometryInfo(mk, list);
    mk.action = visualization_msgs::Marker::ADD;
    pubs_[pub_id].publish(mk);
    ros::Duration(0.0005).sleep();
  }

  void PlanningVisualization::drawLines(const vector<Eigen::Vector3d> &list1,
                                        const vector<Eigen::Vector3d> &list2, const double &scale,
                                        const Eigen::Vector4d &color, const string &ns, const int &id,
                                        const int &pub_id)
  {
    visualization_msgs::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                  visualization_msgs::Marker::LINE_LIST);

    // clean old marker
    mk.action = visualization_msgs::Marker::DELETE;
    pubs_[pub_id].publish(mk);

    if (list1.size() == 0)
      return;

    // pub new marker
    fillGeometryInfo(mk, list1, list2);
    mk.action = visualization_msgs::Marker::ADD;
    pubs_[pub_id].publish(mk);
    ros::Duration(0.0005).sleep();
  }

  void PlanningVisualization::drawLines(const vector<Eigen::Vector3d> &list, const double &scale,
                                        const Eigen::Vector4d &color, const string &ns, const int &id,
                                        const int &pub_id)
  {
    visualization_msgs::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                  visualization_msgs::Marker::LINE_LIST);

    // clean old marker
    mk.action = visualization_msgs::Marker::DELETE;
    pubs_[pub_id].publish(mk);

    if (list.size() == 0)
      return;

    // split the single list into two
    vector<Eigen::Vector3d> list1, list2;
    for (int i = 0; i < list.size() - 1; ++i)
    {
      list1.push_back(list[i]);
      list2.push_back(list[i + 1]);
    }

    // pub new marker
    fillGeometryInfo(mk, list1, list2);
    mk.action = visualization_msgs::Marker::ADD;
    pubs_[pub_id].publish(mk);
    ros::Duration(0.0005).sleep();
  }

  void PlanningVisualization::displaySphereList(const vector<Eigen::Vector3d> &list, double resolution,
                                                const Eigen::Vector4d &color, int id, int pub_id)
  {
    auto scale = vis_utils::marker_utils::makeScale(resolution, resolution, resolution);
    auto ros_color = vis_utils::marker_utils::toRosColor(color);

    // DELETE old marker
    auto mk_del = vis_utils::marker_utils::makeSphereListMarker(
        "world", "", id, scale, ros_color, visualization_msgs::Marker::DELETE);
    pubs_[pub_id].publish(mk_del);

    // ADD new marker
    auto mk = vis_utils::marker_utils::makeSphereListMarker("world", "", id, scale, ros_color);
    vis_utils::marker_utils::appendPoints(mk, list);
    pubs_[pub_id].publish(mk);
    ros::Duration(0.0005).sleep();
  }

  void PlanningVisualization::displayCubeList(const vector<Eigen::Vector3d> &list, double resolution,
                                              const Eigen::Vector4d &color, int id, int pub_id)
  {
    auto scale = vis_utils::marker_utils::makeScale(resolution, resolution, resolution);
    auto ros_color = vis_utils::marker_utils::toRosColor(color);

    // DELETE old marker
    auto mk_del = vis_utils::marker_utils::makeMarker(
        "world", "", id, visualization_msgs::Marker::CUBE_LIST, scale, ros_color,
        visualization_msgs::Marker::DELETE);
    pubs_[pub_id].publish(mk_del);

    // ADD new marker
    auto mk = vis_utils::marker_utils::makeMarker(
        "world", "", id, visualization_msgs::Marker::CUBE_LIST, scale, ros_color);
    vis_utils::marker_utils::appendPoints(mk, list);
    pubs_[pub_id].publish(mk);
    ros::Duration(0.0005).sleep();
  }

  void PlanningVisualization::displayLineList(const vector<Eigen::Vector3d> &list1,
                                              const vector<Eigen::Vector3d> &list2, double line_width,
                                              const Eigen::Vector4d &color, int id, int pub_id)
  {
    auto ros_color = vis_utils::marker_utils::toRosColor(color);

    // DELETE old marker
    auto mk_del = vis_utils::marker_utils::makeLineListMarker(
        "world", "", id, line_width, ros_color, visualization_msgs::Marker::DELETE);
    pubs_[pub_id].publish(mk_del);

    // ADD new marker
    auto mk = vis_utils::marker_utils::makeLineListMarker(
        "world", "", id, line_width, ros_color);
    for (int i = 0; i < int(list1.size()); ++i)
    {
      vis_utils::marker_utils::appendLine(mk, list1[i], list2[i]);
    }
    pubs_[pub_id].publish(mk);
    ros::Duration(0.0005).sleep();
  }

  void PlanningVisualization::drawBsplinesPhase1(vector<NonUniformBspline> &bsplines, double size)
  {
    vector<Eigen::Vector3d> empty;

    for (int i = 0; i < last_bspline_phase1_num_; ++i)
    {
      displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE + i % 100);
      displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE_CTRL_PT + i % 100);
    }
    last_bspline_phase1_num_ = bsplines.size();

    for (int i = 0; i < bsplines.size(); ++i)
    {
      drawBspline(bsplines[i], size, getColor(double(i) / bsplines.size(), 0.2), false, 2 * size,
                  getColor(double(i) / bsplines.size()), i);
    }
  }

  void PlanningVisualization::drawBsplinesPhase2(vector<NonUniformBspline> &bsplines, double size)
  {
    vector<Eigen::Vector3d> empty;

    for (int i = 0; i < last_bspline_phase2_num_; ++i)
    {
      drawSpheres(empty, size, Eigen::Vector4d(1, 0, 0, 1), "B-Spline", i, 0);
      drawSpheres(empty, size, Eigen::Vector4d(1, 0, 0, 1), "B-Spline", i + 50, 0);
      // displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE + (50 + i) % 100);
      // displaySphereList(empty, size, Eigen::Vector4d(1, 0, 0, 1), BSPLINE_CTRL_PT + (50 + i) % 100);
    }
    last_bspline_phase2_num_ = bsplines.size();

    for (int i = 0; i < bsplines.size(); ++i)
    {
      drawBspline(bsplines[i], size, getColor(double(i) / bsplines.size(), 0.6), false, 1.5 * size,
                  getColor(double(i) / bsplines.size()), i);
    }
  }

  void PlanningVisualization::drawBspline(NonUniformBspline &bspline, double size,
                                          const Eigen::Vector4d &color, bool show_ctrl_pts, double size2,
                                          const Eigen::Vector4d &color2, int id1)
  {
    if (bspline.getControlPoint().size() == 0)
      return;

    vector<Eigen::Vector3d> traj_pts;
    double tm, tmp;
    bspline.getTimeSpan(tm, tmp);

    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::Vector3d pt = bspline.evaluateDeBoor(t);
      traj_pts.push_back(pt);
    }
    // displaySphereList(traj_pts, size, color, BSPLINE + id1 % 100);
    drawSpheres(traj_pts, size, color, "B-Spline", id1, 0);

    // draw the control point
    if (show_ctrl_pts)
    {
      Eigen::MatrixXd ctrl_pts = bspline.getControlPoint();
      vector<Eigen::Vector3d> ctp;
      for (int i = 0; i < int(ctrl_pts.rows()); ++i)
      {
        Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
        ctp.push_back(pt);
      }
      // displaySphereList(ctp, size2, color2, BSPLINE_CTRL_PT + id2 % 100);
      drawSpheres(ctp, size2, color2, "B-Spline", id1 + 50, 0);
    }
  }

  void PlanningVisualization::drawGoal(Eigen::Vector3d goal, double resolution,
                                       const Eigen::Vector4d &color, int id)
  {
    vector<Eigen::Vector3d> goal_vec = {goal};
    displaySphereList(goal_vec, resolution, color, GOAL + id % 100);
  }

  void PlanningVisualization::drawGeometricPath(const vector<Eigen::Vector3d> &path, double resolution,
                                                const Eigen::Vector4d &color, int id)
  {
    displaySphereList(path, resolution, color, PATH + id % 100);
  }

  void PlanningVisualization::drawPolynomialTraj(PolynomialTraj poly_traj, double resolution,
                                                 const Eigen::Vector4d &color, int id)
  {
    vector<Eigen::Vector3d> poly_pts;
    poly_traj.getSamplePoints(poly_pts);
    displaySphereList(poly_pts, resolution, color, POLY_TRAJ + id % 100);
  }

  void PlanningVisualization::drawFrontier(const vector<vector<Eigen::Vector3d>> &frontiers)
  {
    for (int i = 0; i < frontiers.size(); ++i)
    {
      // displayCubeList(frontiers[i], 0.1, getColor(double(i) / frontiers.size(),
      // 0.4), i, 4);
      drawCubes(frontiers[i], 0.1, getColor(double(i) / frontiers.size(), 0.8), "frontier", i, 4);
    }

    vector<Eigen::Vector3d> frontier;
    for (int i = frontiers.size(); i < last_frontier_num_; ++i)
    {
      // displayCubeList(frontier, 0.1, getColor(1), i, 4);
      drawCubes(frontier, 0.1, getColor(1), "frontier", i, 4);
    }
    last_frontier_num_ = frontiers.size();
  }

  void PlanningVisualization::drawYawTraj(NonUniformBspline &pos, NonUniformBspline &yaw,
                                          const double &dt)
  {
    double duration = pos.getTimeSum();
    vector<Eigen::Vector3d> pts1, pts2;

    for (double tc = 0.0; tc <= duration + 1e-3; tc += dt)
    {
      Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
      pc[2] += 0.15;
      double yc = yaw.evaluateDeBoorT(tc)[0];
      Eigen::Vector3d dir(cos(yc), sin(yc), 0);
      Eigen::Vector3d pdir = pc + 1.0 * dir;
      pts1.push_back(pc);
      pts2.push_back(pdir);
    }
    displayLineList(pts1, pts2, 0.04, Eigen::Vector4d(1, 0.5, 0, 1), 0, 5);
  }

  void PlanningVisualization::drawYawPath(NonUniformBspline &pos, const vector<double> &yaw,
                                          const double &dt)
  {
    vector<Eigen::Vector3d> pts1, pts2;

    for (int i = 0; i < yaw.size(); ++i)
    {
      Eigen::Vector3d pc = pos.evaluateDeBoorT(i * dt);
      pc[2] += 0.3;
      Eigen::Vector3d dir(cos(yaw[i]), sin(yaw[i]), 0);
      Eigen::Vector3d pdir = pc + 1.0 * dir;
      pts1.push_back(pc);
      pts2.push_back(pdir);
    }
    displayLineList(pts1, pts2, 0.04, Eigen::Vector4d(1, 0, 1, 1), 1, 5);
  }

  Eigen::Vector4d PlanningVisualization::getColor(const double &h, double alpha)
  {
    double h1 = h;
    if (h1 < 0.0 || h1 > 1.0)
    {
      std::cout << "h out of range" << std::endl;
      h1 = 0.0;
    }

    double lambda;
    Eigen::Vector4d color1, color2;
    if (h1 >= -1e-4 && h1 < 1.0 / 6)
    {
      lambda = (h1 - 0.0) * 6;
      color1 = Eigen::Vector4d(1, 0, 0, 1);
      color2 = Eigen::Vector4d(1, 0, 1, 1);
    }
    else if (h1 >= 1.0 / 6 && h1 < 2.0 / 6)
    {
      lambda = (h1 - 1.0 / 6) * 6;
      color1 = Eigen::Vector4d(1, 0, 1, 1);
      color2 = Eigen::Vector4d(0, 0, 1, 1);
    }
    else if (h1 >= 2.0 / 6 && h1 < 3.0 / 6)
    {
      lambda = (h1 - 2.0 / 6) * 6;
      color1 = Eigen::Vector4d(0, 0, 1, 1);
      color2 = Eigen::Vector4d(0, 1, 1, 1);
    }
    else if (h1 >= 3.0 / 6 && h1 < 4.0 / 6)
    {
      lambda = (h1 - 3.0 / 6) * 6;
      color1 = Eigen::Vector4d(0, 1, 1, 1);
      color2 = Eigen::Vector4d(0, 1, 0, 1);
    }
    else if (h1 >= 4.0 / 6 && h1 < 5.0 / 6)
    {
      lambda = (h1 - 4.0 / 6) * 6;
      color1 = Eigen::Vector4d(0, 1, 0, 1);
      color2 = Eigen::Vector4d(1, 1, 0, 1);
    }
    else if (h1 >= 5.0 / 6 && h1 <= 1.0 + 1e-4)
    {
      lambda = (h1 - 5.0 / 6) * 6;
      color1 = Eigen::Vector4d(1, 1, 0, 1);
      color2 = Eigen::Vector4d(1, 0, 0, 1);
    }

    Eigen::Vector4d fcolor = (1 - lambda) * color1 + lambda * color2;
    fcolor(3) = alpha;

    return fcolor;
  }

  // PlanningVisualization::
} // namespace fast_planner
