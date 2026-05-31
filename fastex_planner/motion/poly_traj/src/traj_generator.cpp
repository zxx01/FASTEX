#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <vis_utils/marker_utils.h>
#include <Eigen/Eigen>
#include <ros/ros.h>

#include <swarmtal_msgs/drone_onboard_command.h>
#include <traj_generator/polynomial_traj.hpp>

using namespace std;

ros::Publisher state_pub, pos_cmd_pub, traj_pub;

nav_msgs::Odometry odom;
bool have_odom;

void displayPathWithColor(vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color,
                          int id) {
  auto scale = vis_utils::marker_utils::makeScale(resolution, resolution, resolution);
  auto ros_color = vis_utils::marker_utils::toRosColor(color);

  // DELETE old marker
  auto mk_del = vis_utils::marker_utils::makeSphereListMarker(
      "world", "", id, scale, ros_color, visualization_msgs::Marker::DELETE);
  traj_pub.publish(mk_del);

  // ADD new marker
  auto mk = vis_utils::marker_utils::makeSphereListMarker("world", "", id, scale, ros_color);
  vis_utils::marker_utils::appendPoints(mk, path);
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void drawState(Eigen::Vector3d pos, Eigen::Vector3d vec, int id, Eigen::Vector4d color) {
  auto ros_color = vis_utils::marker_utils::toRosColor(color);
  auto scale = vis_utils::marker_utils::makeScale(0.1, 0.2, 0.3);

  auto mk_state = vis_utils::marker_utils::makeMarker("world", "", id,
      visualization_msgs::Marker::ARROW, scale, ros_color);
  vis_utils::marker_utils::appendLine(mk_state, pos, pos + vec);
  state_pub.publish(mk_state);
}

void odomCallbck(const nav_msgs::Odometry& msg) {
  if (msg.child_frame_id == "X" || msg.child_frame_id == "O") return;

  odom = msg;
  have_odom = true;
}

int main(int argc, char** argv) {
  /* ---------- initialize ---------- */
  ros::init(argc, argv, "traj_generator");
  ros::NodeHandle node;

  ros::Subscriber odom_sub = node.subscribe("/uwb_vicon_odom", 50, odomCallbck);

  traj_pub = node.advertise<visualization_msgs::Marker>("/traj_generator/traj_vis", 10);
  state_pub = node.advertise<visualization_msgs::Marker>("/traj_generator/cmd_vis", 10);

  // pos_cmd_pub =
  // node.advertise<quadrotor_msgs::PositionCommand>("/traj_generator/position_cmd",
  // 50);

  pos_cmd_pub =
      node.advertise<swarmtal_msgs::drone_onboard_command>("/drone_commander/onboard_command", 10);

  ros::Duration(1.0).sleep();

  /* ---------- wait for odom ready ---------- */
  have_odom = false;
  while (!have_odom && ros::ok()) {
    cout << "no odomeetry." << endl;
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }

  /* ---------- generate trajectory using close-form minimum jerk ---------- */
  Eigen::MatrixXd pos(9, 3);
  // pos.row(0) =
  //     Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y,
  //     odom.pose.pose.position.z);

  pos.row(0) =
      Eigen::Vector3d(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
  // pos.row(0) = Eigen::Vector3d(-2, 0, 1);
  pos.row(1) = Eigen::Vector3d(-0.5, 0.5, 1);
  pos.row(2) = Eigen::Vector3d(0, 0, 1);
  pos.row(3) = Eigen::Vector3d(0.5, -0.5, 1);
  pos.row(4) = Eigen::Vector3d(1, 0, 1);
  pos.row(5) = Eigen::Vector3d(0.5, 0.5, 1);
  pos.row(6) = Eigen::Vector3d(0, 0, 1);
  pos.row(7) = Eigen::Vector3d(-0.5, -0.5, 1);
  pos.row(8) = Eigen::Vector3d(-1, 0, 1);

  Eigen::VectorXd time(8);
  time(0) = 2.0;
  time(1) = 1.5;
  time(2) = 1.5;
  time(3) = 1.5;
  time(4) = 1.5;
  time(5) = 1.5;
  time(6) = 1.5;
  time(7) = 2.0;

  Eigen::MatrixXd poly = generateTraj(pos, time);
  cout << "poly:\n" << poly << endl;

  cout << "pos:\n" << pos << endl;

  cout << "pos 0 1 2: " << pos(0) << ", " << pos(1) << ", " << pos(2) << endl;

  /* ---------- use polynomials ---------- */
  PolynomialTraj poly_traj;
  for (int i = 0; i < poly.rows(); ++i) {
    vector<double> cx(6), cy(6), cz(6);
    for (int j = 0; j < 6; ++j) {
      cx[j] = poly(i, j), cy[j] = poly(i, j + 6), cz[j] = poly(i, j + 12);
    }
    reverse(cx.begin(), cx.end());
    reverse(cy.begin(), cy.end());
    reverse(cz.begin(), cz.end());
    double ts = time(i);
    poly_traj.addSegment(cx, cy, cz, ts);
  }
  poly_traj.init();
  vector<Eigen::Vector3d> traj_vis = poly_traj.getTraj();

  displayPathWithColor(traj_vis, 0.05, Eigen::Vector4d(1, 0, 0, 1), 1);

  /* ---------- publish command ---------- */
  ros::Time start_time = ros::Time::now();
  ros::Time time_now;

  ros::Duration(0.1).sleep();

  swarmtal_msgs::drone_onboard_command cmd;
  cmd.command_type = swarmtal_msgs::drone_onboard_command::CTRL_POS_COMMAND;
  cmd.param1 = 0;
  cmd.param2 = 0;
  cmd.param3 = 0;
  cmd.param4 = 666666;
  cmd.param5 = 0;
  cmd.param6 = 0;
  cmd.param7 = 0;
  cmd.param8 = 0;
  cmd.param9 = 0;
  cmd.param10 = 0;

  while (ros::ok()) {
    time_now = ros::Time::now();
    double tn = (time_now - start_time).toSec();
    Eigen::Vector3d pt = poly_traj.evaluate(tn);
    Eigen::Vector3d vel = poly_traj.evaluateVel(tn);
    Eigen::Vector3d acc = poly_traj.evaluateAcc(tn);

    cmd.param1 = int(pt(0) * 10000);
    cmd.param2 = int(pt(1) * 10000);
    cmd.param3 = int(pt(2) * 10000);

    cmd.param5 = int(vel(0) * 10000);
    cmd.param6 = int(vel(1) * 10000);

    cmd.param7 = 0;
    cmd.param8 = int(acc(0) * 10000);
    cmd.param9 = int(acc(1) * 10000);

    pos_cmd_pub.publish(cmd);

    drawState(pt, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
    drawState(pt, acc, 1, Eigen::Vector4d(0, 0, 1, 1));
    ros::Duration(0.01).sleep();
  }

  ros::spin();
  return 0;
}
