/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-05-04 15:09:24
 * @LastEditTime: 2026-05-04 15:53:47
 * @Description: Utility functions for creating and manipulating ROS visualization markers
 */

#ifndef VIS_UTILS_MARKER_UTILS_H
#define VIS_UTILS_MARKER_UTILS_H

#include <initializer_list>
#include <string>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace vis_utils
{
/**
 * @brief Utility functions for creating and manipulating ROS visualization markers
 */
namespace marker_utils
{
/**
 * @brief Convert a generic 3D point type to geometry_msgs::Point
 *
 * @tparam PointType The type of the input point, must have x(), y(), z() methods
 * @param point The input point to convert
 * @return geometry_msgs::Point The converted ROS geometry_msgs::Point
 */
template <typename PointType> inline geometry_msgs::Point toRosPoint(const PointType& point)
{
    geometry_msgs::Point ros_point;
    ros_point.x = point.x();
    ros_point.y = point.y();
    ros_point.z = point.z();
    return ros_point;
}

/**
 * @brief Convert a generic color type (e.g. Eigen::Vector4d) to std_msgs::ColorRGBA
 *
 * @tparam ColorType The type of the input color, must support array indexing [0] to [3] (RGBA)
 * @param rgba The input color to convert
 * @return std_msgs::ColorRGBA The converted ROS std_msgs::ColorRGBA
 */
template <typename ColorType> inline std_msgs::ColorRGBA toRosColor(const ColorType& rgba)
{
    std_msgs::ColorRGBA color;
    color.r = rgba[0];
    color.g = rgba[1];
    color.b = rgba[2];
    color.a = rgba[3];
    return color;
}

/**
 * @brief Create a geometry_msgs::Vector3 representing the scale of a marker
 *
 * @param x Scale in x axis
 * @param y Scale in y axis
 * @param z Scale in z axis
 * @return geometry_msgs::Vector3 The scale vector
 */
inline geometry_msgs::Vector3 makeScale(const double x, const double y, const double z)
{
    geometry_msgs::Vector3 scale;
    scale.x = x;
    scale.y = y;
    scale.z = z;
    return scale;
}

/**
 * @brief Configure basic properties of an existing visualization_msgs::Marker
 *
 * @param marker The marker to be configured
 * @param frame_id The TF frame ID
 * @param ns The namespace of the marker
 * @param id The marker ID
 * @param type The type of the marker (e.g. visualization_msgs::Marker::SPHERE_LIST)
 * @param scale The scale of the marker
 * @param color The color of the marker
 * @param action The action to perform (e.g. visualization_msgs::Marker::ADD)
 * @param stamp The timestamp of the marker
 */
inline void configureMarker(visualization_msgs::Marker& marker, const std::string& frame_id,
                            const std::string& ns, const int id, const int type,
                            const geometry_msgs::Vector3& scale, const std_msgs::ColorRGBA& color,
                            const int action = visualization_msgs::Marker::ADD,
                            const ros::Time& stamp = ros::Time::now())
{
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = action;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale = scale;
    marker.color = color;
    marker.points.clear();
    marker.colors.clear();
}

/**
 * @brief Create and configure a new visualization_msgs::Marker
 *
 * @param frame_id The TF frame ID
 * @param ns The namespace of the marker
 * @param id The marker ID
 * @param type The type of the marker (e.g. visualization_msgs::Marker::SPHERE_LIST)
 * @param scale The scale of the marker
 * @param color The color of the marker
 * @param action The action to perform
 * @param stamp The timestamp of the marker
 * @return visualization_msgs::Marker The configured marker
 */
inline visualization_msgs::Marker makeMarker(const std::string& frame_id, const std::string& ns,
                                             const int id, const int type,
                                             const geometry_msgs::Vector3& scale,
                                             const std_msgs::ColorRGBA& color,
                                             const int action = visualization_msgs::Marker::ADD,
                                             const ros::Time& stamp = ros::Time::now())
{
    visualization_msgs::Marker marker;
    configureMarker(marker, frame_id, ns, id, type, scale, color, action, stamp);
    return marker;
}

/**
 * @brief Create a POINTS type marker
 *
 * @param frame_id The TF frame ID
 * @param ns The namespace of the marker
 * @param id The marker ID
 * @param scale The scale of the points
 * @param color The color of the points
 * @param action The action to perform
 * @param stamp The timestamp of the marker
 * @return visualization_msgs::Marker The POINTS marker
 */
inline visualization_msgs::Marker
makePointMarker(const std::string& frame_id, const std::string& ns, const int id,
                const geometry_msgs::Vector3& scale, const std_msgs::ColorRGBA& color,
                const int action = visualization_msgs::Marker::ADD,
                const ros::Time& stamp = ros::Time::now())
{
    return makeMarker(frame_id, ns, id, visualization_msgs::Marker::POINTS, scale, color, action,
                      stamp);
}

/**
 * @brief Create a SPHERE_LIST type marker
 *
 * @param frame_id The TF frame ID
 * @param ns The namespace of the marker
 * @param id The marker ID
 * @param scale The scale of the spheres
 * @param color The color of the spheres
 * @param action The action to perform
 * @param stamp The timestamp of the marker
 * @return visualization_msgs::Marker The SPHERE_LIST marker
 */
inline visualization_msgs::Marker
makeSphereListMarker(const std::string& frame_id, const std::string& ns, const int id,
                     const geometry_msgs::Vector3& scale, const std_msgs::ColorRGBA& color,
                     const int action = visualization_msgs::Marker::ADD,
                     const ros::Time& stamp = ros::Time::now())
{
    return makeMarker(frame_id, ns, id, visualization_msgs::Marker::SPHERE_LIST, scale, color,
                      action, stamp);
}

/**
 * @brief Create a LINE_LIST type marker
 *
 * @param frame_id The TF frame ID
 * @param ns The namespace of the marker
 * @param id The marker ID
 * @param width The width of the lines
 * @param color The color of the lines
 * @param action The action to perform
 * @param stamp The timestamp of the marker
 * @return visualization_msgs::Marker The LINE_LIST marker
 */
inline visualization_msgs::Marker
makeLineListMarker(const std::string& frame_id, const std::string& ns, const int id,
                   const double width, const std_msgs::ColorRGBA& color,
                   const int action = visualization_msgs::Marker::ADD,
                   const ros::Time& stamp = ros::Time::now())
{
    return makeMarker(frame_id, ns, id, visualization_msgs::Marker::LINE_LIST,
                      makeScale(width, 0.0, 0.0), color, action, stamp);
}

/**
 * @brief Append a single point to a marker
 *
 * @tparam PointType The type of the input point
 * @param marker The marker to append the point to
 * @param point The point to append
 */
template <typename PointType>
inline void appendPoint(visualization_msgs::Marker& marker, const PointType& point)
{
    marker.points.push_back(toRosPoint(point));
}

/**
 * @brief Append a single colored point to a marker
 *
 * @tparam PointType The type of the input point
 * @tparam ColorType The type of the input color
 * @param marker The marker to append the point to
 * @param point The point to append
 * @param color The color of the point
 */
template <typename PointType, typename ColorType>
inline void appendColoredPoint(visualization_msgs::Marker& marker, const PointType& point,
                               const ColorType& color)
{
    marker.points.push_back(toRosPoint(point));
    marker.colors.push_back(toRosColor(color));
}

/**
 * @brief Append a container of points to a marker
 *
 * @tparam PointContainer The type of the container holding the points
 * @param marker The marker to append the points to
 * @param points The container of points to append
 */
template <typename PointContainer>
inline void appendPoints(visualization_msgs::Marker& marker, const PointContainer& points)
{
    marker.points.reserve(marker.points.size() + points.size());
    for (const auto& point : points)
        appendPoint(marker, point);
}

/**
 * @brief Append a line segment (two points) to a LINE_LIST marker
 *
 * @tparam StartPointType The type of the start point
 * @tparam EndPointType The type of the end point
 * @param marker The marker to append the line to
 * @param start The start point of the line
 * @param end The end point of the line
 */
template <typename StartPointType, typename EndPointType>
inline void appendLine(visualization_msgs::Marker& marker, const StartPointType& start,
                       const EndPointType& end)
{
    marker.points.push_back(toRosPoint(start));
    marker.points.push_back(toRosPoint(end));
}

/**
 * @brief Assign an initializer list of markers to a MarkerArray
 *
 * @param marker_array The MarkerArray to assign to
 * @param markers The initializer list of markers
 */
inline void assignMarkers(visualization_msgs::MarkerArray& marker_array,
                          std::initializer_list<visualization_msgs::Marker> markers)
{
    marker_array.markers.assign(markers.begin(), markers.end());
}

} // namespace marker_utils
} // namespace vis_utils

#endif
