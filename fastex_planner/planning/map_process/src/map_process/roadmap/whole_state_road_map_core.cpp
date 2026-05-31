/*
 * @Author: Xiaoxun Zhang
 * @Date: 2024-09-19 12:09:37
 * @LastEditTime: 2026-05-03 14:18:25
 * @Description:
 */

#include "map_process/roadmap/whole_state_road_map.h"

#include "map_process/core/map_process_constants.h"
#include <vis_utils/marker_utils.h>
#include "utils/hash_grid.hpp"

#include <random>

namespace map_process
{

/**
 * @brief Construct a new WSRoadMap object.
 *
 * @param nh ROS node handle for parameter loading.
 * @param sdf_map Shared pointer to the SDF map for collision queries.
 * @param planner_dim Planning dimension (2 or 3).
 * @throws std::runtime_error if planner_dim is not 2 or 3.
 */
WSRoadMap::WSRoadMap(ros::NodeHandle& nh, fast_planner::SDFMap::Ptr sdf_map, const int planner_dim)
{
    if (planner_dim != 2 && planner_dim != 3)
        throw std::runtime_error("planner_dim is error!");

    is_graph_initialized_ = false;
    sdf_map_ = sdf_map;
    planner_dim_ = planner_dim;
    loadParamsFromROS(nh);

    active_points_ikdtree_ = std::make_shared<KD_TREE<PointType>>(0.3, 0.6, 0.2);
    fixed_points_ikdtree_ = std::make_shared<KD_TREE<PointType>>(0.3, 0.6, 0.2);
    is_active_points_ikdtree_built_ = false;
    is_fixed_points_ikdtree_built_ = false;

    valid_region_aabb_tree_ = std::make_unique<aabb::Tree>(3, 0.0, 16, true);
}

/**
 * @brief Load parameters from ROS parameter server
 *
 * This function loads various parameters from the ROS parameter server and assigns them to the
 * corresponding member variables of the WSRoadMap class.
 *
 * @param nh The ROS NodeHandle used to access the parameter server
 */
void WSRoadMap::loadParamsFromROS(const ros::NodeHandle& nh)
{
    bool is_param_load = true;

    // Load parameters from the ROS parameter server and assign them to the corresponding member
    // variables.
    is_param_load &= nh.param("WSRoadmap/frame_id", wsgp_.frame_id_, std::string("world"));
    is_param_load &= nh.param("WSRoadmap/sample_dist", wsgp_.sample_dist_, 1.0f);
    is_param_load &= nh.param("WSRoadmap/min_interval", wsgp_.min_interval_, 2.0f);
    is_param_load &= nh.param("WSRoadmap/bound_margin", wsgp_.bound_margin_, 1.0f);
    is_param_load &= nh.param("WSRoadmap/connectable_range", wsgp_.connectable_range_, 5.0f);
    is_param_load &= nh.param("WSRoadmap/connectable_num", wsgp_.connectable_num_, 6);

    // Check if all parameters were successfully loaded.
    if (!is_param_load)
        ROS_ERROR("WSRoadmap params load failed!");
}

/**
 * @brief Set the planner dimension
 *
 * This function sets the dimension of the planner. The dimension can only be 2 or 3.
 * If an invalid dimension is provided, the function throws a runtime error.
 *
 * @param planner_dim The dimension of the planner (must be 2 or 3)
 * @throws std::runtime_error if the planner_dim is not 2 or 3
 */
void WSRoadMap::setPlannerDim(const int planner_dim)
{
    if (planner_dim != 2 && planner_dim != 3)
        throw std::runtime_error("planner_dim is error!");

    planner_dim_ = planner_dim;
}

/**
 * @brief Set the initial position
 *
 * This function sets the initial position for the roadmap.
 *
 * @param initial_position The initial position to be set
 */
void WSRoadMap::setInitialPosition(const eigen_utils::Vec3f& initial_position)
{
    wsgp_.initial_position_ = initial_position;
}

/**
 * @brief Set the current position
 *
 * This function sets the current position for the roadmap.
 *
 * @param current_position The current position to be set
 */
void WSRoadMap::setCurrentPosition(const eigen_utils::Vec3f& current_position)
{
    current_position_ = current_position;
}

/**
 * @brief Set the boundaries for sample points
 *
 * This function sets the maximum and minimum boundaries for the sample points used in the roadmap.
 *
 * @param max_bound The maximum boundary for the sample points
 * @param min_bound The minimum boundary for the sample points
 */
void WSRoadMap::setSamplePointsBound(const eigen_utils::Vec3d& max_bound,
                                     const eigen_utils::Vec3d& min_bound)
{
    samplepoint_max_bound_ = max_bound;
    samplepoint_min_bound_ = min_bound;
}

/**
 * @brief Insert valid region boxes into the AABB tree
 *
 * This function inserts the given valid region boxes into the AABB tree. Each box
 * is represented by its minimum and maximum corners. The boxes are stored in the
 * `valid_region_boxes_` vector and their corresponding AABB representations are
 * inserted into the `valid_region_aabb_tree_`.
 *
 * @param boxes A vector of valid region boxes to be inserted
 */
void WSRoadMap::insertValidRegionBoxes(const std::vector<process_utils::CubeBox>& boxes)
{
    for (const auto& box : boxes)
    {
        std::vector<double> lb(box.min_.data(), box.min_.data() + box.min_.size());
        std::vector<double> ub(box.max_.data(), box.max_.data() + box.max_.size());
        valid_region_aabb_tree_->insertParticle(valid_region_count_, lb, ub);
        valid_region_explored_state_[valid_region_count_] = false;
        ++valid_region_count_;
    }
}

/**
 * @brief Get the vertex ID for a given point
 *
 * This function retrieves the vertex ID associated with a given point in the roadmap.
 *
 * @param point The point for which the vertex ID is to be retrieved
 * @return int The vertex ID associated with the given point
 */
int WSRoadMap::getVertexId(const eigen_utils::Vec3f& point) { return graph_.getVertexId(point); }

/**
 * @brief Check if a vertex exists for a given point
 *
 * This function checks whether a vertex exists in the roadmap for a given point.
 *
 * @param point The point to be checked
 * @return true If the vertex exists for the given point
 * @return false If the vertex does not exist for the given point
 */
bool WSRoadMap::isVertexExisted(const eigen_utils::Vec3f& point) const
{
    return graph_.isVertexExisted(point);
}

/**
 * @brief Get the vertex with the specified ID.
 *
 * This function retrieves the vertex from the road map that corresponds to the given vertex ID.
 *
 * @param vertex_id The ID of the vertex to be retrieved.
 * @return WSRoadMap::VertexType& A reference to the vertex with the specified ID.
 */
WSRoadMap::VertexType& WSRoadMap::getVertex(const int vertex_id)
{
    return graph_.getVertex(vertex_id);
}

/**
 * @brief Get the total number of vertices in the graph.
 *
 * @return int Number of vertices.
 */
int WSRoadMap::getVertexNum() const { return graph_.getVertexNum(); }

/**
 * @brief Get the linked edges for a given vertex.
 *
 * This function retrieves the edges that are linked to the specified vertex in the road map.
 *
 * @param vertex_id The ID of the vertex for which the linked edges are to be retrieved.
 * @return std::unordered_map<int, WSRoadMap::EdgeType>& A reference to an unordered map containing
 * the linked edges.
 */
std::unordered_map<int, WSRoadMap::EdgeType>& WSRoadMap::getLinkedEdges(const int vertex_id)
{
    return graph_.getLinkedEdges(vertex_id);
}

/**
 * @brief Check if the vertex needs to be updated manually
 *
 * @param vertex The vertex to be checked
 * @return true If the vertex needs to be updated manually
 * @return false If the vertex does not need to be updated manually
 */
bool WSRoadMap::isManuallyUpdatedVertex(const VertexType& vertex) const
{
    return vertex.extra_data_.vertex_type_ == WSVertexExtraData::VertexType::VIEWPOINT ||
           vertex.extra_data_.vertex_type_ == WSVertexExtraData::VertexType::REGION_CENTER;
}

/**
 * @brief Initialize the topological graph from an origin position.
 *
 * Resets the graph and adds the origin as the first vertex. Builds the
 * active-points KD-tree with the origin position if not yet initialized.
 *
 * @param origin_position The seed position for the graph.
 */
void WSRoadMap::initTopoGraph(const eigen_utils::Vec3f& origin_position)
{
    // Reset the graph
    graph_.resetGraph();
    WSVertexExtraData vertex_extra_data(WSVertexExtraData::VertexSource::SAMPLE,
                                        WSVertexExtraData::VertexState::FREE,
                                        WSVertexExtraData::VertexType::COMMON);
    graph_.addVertex(VertexType(origin_position, true, vertex_extra_data), true);
    is_graph_initialized_ = true;

    PointVector add_points_vector;
    add_points_vector.emplace_back(origin_position.x(), origin_position.y(), origin_position.z());

    if (!is_active_points_ikdtree_built_)
    {
        active_points_ikdtree_->Build(add_points_vector);
        is_active_points_ikdtree_built_ = true;
    }
    else
    {
        active_points_ikdtree_->Add_Points(add_points_vector, false);
    }
}

} // namespace map_process
