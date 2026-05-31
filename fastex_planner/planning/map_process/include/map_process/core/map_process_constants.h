/*
 * @Author: Xiaoxun Zhang
 * @Date: 2026-05-02
 * @Description: Centralized constants for the map_process module.
 *     All magic numbers scattered across the codebase are consolidated here
 *     to improve maintainability and eliminate hardcoded values.
 */

#ifndef _MAP_PROCESS_CONSTANTS_H_
#define _MAP_PROCESS_CONSTANTS_H_

#include <cmath>

namespace map_process
{
namespace constants
{

// ============================================================================
//  Generic / cross-module tolerances & defaults
// ============================================================================

/// Epsilon for floating-point equality checks (point coincidence, etc.)
constexpr double kEpsilon = 1e-3;

/// Default voxel size for point-cloud down-sampling [m]
constexpr float kVoxelSizeDownsample = 0.5f;

/// Default voxel size for voxel-grid leaf [m]
constexpr double kVoxelGridLeafSize = 2.0;

/// Neighbour search range used by roadmap / path-searcher / dynamic-grid [m]
constexpr double kNeighborSearchRange = 8.0;

/// Default box margin applied when expanding perception / update regions [m]
constexpr double kBoxMargin = 0.25;

/// Default interpolation step size used when straightening a path [m]
constexpr double kPathInterpolationStep = 2.0;

/// Default max straight-line distance used in coarse-path optimisation [m]
constexpr double kMaxStraightDistanceDefault = 5.0;

/// Fallback path cost returned when a search fails
constexpr double kPathCostFallback = 1e4;

// ============================================================================
//  Frontier Manager — search & clustering
// ============================================================================

/// Hash-grid resolution multiplier for global frontier spatial index
constexpr double kFrontierHashResMultiplier = 10.0;

/// Bounding-box half-side used for local frontier search [m]
constexpr double kFrontierSearchBoxHalfSize = 8.0;

/// Normal-weight factor when computing frontier normals
constexpr double kFrontierNormalFreeWeight = 0.5;

/// Cosine threshold for frontier-direction angle check  (cos 60°)
constexpr double kFrontierCosAngleThreshold = 0.5; // cos(M_PI / 3)

/// Pitch-angle threshold for dormant-cluster detection [rad]
constexpr double kFrontierDormantPitchThreshold = M_PI / 3.0; // 60°

// ============================================================================
//  Frontier Manager — viewpoint generation
// ============================================================================

/// Non-Maximum-Suppression distance threshold [m]
constexpr double kViewpointNmsDistance = 1.5;

/// Minimum / maximum viewpoint sampling distance along frontier normal [m]
constexpr double kViewpointSampleDistMin = 1.0;
constexpr double kViewpointSampleDistMax = 5.0;

/// Number of viewpoint sampling steps along the normal direction
constexpr int kViewpointSampleSteps = 3;

/// Adaptive-threshold parameter offset for visibility scoring
constexpr double kViewpointThresholdOffset = -0.5;

/// Distance-threshold multiplier & clamp values for adaptive threshold
constexpr double kViewpointDistThreshMultiplier = 1.5;
constexpr double kViewpointDistThreshMin = 2.0;
constexpr double kViewpointDistThreshMax = 3.0;

/// Vertical-normal threshold for rejecting sideways viewpoints [deg]
constexpr double kViewpointVerticalNormalDeg = 30.0;

// ============================================================================
//  Whole-State RoadMap
// ============================================================================

/// KD-tree delete / balance / box-length parameters
constexpr float kWSRoadMapKDTreeDeleteParam = 0.3f;
constexpr float kWSRoadMapKDTreeBalanceParam = 0.6f;
constexpr float kWSRoadMapKDTreeBoxLength = 0.2f;

/// Default point-insertion minimum interval [m]
constexpr float kWSRoadMapDefaultMinInterval = 2.0f;

/// Default sample distance for graph growing [m]
constexpr float kWSRoadMapDefaultSampleDist = 1.0f;

/// Default bounding margin [m]
constexpr float kWSRoadMapDefaultBoundMargin = 1.0f;

/// Default max connectable range for newly added points [m]
constexpr float kWSRoadMapDefaultConnectableRange = 5.0f;

/// Default max number of connections per new vertex
constexpr int kWSRoadMapDefaultConnectableNum = 6;

/// Box margin for AABB collision queries [m]
constexpr double kWSRoadMapBoxMarginCollision = 0.1;

/// Standard-deviation factor for normal-distribution sampling
constexpr double kWSRoadMapNormalStdFactor = 0.5;

/// Gray-channel value used for visualization
constexpr double kWSRoadMapVisGray = 0.5;

/// Alpha for translucent markers
constexpr double kWSRoadMapVisAlpha = 0.3;

/// Green channel for valid (free) vertices
constexpr double kWSRoadMapVisGreen = 0.5;

// ============================================================================
//  Path Searcher
// ============================================================================

/// Velocity norm below which the robot is considered stationary → penalty
constexpr double kPathSearchStationaryVelocityNorm = 1e-3;

/// Midpoint interpolation coefficient for cost calculation
constexpr double kPathSearchMidpointCoeff = 0.5;

// ============================================================================
//  Dynamic Expanding Grid
// ============================================================================

/// KD-tree delete / balance / box-length for key-position tree
constexpr float kDynGridKDTreeDeleteParam = 0.3f;
constexpr float kDynGridKDTreeBalanceParam = 0.6f;
constexpr float kDynGridKDTreeBoxLength = 0.2f;

/// Default initial grid resolution (all dimensions) [m]
constexpr double kDynGridDefaultResolution = 16.0;

/// Default min / max extent of the whole grid [m]
constexpr double kDynGridDefaultMinXY = -100.0;
constexpr double kDynGridDefaultMinZ = -0.5;
constexpr double kDynGridDefaultMaxXY = 100.0;
constexpr double kDynGridDefaultMaxZ = 20.0;

/// Default consistent cost for enduring edges
constexpr double kDynGridDefaultConsistentCost = -5.0;

/// Default max straight distance for temporary edge checks [m]
constexpr double kDynGridDefaultMaxStraightDist = 12.0;

/// Covered range for zone coverage evaluation [m]
constexpr double kDynGridCoveredRange = 15.0;

/// Known-ratio threshold for grid exploration state decisions
constexpr double kDynGridKnownRatioThreshold = 0.2;

/// Label-cloud coverage threshold
constexpr double kDynGridLabelCloudThreshold = 0.1;

/// Zone-centroid weighting for unknown-zone centroid computation
constexpr double kDynGridZoneCenterWeight = 0.5;

/// Small perturbation added to centroids to avoid degeneracy
constexpr double kDynGridCentroidPerturbation = 0.1;

/// Epsilon for grid-boundary comparisons
constexpr double kDynGridBoundaryEpsilon = 1e-3;

/// Edge-cost penalty multiplier
constexpr double kDynGridEdgeCostPenalty = 1.0;

/// Flag value indicating an invalid / uncomputed edge cost
constexpr double kDynGridInvalidEdgeCost = -1.0;

/// Point-intensity threshold for coverage evaluation
constexpr double kDynGridIntensityThreshold = 1e-3;

/// Search box half-size used for zone queries [m]
constexpr double kDynGridSearchBoxHalfSize = 8.0;

/// Box margin added when extending grid range in map_process [m]
constexpr double kDynGridExtendMargin = 0.25;

// ============================================================================
//  History Position Graph
// ============================================================================

/// Minimum distance between inserted history positions [m]
constexpr float kHistoryPosMinInsertDist = 4.0f;

/// Minimum interval between consecutive key positions [m]
constexpr float kHistoryPosMinIntervalDist = 1.0f;

/// Maximum link distance for graph edges [m]
constexpr float kHistoryPosMaxLinkDist = 32.0f;

// ============================================================================
//  Map Process — main orchestration
// ============================================================================

/// Box margin added around the perception range [m]
constexpr float kMapProcessPerceptionBoxMargin = 4.0f;

/// Alpha channel for visualization markers
constexpr float kMapProcessVisAlpha = 0.2f;

// ============================================================================
//  Single-Level Grid — exploration state machine
// ============================================================================

/// Known-ratio threshold above which a grid is considered EXPLORED
constexpr double kSingleLevelGridKnownRatioExplored = 0.2;

/// Label-cloud coverage threshold
constexpr double kSingleLevelGridCoverageThreshold = 0.1;

/// Sub-grid dimensions used by SubGridManager (default)
constexpr int kSubGridNumX = 8;
constexpr int kSubGridNumY = 8;
constexpr int kSubGridNumZ = 4;

// ============================================================================
//  Exploration Manager — shared constants with exploration_manager module
// ============================================================================

/// Default divide range for grid partitioning in incremental planning [m]
constexpr double kDefaultDivideRange = 40.0;

/// Default number of segment points for path partitioning
constexpr int kDefaultSegmentPtNum = 5;

/// Default path-length threshold for target extraction [m]
constexpr double kDefaultPathLengthThreshold = 15.0;

/// Default planning distance [m]
constexpr double kDefaultPlanDist = 5.0;

/// Default max straight distance for coarse path [m]
constexpr double kDefaultStraightMaxDist = 12.0;

/// Default relax time for yaw B-spline [s]
constexpr double kDefaultRelaxTime = 1.0;

// FSM timer intervals [s]
constexpr double kFSMTimerExecInterval = 0.01;
constexpr double kFSMTimerSafetyInterval = 0.05;
constexpr double kFSMTimerVisInterval = 0.05;
constexpr double kFSMTimerFrontierInterval = 0.5;

/// Default replan threshold (negative = not loaded from ROS)
constexpr double kDefaultReplanThresh = -1.0;

// ============================================================================
//  IK-D Tree (third-party, original constants preserved for reference)
// ============================================================================

namespace ikdtree
{
constexpr double kEps = 1e-6;
constexpr int kMinUnbalancedTreeSize = 10;
constexpr int kMultiThreadRebuildPointNum = 1500;
constexpr bool kDownsampleSwitch = false;
constexpr float kForceRebuildPercentage = 0.2f;
constexpr int kQueueLength = 1000000;

constexpr float kDefaultAlphaBal = 0.5f;
constexpr float kDefaultAlphaDel = 0.0f;
constexpr float kDefaultDeleteCriterion = 0.5f;
constexpr float kDefaultBalanceCriterion = 0.7f;
constexpr float kDefaultDownsampleSize = 0.2f;
} // namespace ikdtree

} // namespace constants
} // namespace map_process

#endif // !_MAP_PROCESS_CONSTANTS_H_
