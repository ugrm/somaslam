#pragma once
#include <ros/ros.h>
#include <ros/message_forward.h>
#include <tf/tf.h>
#include <tf2_ros/transform_broadcaster.h>
#include "loop_closer/submap_loop_closer.h"
#include "odom_error_propagator.h"
#include "visualizer.h"
#include "graphs.h"
#include "ls_extractor/impl/smc.h"
#include "ls_extractor/utils.h"
#include <set>


namespace nav_msgs {
    ROS_DECLARE_MESSAGE(Odometry)
}
namespace sensor_msgs {
    ROS_DECLARE_MESSAGE(LaserScan)
}
namespace ls_extractor {
    class LineSegment;
    using SegmentVector = std::vector<LineSegment>;
}

constexpr int LANDMARK_BASE_ID = 10000000;
constexpr int LINESEGMENT_BASE_ID = 20000000;
class Drone {
   public:
    tf2_ros::TransformBroadcaster br;
    tf::Transform odom_map_tf;
    geometry_msgs::TransformStamped odom_bl_tf;

    OdomErrorPropagator<double> odom_prop;
    geometry_msgs::PoseStamped cor_pose_msg;
    ros::Publisher cor_pose_pub;

    std::vector<Eigen::Vector2f> table;
    Delta prev_pose;

    LandmarkGraph lm_graph;
    PoseGraph pose_graph;
    SubmapLoopCloser loop_closer;

    bool need_reinit = true;
    int vertex_id = 0;
    int landmark_id = LANDMARK_BASE_ID;
    int lineSegment_id = LINESEGMENT_BASE_ID;
    const double landmark_assoc_thresh;
    const double landmark_max_gap;
    const double landmark_max_dist;

    int removed_ll_constraints_count = 0;  // Counter for removed constraints

    std::vector<double> final_ll_constraint_errors;
    std::vector<double> removed_ll_constraint_errors;

    const int maximum_constraints;
    const double landmark_min_length_threshold;
    const int max_past_landmark_ID;
    const double max_past_landmark_dist;
    const int min_pl_edge_count;
    const double parallel_ll_info_matrix;
    const double orthogonal_ll_info_matrix;
    
    double chi2_before = 0.0;
    double traveled_dist = 0.0;
    //std::vector<Landmark> landmarks;

    Drone(ros::NodeHandle& nh, const XmlRpc::XmlRpcValue& config);

    void msgCallback(const ls_extractor::SegmentVector& segments_in, const nav_msgs::Odometry& odom, const sensor_msgs::LaserScan& pc);

    void addLandmarkObservations(g2o::VertexRhoTheta* landmark, const ls_extractor::LineSegment& bl_line, g2o::VertexSE2* new_vertex);

    void addFakeLandmarkLandmarkConstraint(g2o::VertexRhoTheta* landmark1, g2o::VertexRhoTheta* landmark2);

    void addLandmarkLandmarkConstraint(g2o::VertexRhoTheta *landmark1, g2o::VertexRhoTheta *landmark2, double angle1, const ls_extractor::LineSegment &line2, g2o::VertexSE2 *new_vertex);

    void printPoseLandmarkEdgeVertexIDs();


    g2o::VertexRhoTheta* mergeLine(const Eigen::Vector2f &start, const Eigen::Vector2f &end);
    void submap_matching();

private:
    int added_ll_constraints_count = 0;  // total constraints added
    std::set<std::pair<int, int>> unique_ll_constraint_pairs;  // unique landmark-landmark pairs added

};
