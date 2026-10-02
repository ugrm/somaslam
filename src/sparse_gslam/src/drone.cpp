#include "drone.h"


#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>


#include <boost/math/distributions/chi_squared.hpp>
#include <unordered_set>


#include "g2o_bindings/edge_se2_rhotheta.h"
#include "g2o_bindings/edge_rhotheta_rhotheta.h"


#include "ls_extractor/impl/smc.h"
#include "ls_extractor/utils.h"


#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <iostream>


#include <unordered_map>
#include <unordered_set>  // For the set of landmark IDs
#include <atomic>

#include <fstream>

#include "g2o/core/robust_kernel_impl.h"


// Define the maximum number of constraints allowed between any pair of landmarks
const int MAX_CONSTRAINTS = 4;  // Set this to the desired maximum number


// Use a map to track the number of constraints between pairs of landmarks
std::unordered_map<std::pair<int, int>, int, boost::hash<std::pair<int, int>>> constraint_count;


std::pair<int, int> makeOrderedPair(int id1, int id2) {
    if (id1>id2) std::swap(id1, id2);
    return std::make_pair(id1, id2);
}




const double ERROR= M_PI/18;


using chi2_distribution = boost::math::chi_squared_distribution<double>;


double norm_angle(double angle) {
    if (angle > M_PI) {
        return angle - 2 * M_PI;
    } else if (angle < -M_PI) {
        return angle + 2 * M_PI;
    } else {
        return angle;
    }
}


// Function to count landmark-landmark constraints connected to a specific landmark
int countLandmarkLandmarkConstraints(const g2o::VertexRhoTheta *landmark) {
    int count = 0;




    // Iterate over all edges connected to the landmark
    for (const auto &edge : landmark->edges()) {
        // Check if the edge is a landmark-landmark constraint
        if (dynamic_cast<g2o::EdgeRhoThetaRhoTheta*>(edge)) {
            count++;
        }
    }




    return count;
}




// Function to compute the Euclidean distance between two points
double distanceBetweenPoints(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2) {
    return (p1 - p2).norm();
}


// Function to compute the minimum distance between a point and a line segment
double pointToSegmentDistance(const Eigen::Vector2f &p, const Eigen::Vector2f &seg_start, const Eigen::Vector2f &seg_end) {
    Eigen::Vector2f v = seg_end - seg_start;
    Eigen::Vector2f w = p - seg_start;


    double c1 = w.dot(v);
    if (c1 <= 0.0) {
        return distanceBetweenPoints(p, seg_start);
    }


    double c2 = v.dot(v);
    if (c2 <= c1) {
        return distanceBetweenPoints(p, seg_end);
    }


    double b = c1 / c2;
    Eigen::Vector2f pb = seg_start + b * v;
    return distanceBetweenPoints(p, pb);
}


// Function to check if two segments are close
bool areSegmentsClose(const Eigen::Vector2f &seg1_start, const Eigen::Vector2f &seg1_end,
                      const Eigen::Vector2f &seg2_start, const Eigen::Vector2f &seg2_end, double threshold) {
    // Check the minimum distance between each endpoint of one segment to the other segment
    double dist1 = pointToSegmentDistance(seg1_start, seg2_start, seg2_end);
    double dist2 = pointToSegmentDistance(seg1_end, seg2_start, seg2_end);
    double dist3 = pointToSegmentDistance(seg2_start, seg1_start, seg1_end);
    double dist4 = pointToSegmentDistance(seg2_end, seg1_start, seg1_end);


    // Check if any of these distances is below the threshold
    return (dist1 <= threshold || dist2 <= threshold || dist3 <= threshold || dist4 <= threshold);
}










Eigen::Vector2d transformLandmarkToPoseFrame(const g2o::VertexRhoTheta* landmark, const g2o::SE2& current_pose) {
    // Step 1: Extract the landmark's global estimate
    double rho_g = landmark->estimate()[0];
    double theta_g = landmark->estimate()[1];




    // Step 2: Convert the landmark's global rho-theta to Cartesian coordinates
    double x_g = rho_g * std::cos(theta_g);
    double y_g = rho_g * std::sin(theta_g);




    // Step 3: Transform to the current pose's frame using the inverse pose transformation
    double x_p = current_pose.translation()[0];
    double y_p = current_pose.translation()[1];
    double theta_p = current_pose.rotation().angle();




    double x_l = std::cos(theta_p) * (x_g - x_p) + std::sin(theta_p) * (y_g - y_p);
    double y_l = -std::sin(theta_p) * (x_g - x_p) + std::cos(theta_p) * (y_g - y_p);




    // Step 4: Convert back to rho-theta in the local frame
    double rho_l = std::sqrt(x_l * x_l + y_l * y_l);
    double theta_l = std::atan2(y_l, x_l);




    // Return the transformed estimate as a 2D vector (rho_l, theta_l)
    return Eigen::Vector2d(rho_l, theta_l);
}




double transformLineAngleToAnotherPose(double line1_angle, const g2o::SE2& pose1, const g2o::SE2& pose2) {
    // Extract the rotation components of pose1 and pose2
    double theta1 = pose1.rotation().angle();
    double theta2 = pose2.rotation().angle();


    // Calculate the relative rotation from pose1 to pose2
    double theta_relative = theta2 - theta1;


    // Transform the line angle to pose2's frame
    double line1_angle_in_pose2 = line1_angle + theta_relative;


    // Normalize the angle to be within [-π, π]
    line1_angle_in_pose2 = norm_angle(line1_angle_in_pose2);


    return line1_angle_in_pose2;
}


void printPoseID(const g2o::SE2 &pose, int pose_id, const std::string &label) {
    std::cout << label << " - Pose ID: " << pose_id << std::endl;
}


void printLandmarkIDs(const std::vector<g2o::VertexRhoTheta*> &landmarks, const std::string &label) {
    std::cout << label << " - Landmark IDs: ";
    for (const auto &landmark : landmarks) {
        if (landmark) {
            std::cout << landmark->id() << " ";
        }
    }
    std::cout << std::endl;
}


double calculateIdealAngle(double angle_diff, double error_threshold = M_PI/18) {
    const double ERROR = error_threshold;  // Error threshold for comparison


    // Determine the ideal angle based on the angle difference
    double ideal_angle;
    if ((angle_diff > -ERROR) && (angle_diff < ERROR)) { // parallel
        ideal_angle = 0;
    } else if ((angle_diff > M_PI / 2 - ERROR) && (angle_diff < M_PI / 2 + ERROR)) { // orthogonal
        ideal_angle = M_PI / 2;
    } else if ((angle_diff > M_PI - ERROR) && (angle_diff < M_PI + ERROR)) { // parallel
        ideal_angle = M_PI;
    } else if ((angle_diff > -M_PI) && (angle_diff < -M_PI + ERROR)) { // parallel
        ideal_angle = -M_PI;
    } else if ((angle_diff > -M_PI / 2 - ERROR) && (angle_diff < -M_PI / 2 + ERROR)) {  // orthogonal
        ideal_angle = -M_PI / 2;
    } else {
        ideal_angle = angle_diff; // Use the actual angle difference if it doesn't fit any special case
    }


    return ideal_angle;
}


Eigen::Vector2d findIntersectionPoint(const Eigen::Vector2d& line1_rhotheta, const Eigen::Vector2d& line2_rhotheta) {
    double rho1 = line1_rhotheta[0];
    double theta1 = line1_rhotheta[1];
    double rho2 = line2_rhotheta[0];
    double theta2 = line2_rhotheta[1];


    Eigen::Matrix2d A;
    A << cos(theta1), sin(theta1), cos(theta2), sin(theta2);
    Eigen::Vector2d b(rho1, rho2);


    if (std::abs(A.determinant()) < 1e-10) {
        return Eigen::Vector2d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    }


    Eigen::Vector2d intersection = A.inverse() * b;


    return intersection;
}




double calculateLandmarkDistance(const Eigen::Vector2d& line1_rhotheta, const Eigen::Vector2d& line2_rhotheta, double angle) {
    double rho1 = line1_rhotheta[0];
    double rho2 = line2_rhotheta[0];


    if (angle == M_PI || angle == -M_PI) {
        return std::abs(rho1 + rho2);
    } else if (angle == 0) {
        return std::abs(rho1 - rho2);
    } else {
        return (rho1 - rho2) / cos(angle);
    }
}




double calculateDistancePointToLandmark(const Eigen::Vector2f& point, const Eigen::Vector2d& line_rhotheta) {
    double rho = line_rhotheta[0];
    double theta = line_rhotheta[1];


    return std::abs(rho - point.x() * cos(theta) - point.y() * sin(theta));
}






// Helper function to check if point q lies on segment pr
bool onSegment(const Eigen::Vector2f &p, const Eigen::Vector2f &q, const Eigen::Vector2f &r) {
    return q.x() <= std::max(p.x(), r.x()) && q.x() >= std::min(p.x(), r.x()) &&
           q.y() <= std::max(p.y(), r.y()) && q.y() >= std::min(p.y(), r.y());
}


// Function to determine if two line segments (p1-p2 and q1-q2) intersect
bool segmentsIntersect(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2,
                       const Eigen::Vector2f &q1, const Eigen::Vector2f &q2) {
    // Helper function to check the orientation of ordered triplet (p, q, r)
    auto orientation = [](const Eigen::Vector2f &p, const Eigen::Vector2f &q, const Eigen::Vector2f &r) {
        float val = (q.y() - p.y()) * (r.x() - q.x()) -
                    (q.x() - p.x()) * (r.y() - q.y());
        if (val == 0) return 0;  // collinear
        return (val > 0) ? 1 : 2; // clock or counterclock wise
    };


    int o1 = orientation(p1, p2, q1);
    int o2 = orientation(p1, p2, q2);
    int o3 = orientation(q1, q2, p1);
    int o4 = orientation(q1, q2, p2);


    // General case
    if (o1 != o2 && o3 != o4) return true;


    // Special cases
    // p1, p2 and q1 are collinear and q1 lies on segment p1p2
    if (o1 == 0 && onSegment(p1, q1, p2)) return true;


    // p1, p2 and q2 are collinear and q2 lies on segment p1p2
    if (o2 == 0 && onSegment(p1, q2, p2)) return true;


    // q1, q2 and p1 are collinear and p1 lies on segment q1q2
    if (o3 == 0 && onSegment(q1, p1, q2)) return true;


    // q1, q2 and p2 are collinear and p2 lies on segment q1q2
    if (o4 == 0 && onSegment(q1, p2, q2)) return true;


    return false; // Doesn't fall in any of the above cases
}





double calculateLandmarkLength(const g2o::VertexRhoTheta* landmark) {
    Eigen::Vector2f start = landmark->start;
    Eigen::Vector2f end = landmark->end;
    return (start-end).norm();
}



Drone::Drone(ros::NodeHandle &nh,
             const XmlRpc::XmlRpcValue &config) : odom_prop(config["std_x"], config["std_y"], config["std_w"]),
                                                  cor_pose_pub(nh.advertise<geometry_msgs::PoseStamped>("/corrected_pose", 1)),
                                                  loop_closer(*this, config),
                                                  landmark_assoc_thresh(config["landmark_assoc_thresh"]),
                                                  landmark_max_gap(config["landmark_max_gap"]),
                                                  landmark_max_dist(config["landmark_max_dist"]),
                                                  maximum_constraints(config["maximum_constraints"]),
                                                  landmark_min_length_threshold(config["landmark_min_length_threshold"]),
                                                  max_past_landmark_ID(config["max_past_landmark_ID"]), 
                                                  max_past_landmark_dist(config["max_past_landmark_dist"]), 
                                                  min_pl_edge_count(config["min_pl_edge_count"]),
                                                  parallel_ll_info_matrix(config["parallel_ll_info_matrix"]),
                                                  orthogonal_ll_info_matrix(config["orthogonal_ll_info_matrix"]) {
    cor_pose_msg.header.frame_id = "map";
    odom_bl_tf.header.frame_id = "odom";
    odom_bl_tf.child_frame_id = "base_link";


    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    if (!tf_buffer.canTransform("map", "odom", ros::Time(0), ros::Duration(2.0)))
        abort();
    tf::transformMsgToTF(tf_buffer.lookupTransform("map", "odom", ros::Time(0)).transform, odom_map_tf);
}


static g2o::VertexRhoTheta* reference_landmark = nullptr;
static bool reference_set = false;
static double reference_angle = 0.0;
static double reference_rho = 0.0;
static ls_extractor::LineSegment reference_line;


void Drone::msgCallback(const ls_extractor::SegmentVector &bl_lines, const nav_msgs::Odometry &odom, const sensor_msgs::LaserScan &pc) {
    tf::Transform transform;
    tf::poseMsgToTF(odom.pose.pose, transform);
    transform = odom_map_tf * transform;
    auto &_translation = transform.getOrigin();
    auto &_mat = transform.getBasis();
    double _r, _p, _y;
    _mat.getRPY(_r, _p, _y);
    g2o::SE2 new_pose(_translation.getX(), _translation.getY(), _y);


   
    // Additional debug prints
    //std::cout << "New Pose: (" << new_pose[0] << ", " << new_pose[1] << ", " << new_pose[2] << ")" << std::endl;


    // Visualization debug
    //std::cout << "Translation: (" << _translation.getX() << ", " << _translation.getY() << ", " << _translation.getZ() << ")" << std::endl;
    //std::cout << "Rotation: (roll=" << _r << ", pitch=" << _p << ", yaw=" << _y << ")" << std::endl;




    if (!lm_graph.poses.size()) {  // add initial fixed pose
        table.resize(pc.ranges.size());
        for (int i = 0; i < pc.ranges.size(); i++) {
            float angle = pc.angle_min + pc.angle_increment * i;
            table[i] = {cos(angle), sin(angle)};
        }


        boost::unique_lock<boost::shared_mutex> lm_lock(lm_graph.mu);
        lm_graph.poses.emplace_back(pc, table, new_pose);
        auto *initialPose = &lm_graph.poses.back().pose;
        initialPose->setId(vertex_id++);
        initialPose->setEstimate(new_pose);
        initialPose->setFixed(true);
        lm_graph.opt.addVertex(initialPose);
        lm_graph.new_vset.insert(initialPose);


        boost::unique_lock<boost::shared_mutex> pose_lock(pose_graph.mu);
        pose_graph.poses.emplace_back();
        auto *ip = &pose_graph.poses.back().pose;
        ip->setId(initialPose->id());
        ip->setEstimate(initialPose->estimate());
        ip->setFixed(true);
        pose_graph.opt.addVertex(ip);
        prev_pose.dt = odom.header.stamp.toSec();
        prev_pose.dpose = new_pose;
        return;
    }


    

    // compute the delta pose and update prev pose
    auto delta_pose = prev_pose.dpose.inverse() * new_pose;
    odom_prop.step({odom.header.stamp.toSec() - prev_pose.dt, delta_pose});
    prev_pose.dt = odom.header.stamp.toSec();
    prev_pose.dpose = new_pose;


    // compute the corrected pose based on the last optimize pose vertex
    auto *prev_vertex = &lm_graph.poses.back().pose;
    g2o::SE2 cor_pose = prev_vertex->estimate() * odom_prop.pose;


    // update transform so it has the correction applied
    _translation.setX(cor_pose[0]);
    _translation.setY(cor_pose[1]);
    _mat.setRPY(_r, _p, cor_pose[2]);


    odom_bl_tf.header.stamp = odom.header.stamp;
    tf::transformTFToMsg(odom_map_tf.inverse() * transform, odom_bl_tf.transform);


    // Additional debug print
    //std::cout << "Corrected Pose: (" << cor_pose[0] << ", " << cor_pose[1] << ", " << cor_pose[2] << ")" << std::endl;


    br.sendTransform(odom_bl_tf);


    // ---------------- visualization--------------------
    cor_pose_msg.header.stamp = odom.header.stamp;
    cor_pose_msg.pose.position.x = _translation.getX();
    cor_pose_msg.pose.position.y = _translation.getY();
    cor_pose_msg.pose.position.z = _translation.getZ();
    tf::quaternionTFToMsg(transform.getRotation(), cor_pose_msg.pose.orientation);
    cor_pose_pub.publish(cor_pose_msg);
    // ---------------- end visualization--------------------


    // update error with the computed delta
    double disp = odom_prop.pose.translation().norm();
    if ((disp > 0.5 || std::abs(odom_prop.pose[2]) >= M_PI / 6)) {
        traveled_dist += disp;


        boost::unique_lock<boost::shared_mutex> lm_lock(lm_graph.mu);
        lm_graph.poses.emplace_back(pc, table, new_pose);
        auto *new_vertex = &lm_graph.poses.back().pose;
        auto *odom_edge = &lm_graph.poses.back().edge;


        new_vertex->setEstimate(cor_pose);
        new_vertex->setId(vertex_id++);
        lm_graph.opt.addVertex(new_vertex);
        lm_graph.new_vset.insert(new_vertex);


        odom_edge->vertices()[0] = prev_vertex;
        odom_edge->vertices()[1] = new_vertex;
        odom_edge->setMeasurement(odom_prop.pose);
        odom_edge->information().noalias() = odom_prop.cov.inverse();
        // std::cout << "pose information " << std::endl;
        // std::cout << odom_edge->information() << std::endl;
        lm_graph.opt.addEdge(odom_edge);
        lm_graph.new_eset.insert(odom_edge);


        std::vector<g2o::VertexRhoTheta*> created_landmarks; //save the set of newly created landmarks
        std::vector<std::pair<int, ls_extractor::LineSegment>> landmark_lines;


        Eigen::Vector2f trans = cor_pose.translation().cast<float>();
        Eigen::Rotation2Df rot = cor_pose.rotation().cast<float>();
       
        // Storage for a single pair of landmark and line
        //const double landmark_min_length_threshold = 3.0;


        int landmarkSizeConst = lm_graph.landmarks.size(); //constant for one pose (entire of msgCallback)
        lm_graph.line_segments.clear(); // Clear previous line segments
            
        for (const auto &line : bl_lines) {
            if (!std::isfinite(line.start_.x()) || !std::isfinite(line.start_.y()) || !std::isfinite(line.end_.x())   || !std::isfinite(line.end_.y())) {
                continue;  // safely skip lines derived from NaN lidar points
            }
            
            int landmarkSizeFluc = lm_graph.landmarks.size(); // fluctuates according to what line we're looking at
            auto *landmark = mergeLine(rot * line.start_ + trans, rot * line.end_ + trans);
            int landmarkID = landmark->id();

            
            addLandmarkObservations(landmark, line, new_vertex);

            
            for (auto &past_landmark : lm_graph.landmarks) {
               
                int pastLandmarkID = past_landmark.id();
                if ( (segmentsIntersect(landmark->start, landmark->end, past_landmark.start, past_landmark.end)
                    || areSegmentsClose(landmark->start, landmark->end, past_landmark.start, past_landmark.end, max_past_landmark_dist))
                    && (landmarkID - pastLandmarkID <= max_past_landmark_ID) && (landmarkID - pastLandmarkID > 0)
                    && (calculateLandmarkLength(&past_landmark)>landmark_min_length_threshold) && (calculateLandmarkLength(landmark)>landmark_min_length_threshold)) {
                    Eigen::Vector2d transformed_landmark = transformLandmarkToPoseFrame(&past_landmark, new_vertex->estimate());
                    double angle_diff = line.rhotheta[1] - transformed_landmark[1];
                    double ideal_angle = calculateIdealAngle(norm_angle(angle_diff));
                    std::cout << "About to do LL constraint!: " << std::endl;
                    addLandmarkLandmarkConstraint(&past_landmark, landmark, ideal_angle, line, new_vertex);
                    std::cout << "Did LL constraint!: " << std::endl;
                }
            } 
                     
        }
        
       
        odom_prop.reset();


        if (need_reinit) {
            // we cannot rely on updateInitialization when we removed some edges or it is the first time we call init/optimize
            lm_graph.opt.initializeOptimization();
            lm_graph.opt.push();
            lm_graph.opt.optimize(15, false);
            need_reinit = false;
        } else {
            lm_graph.opt.updateInitialization(lm_graph.new_vset, lm_graph.new_eset);
            lm_graph.opt.push();
            lm_graph.opt.optimize(15, true);
           
        }
       
           
                   
        // later when we need to remove edges because of rejection, this makes sure that we keep the pose-pose edge
        lm_graph.new_eset.erase(odom_edge);


       
        //std::cout << "Total number of poses: " << lm_graph.poses.size() << std::endl;

        //std::cout << "Total number of landmarks before consistency check: " << lm_graph.landmarks.size() << std::endl;

        // ────────────────────────────────────────────────────
        // new‐batch marker for final and removed error logs
        {
            std::ofstream fout1(
                "/root/sparse-gslam/src/sparse_gslam/datasets/final_ll_constraint_errors.txt",
                std::ios::out | std::ios::app
            );
            std::ofstream fout2(
                "/root/sparse-gslam/src/sparse_gslam/datasets/removed_ll_constraint_errors.txt",
                std::ios::out | std::ios::app
            );
            fout1 << "BATCH\n";
            fout2 << "BATCH\n";
        }
        // ────────────────────────────────────────────────────
        

        int dof = 0;
        for (auto *edge : lm_graph.opt.activeEdges())
            dof += static_cast<g2o::OptimizableGraph::Edge *>(edge)->dimension();
        lm_graph.opt.computeActiveErrors();
        double chi2_after = lm_graph.opt.activeChi2();

        // std::cout << chi2_before << "," << chi2_after << std::endl;
        if (chi2_after > boost::math::quantile(chi2_distribution(dof), 0.9)) {
            // std::cout << "rejecting data association" << std::endl;
           
            while (lm_graph.landmark_landmarks.size() > lm_graph.last_landmark_landmark_edge) {
                auto *edge = &lm_graph.landmark_landmarks.back();

                // ────────────────────────────────────────────────────
                // compute χ² for this constraint before removal, then append
                edge->computeError();
                Eigen::VectorXd e    = edge->error();
                Eigen::Matrix2d info    = edge->information();
                double chi2 = e.dot(info * e);

                std::ofstream fout2(
                    "/root/sparse-gslam/src/sparse_gslam/datasets/removed_ll_constraint_errors.txt",
                    std::ios::out | std::ios::app
                );
                fout2 << chi2 << "\n";
                // ────────────────────────────────────────────────────


                lm_graph.opt.removeEdge(edge);
               
                // this seems to fix the segfault problem of online optimization
                // remove any vertices with no edges connected to them
                auto *lm = edge->vertex(1);
                if (lm->edges().size() == 0)
                    lm_graph.opt.removeVertex(lm);
                removed_ll_constraints_count++;
        
                lm_graph.landmark_landmarks.pop_back();
            }
           
            while (lm_graph.pose_landmarks.size() > lm_graph.last_landmark_edge) {
                auto *edge = &lm_graph.pose_landmarks.back();
                lm_graph.opt.removeEdge(edge);


                // this seems to fix the segfault problem of online optimization
                // remove any vertices with no edges connected to them
                auto *lm = edge->vertex(1);
                if (lm->edges().size() == 0)
                    lm_graph.opt.removeVertex(lm);
                lm_graph.pose_landmarks.pop_back();
            }


            lm_graph.opt.pop();
            need_reinit = true;
        }
        else {

            std::ofstream fout2(
                "/root/sparse-gslam/src/sparse_gslam/datasets/final_ll_constraint_errors.txt",
                std::ios::out | std::ios::app
            );
            if (!fout2.is_open()) {
                std::cerr << "Error opening file for logging chi-squared errors!" << std::endl;
                return;  // Exit if the file can't be opened
            }
        
            // Start a new batch for logging
            fout2 << "BATCH\n";
        
            // Iterate through landmark_landmarks and log chi-squared errors for "good" constraints
            for (size_t i = lm_graph.last_landmark_landmark_edge; i < lm_graph.landmark_landmarks.size(); ++i) {
                auto *edge = &lm_graph.landmark_landmarks[i];
        
                // Compute χ² for this constraint
                edge->computeError();
                Eigen::VectorXd e = edge->error();
                Eigen::Matrix2d info = edge->information();
                double chi2 = e.dot(info * e);
        
                // Log the chi-squared error for the valid constraint (below the threshold)
                if (chi2_after <= boost::math::quantile(chi2_distribution(dof), 0.9)) {
                    fout2 << chi2 << "\n";  // Log the chi-squared error for good constraints
                }
            }
        
            // Close the file after logging
            fout2.close();
            
            chi2_before = chi2_after;
            lm_graph.opt.discardTop();
            for (auto &lm : lm_graph.landmarks)
                lm.updateEndpoints();
        }
        
    
        lm_graph.last_landmark_edge = lm_graph.pose_landmarks.size();
        lm_graph.last_landmark_landmark_edge = lm_graph.landmark_landmarks.size();
        lm_graph.new_vset.clear();
        lm_graph.new_eset.clear();

    } else if (delta_pose.translation().norm() > 0.01 || std::abs(delta_pose[2]) >= M_PI / 180) {
        // we add points only if the robot moves a little
        // we don't hold lock here, since other threads should never touch the latest pose
        lm_graph.poses.back().addPoints(pc, odom_prop.pose, table);
    }
    std::cout << "Number of poses: " << lm_graph.poses.size() << std::endl;
    std::cout << "Number of landmarks: " << lm_graph.landmarks.size() << std::endl;
    //std::cout << "Number of pose-landmark constraints: " << lm_graph.pose_landmarks.size() << std::endl;
    //std::cout << "Number of landmark-landmark constraints: " << lm_graph.landmark_landmarks.size() << std::endl;
    //std::cout << "Total number of loop-closing constraints: " << lm_graph.loop_closures.size() << std::endl;

    //std::cout << "Number of landmark-landmark constraints actually added: "  << added_ll_constraints_count << std::endl;
    std::cout << "Total constraints added: " << added_ll_constraints_count << std::endl;
    std::cout << "Unique constraint pairs added: " << unique_ll_constraint_pairs.size() << std::endl;
    std::cout << "Total number of removed landmark-landmark constraints due to consistency checks: " << removed_ll_constraints_count << std::endl;
    

}

void Drone::addLandmarkObservations(g2o::VertexRhoTheta *landmark, const ls_extractor::LineSegment &bl_line, g2o::VertexSE2 *new_vertex) {
   
    lm_graph.pose_landmarks.emplace_back();
    auto *pose_landmark = &lm_graph.pose_landmarks.back();
    pose_landmark->vertices()[0] = new_vertex;
    pose_landmark->vertices()[1] = landmark;
    //std::cout << "bl_line: " << bl_line << std::endl;
    pose_landmark->information().noalias() = bl_line.cov.cast<double>().inverse();
    //std::cout << "Pose-landmark info matrix " << bl_line.cov.cast<double>().inverse() << std::endl;
    pose_landmark->setMeasurement(bl_line.rhotheta.cast<double>());
    pose_landmark->start = bl_line.start_;
    pose_landmark->end = bl_line.end_;
   
    lm_graph.new_eset.insert(pose_landmark);
    // the landmark vertex might have been removed previously due to data association rejection
    if (!lm_graph.opt.vertex(landmark->id())) {
        lm_graph.new_vset.insert(landmark);
        lm_graph.opt.addVertex(landmark);
    }
   
    lm_graph.opt.addEdge(pose_landmark);




    //std::cerr << "Landmark ID at during pose-landmark " << landmark->id() << std::endl;


}


void Drone::addLandmarkLandmarkConstraint(g2o::VertexRhoTheta *landmark1, g2o::VertexRhoTheta *landmark2, double idealAngle, const ls_extractor::LineSegment &line2, g2o::VertexSE2 *new_vertex) { //angle1 is the angle of line1 seen from the frame of new_vertex
    int pose_landmark_edge_count = 0;


    for (auto edge : landmark1->edges()) {
        // Check if the edge is a pose-landmark edge
        if (dynamic_cast<g2o::EdgeSE2RhoTheta*>(edge)) {
            pose_landmark_edge_count++;
        }
    }
    // If there are fewer than 7 pose-landmark edges, don't add the landmark-landmark constraint
    if (pose_landmark_edge_count < min_pl_edge_count) {
        return;
    }


   
   
    int id1 = landmark1->id();
    int id2 = landmark2->id();
   
    std::cout << "Past: " << id1 << " Current: " << id2 <<std::endl;
                   
    // Ensure that id1 is always less than id2 for consistent ordering
    auto constraint_pair = makeOrderedPair(id1, id2);

    
    // Check if the number of constraints between these landmarks exceeds the limit
    if (constraint_count[constraint_pair] >= MAX_CONSTRAINTS) {
        return;  // Skip adding a new constraint if the limit is reached
    }


    if (!lm_graph.opt.vertex(landmark1->id())) {
        std::cerr << "Vertex ID " << landmark1->id() << " not found in optimizer!" << std::endl;
        return;
    }
    if (!lm_graph.opt.vertex(landmark2->id())) {
        std::cerr << "Vertex ID " << landmark2->id() << " not found in optimizer!" << std::endl;
        return;
    }


    double length2 = (line2.end_ - line2.start_).norm();
    lm_graph.landmark_landmarks.emplace_back();
    auto *landmark_landmark = &lm_graph.landmark_landmarks.back();


    // Set the vertices of the edge
    landmark_landmark->vertices()[0] = landmark1;
    landmark_landmark->vertices()[1] = landmark2;
   
   
    double past_theta = landmark1->estimate()[1];
    double current_theta = landmark2->estimate()[1];
    double angle_diff = norm_angle(current_theta-past_theta);
    double ideal_angle = calculateIdealAngle(angle_diff);    
    double distance_using_rhos;
    double distance_using_points;
    Eigen::Matrix2d mat;
    double intensity = (landmark1->start - landmark1->end).norm() + (landmark2->start - landmark2->end).norm();
    mat(0,0) = intensity;
    mat(0,1) = 0;
    mat(1,0) = 0;
    mat(1,1) = intensity;
    if (ideal_angle == M_PI || ideal_angle == -M_PI || ideal_angle == 0) { // parallel lines
        std::cout << "parallel! " << std::endl;
        landmark_landmark->information().noalias() = mat*parallel_ll_info_matrix;
        landmark_landmark->setMeasurement(Eigen::Vector2d(0, ideal_angle+past_theta));
        // Set the start and end of the landmark-landmark constraint
        landmark_landmark->start = line2.start_;
        landmark_landmark->end = line2.end_;
        lm_graph.new_eset.insert(landmark_landmark);
        if (!lm_graph.opt.vertex(landmark2->id())) {
            lm_graph.new_vset.insert(landmark2);
            lm_graph.opt.addVertex(landmark2);
        }
        
        lm_graph.opt.addEdge(landmark_landmark);
        // Mark both landmarks as having a landmark-landmark constraint
        constraint_count[constraint_pair]++;

        added_ll_constraints_count++;

        unique_ll_constraint_pairs.insert(makeOrderedPair(landmark1->id(), landmark2->id()));


    }
    else if (ideal_angle == M_PI/2 || ideal_angle == -M_PI/2) { // orthogonal lines
        std::cout << "Orthogonal" << std::endl;
        landmark_landmark->information().noalias() = mat*orthogonal_ll_info_matrix;
        landmark_landmark->setMeasurement(Eigen::Vector2d(0, ideal_angle+past_theta));
        // Set the start and end of the landmark-landmark constraint
        landmark_landmark->start = line2.start_;
        landmark_landmark->end = line2.end_;
        lm_graph.new_eset.insert(landmark_landmark);
        if (!lm_graph.opt.vertex(landmark2->id())) {
            lm_graph.new_vset.insert(landmark2);
            lm_graph.opt.addVertex(landmark2);
        }
        lm_graph.opt.addEdge(landmark_landmark);
        // Mark both landmarks as having a landmark-landmark constraint
        constraint_count[constraint_pair]++;

        added_ll_constraints_count++;

        unique_ll_constraint_pairs.insert(makeOrderedPair(landmark1->id(), landmark2->id()));

    }
    else {
        return;
    }
   
}




// give a line segment's start point and end point, returns the landmark that best matches this segment
g2o::VertexRhoTheta *Drone::mergeLine(const Eigen::Vector2f &start, const Eigen::Vector2f &end) {
    g2o::VertexRhoTheta *closest = nullptr;
    double error = std::numeric_limits<float>::infinity();
    double threshold_length = 25.0;

    lm_graph.line_segments.emplace_back();  // Add to line_segments
    auto &line_segment = lm_graph.line_segments.back();  // Reference the newly added line segment
    // Set the rho-theta values for the new line segment
    line_segment.setId(lineSegment_id++);
    line_segment.setEstimate(ls_extractor::topolar<float>(start, end).cast<double>());
    line_segment.start = start;
    line_segment.end = end;
    std::cout << line_segment.id() << std::endl;
    
    
    double rho = line_segment.estimate()[0];
    double theta = line_segment.estimate()[1];
    // Extract start and end points of the line segment
    //std::cout << "Line Segment: " << std::endl;
    //std::cout << "  Start: (" << line_segment.start.x() << ", " << line_segment.start.y() << ")" << std::endl;
    //std::cout << "  End: (" << line_segment.end.x() << ", " << line_segment.end.y() << ")" << std::endl;
    //std::cout << "  Rho: " << rho << std::endl;
    //std::cout << "  Theta: " << theta << std::endl;
    
    
    for (auto &landmark : lm_graph.landmarks) {
       
        if (traveled_dist - landmark.dist >= landmark_max_dist)
            continue;
       
        double landmark_length = (landmark.end - landmark.start).norm();
        
        if (landmark_length > threshold_length) {
            continue;
        }
        


        Eigen::Vector2f t_p, t_l;
        Eigen::Vector2f lm_rhotheta = landmark.estimate().cast<float>();
        ls_extractor::calc_endpoints<float>(lm_rhotheta, landmark.start, landmark.end, t_l);
        float e = ls_extractor::ll_distance<float>(lm_rhotheta, start, end, t_p);
        if (e < error && !(t_l[0] > t_p[1] + landmark_max_gap || t_l[1] + landmark_max_gap < t_p[0])) {
            closest = &landmark;
            line_segment.setId(landmark.id());
            error = e;
        }
    }


    //  && abs(norm_angle(closest->estimate()[1] - rhotheta[1])) < M_PI / 6
    if (error > landmark_assoc_thresh) {
        // if suspect implicit loop closure, allow a higher error margin
        if (closest && error < 1.0f) {
            if (traveled_dist - closest->dist > 15.0 && traveled_dist - closest->dist < landmark_max_dist) {
                return closest;
            }
        }
        // error above threshold: create a new landmark
        lm_graph.landmarks.emplace_back();
        auto &line = lm_graph.landmarks.back();
        line.dist = traveled_dist;
        line.setId(landmark_id++);
        line.setEstimate(ls_extractor::topolar<float>(start, end).cast<double>());
        line.start = start;
        line.end = end;
        lm_graph.new_vset.insert(&line);
        lm_graph.opt.addVertex(&line);
        return &line;
    } else {
        closest->dist = traveled_dist;
        return closest;
    }
}


void Drone::submap_matching() {
    while (ros::ok()) {
        loop_closer.precompute();
        loop_closer.match();
    }
}





