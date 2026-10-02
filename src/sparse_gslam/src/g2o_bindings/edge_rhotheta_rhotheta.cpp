#include "g2o_bindings/edge_rhotheta_rhotheta.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"


double norm_angle(double angle) {
   if (angle > M_PI) {
       return angle - 2 * M_PI;
   } else if (angle < -M_PI) {
       return angle + 2 * M_PI;
   } else {
       return angle;
   }
}

double calculateIdealAngle(double angle_diff, double error_threshold = M_PI/18) { // threshold for angle gap
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
Eigen::Vector2d findIntersectionPoint(const g2o::VertexRhoTheta* landmark1, const g2o::VertexRhoTheta* landmark2) {
    // Extract the rho and theta values for both landmarks
    double rho1 = landmark1->estimate()[0];
    double theta1 = landmark1->estimate()[1];
    double rho2 = landmark2->estimate()[0];
    double theta2 = landmark2->estimate()[1];

    // Calculate the intersection point using the rho and theta values
    Eigen::Matrix2d A;
    A << cos(theta1), sin(theta1), cos(theta2), sin(theta2);
    Eigen::Vector2d b(rho1, rho2);

    // If the determinant is near zero, the lines are parallel
    if (std::abs(A.determinant()) < 1e-10) {
        return Eigen::Vector2d(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
    }

    // Solve the linear system to get the intersection point
    Eigen::Vector2d intersection = A.inverse() * b;

    return intersection;
}

double calculateLandmarkDistance(const g2o::VertexRhoTheta* landmark1, const g2o::VertexRhoTheta* landmark2, double angle) {
    double rho1 = landmark1->estimate()[0];
    double rho2 = landmark2->estimate()[0];

    // If angle is M_PI or -M_PI, use the sum of rhos; if 0, use the difference
    if (angle == M_PI || angle == -M_PI) {
        return std::abs(rho1 + rho2);
    } else if (angle == 0) {
        return std::abs(rho1 - rho2);
    } else {
        return (rho1 - rho2) / cos(angle);
    }
}

double calculateDistancePointToLandmark(const Eigen::Vector2f point, const g2o::VertexRhoTheta* landmark) {
    double rho = landmark->estimate()[0];
    double theta = landmark->estimate()[1];

    // Distance formula from a point to a line in polar coordinates
    return std::abs(rho - point.x() * cos(theta) - point.y() * sin(theta));
}

/*
Eigen::Vector2d Drone::findClosestPointOnLandmark(const g2o::SE2& pose, const g2o::VertexRhoTheta* landmark) {
    double rho = landmark->estimate()[0];
    double theta = landmark->estimate()[1];

    // Position of the pose
    Eigen::Vector2d pose_position = pose.translation();

    // Find the point on the line closest to the pose
    double x = pose_position.x();
    double y = pose_position.y();
    double d = rho - (x * cos(theta) + y * sin(theta));
    Eigen::Vector2d closest_point(x + d * cos(theta), y + d * sin(theta));

    return closest_point;
}
*/

Eigen::Vector2f findClosestPointToOrigin(const g2o::VertexRhoTheta* landmark) {
    // Extract the rho and theta values from the landmark
    double rho = landmark->estimate()[0];
    double theta = landmark->estimate()[1];
   
    // Calculate the x and y coordinates of the closest point
    float x = rho * std::cos(theta);
    float y = rho * std::sin(theta);
   
    // Return the point as an Eigen::Vector2f
    return Eigen::Vector2f(x, y);
}

namespace g2o {

void EdgeRhoThetaRhoTheta::computeError() {
    const auto* landmark1 = static_cast<VertexRhoTheta*>(_vertices[0]);
    const auto* landmark2 = static_cast<VertexRhoTheta*>(_vertices[1]);
    Eigen::Vector2d rhotheta1 = landmark1->estimate();
    Eigen::Vector2d rhotheta2 = landmark2->estimate();
    
    double angle_diff = norm_angle(rhotheta2[1] - rhotheta1[1]);
    double ideal_angle = calculateIdealAngle(angle_diff);
    
    double distance_using_rhos;
    double distance_using_points;
    
    if (ideal_angle == M_PI || ideal_angle == -M_PI || ideal_angle == 0) { // parallel landmarks 
        distance_using_rhos = calculateLandmarkDistance(landmark1, landmark2, ideal_angle); // distance between landmarks using the sum of rhos
        distance_using_points = calculateDistancePointToLandmark(findClosestPointToOrigin(landmark1), landmark2); // finds a point on landmark1 closest to the pose. Finds distance between point and landmark2
    }
    else if (ideal_angle == M_PI/2 || ideal_angle == -M_PI/2) { // orthogonal landmarks
        distance_using_rhos = std::sqrt(rhotheta1[0]*rhotheta1[0] + rhotheta2[0]*rhotheta2[0]); // sqrt (rho1^2+rho2^2); 
        distance_using_points = findIntersectionPoint(landmark1, landmark2).norm(); // finds intersection point and the distance between the pose and the point
        
    }
    
    _error[0] = _measurement[0] - 0;
    _error[1] = _measurement[1] - rhotheta2[1];
    //_error[1] = rhotheta1[1] - rhotheta2[1]-ideal_angle;
}

bool EdgeRhoThetaRhoTheta::read(std::istream& is) {
   return true;
}

bool EdgeRhoThetaRhoTheta::write(std::ostream& os) const {
   return true;
}

G2O_REGISTER_TYPE(EDGE_RHOTHETA_RHOTHETA, EdgeRhoThetaRhoTheta);
}  // namespace g2o
