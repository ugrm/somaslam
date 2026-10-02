#include "g2o_bindings/edge_fake_rhotheta_rhotheta.h"

#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"
#include "ls_extractor/utils.h"

namespace g2o {

void EdgeFakeRhoThetaRhoTheta::computeError() {
    double predicted_angle = landmark2->estimate()[1] - landmark1->estimate()[1];
    //double ideal_angle = _measurement[1];
    double angle_diff = norm_angle(predicted_angle);

    double ideal_angle;
    if (angle_diff > -ERROR && angle_diff < ERROR) {
        ideal_angle = 0;
    } else if (angle_diff > M_PI / 2 - ERROR && angle_diff < M_PI / 2 + ERROR) {
        ideal_angle = M_PI / 2;
    } else if (angle_diff > M_PI - ERROR && angle_diff < M_PI) {
        ideal_angle = M_PI;
    } else if (angle_diff > -M_PI && angle_diff < -M_PI + ERROR) {
        ideal_angle = -M_PI;
    } else if (angle_diff > -M_PI / 2 - ERROR && angle_diff < -M_PI / 2 + ERROR) {
        ideal_angle = -M_PI / 2;
    } else {
        ideal_angle = angle_diff; // Use the actual angle difference if it doesn't fit any special case
    }

    //std::cout << "Ideal angle: " << ideal_angle/M_PI << ", Angle difference: " << angle_diff/M_PI << std::endl;


    // Set the error to be the difference between the actual angle difference and the ideal angle
    _error[0]= 0.0; 
    _error[1]= angle_diff - ideal_angle;

}

bool EdgeFakeRhoThetaRhoTheta::read(std::istream& is) {
    return true;
}
bool EdgeFakeRhoThetaRhoTheta::write(std::ostream& os) const {
    return true;
}
G2O_REGISTER_TYPE(EDGE_FAKE_RHOTHETA_RHOTHETA, EdgeFakeRhoThetaRhoTheta);
}  // namespace g2o