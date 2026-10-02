#include "g2o_bindings/vertex_rhotheta.h"
#include "g2o_bindings/edge_se2_rhotheta.h"
#include "g2o_bindings/edge_rhotheta_rhotheta.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "ls_extractor/utils.h"
#include "g2o/core/factory.h"
#include "g2o/stuff/macros.h"

namespace g2o {

void VertexRhoTheta::updateEndpoints() {
    if (_edges.size() == 0)
        return;

    // Clear the vectors to prepare for new data
    start_points.clear();
    end_points.clear();

    Eigen::Vector2d start, dir;
    ls_extractor::calc_start_dir<double>(_estimate, start, dir);
    Eigen::Vector2d t{1e10, -1e10};

    for (auto* e : _edges) {
        auto* edge = dynamic_cast<g2o::EdgeSE2RhoTheta*>(e);
        if (edge) {
            auto* pose = static_cast<g2o::VertexSE2*>(edge->vertex(0));
            Eigen::Vector2d tout;

            // Compute the start and end points for this edge
            ls_extractor::calc_endpoints<double>(
                start, dir,
                pose->estimate() * edge->start.cast<double>(),
                pose->estimate() * edge->end.cast<double>(),
                tout
            );

            // Store the computed points in the vectors
            start_points.push_back((start + tout[0] * dir).cast<float>());
            end_points.push_back((start + tout[1] * dir).cast<float>());

            t[0] = std::min(t[0], tout[0]);
            t[1] = std::max(t[1], tout[1]);
        }
    }

    // Optionally, update the single start and end members
    this->start = (start + t[0] * dir).cast<float>();
    this->end = (start + t[1] * dir).cast<float>();
}

void VertexRhoTheta::setToOriginImpl() {
    _estimate.setZero();
}

void VertexRhoTheta::oplusImpl(const double* update) {
    _estimate[0] += update[0];
    _estimate[1] += update[1];
    normalize_theta(_estimate[1]);
}

bool VertexRhoTheta::read(std::istream& is) {
    // Implement reading logic if necessary, currently just returning true
    return true;
}

bool VertexRhoTheta::write(std::ostream& os) const {
    // Implement writing logic if necessary, currently just checking the stream state
    return os.good();
}

G2O_REGISTER_TYPE(VERTEX_RHOTHETA, VertexRhoTheta);

}  // namespace g2o


/*
namespace g2o {

void VertexRhoTheta::updateEndpoints() {
    if (_edges.size() == 0)
        return;
    Eigen::Vector2d start, dir;
    ls_extractor::calc_start_dir<double>(_estimate, start, dir);
    Eigen::Vector2d t{1e10, -1e10};
    // ls_extractor::calc_endpoints<double>(start, dir, this->start.cast<double>(), this->end.cast<double>(), t);
    for (auto* e : _edges) {
         Eigen::Vector2d tout;
        if (auto* edge = dynamic_cast<g2o::EdgeSE2RhoTheta*>(e)) {
            
            auto* pose = static_cast<g2o::VertexSE2*>(edge->vertex(0));
            ls_extractor::calc_endpoints<double>(start, dir, pose->estimate() * edge->start.cast<double>(), pose->estimate() * edge->end.cast<double>(), tout);
            t[0] = std::min(t[0], tout[0]);
            t[1] = std::max(t[1], tout[1]);
        }
        else if (auto* edge = dynamic_cast<g2o::EdgeRhoThetaRhoTheta*>(e)) {
            
            auto* other_landmark = static_cast<g2o::VertexRhoTheta*>(edge->vertex(0) == this ? edge->vertex(1) : edge->vertex(0));
            Eigen::Vector2d other_start, other_end;
            ls_extractor::calc_start_dir<double>(other_landmark->estimate(), other_start, dir);
            ls_extractor::calc_endpoints<double>(start, dir, other_start, other_end, tout);
            t[0] = std::min(t[0], tout[0]);
            t[1] = std::max(t[1], tout[1]);
        }
    }
    this->start = (start + t[0] * dir).cast<float>();
    this->end = (start + t[1] * dir).cast<float>();
}

void VertexRhoTheta::setToOriginImpl() { _estimate.setZero(); }

void VertexRhoTheta::oplusImpl(const double* update) {
    _estimate[0] += update[0];
    _estimate[1] += update[1];
    normalize_theta(_estimate[1]);
}

bool VertexRhoTheta::read(std::istream& is) {
    return true;
}

bool VertexRhoTheta::write(std::ostream& os) const {
    return os.good();
}
G2O_REGISTER_TYPE(VERTEX_RHOTHETA, VertexRhoTheta);
}  // namespace g2o
*/