#pragma once
#include "g2o/core/base_binary_edge.h" 
#include "g2o/core/hyper_graph_action.h" 
#include "g2o/core/factory.h" 
#include "g2o/stuff/macros.h" 
#include "g2o/types/slam2d/vertex_se2.h" 
#include "g2o_bindings/vertex_rhotheta.h"

namespace g2o {

class EdgeRhoThetaRhoTheta : public BaseBinaryEdge<2, Eigen::Vector2d, VertexRhoTheta, VertexRhoTheta> {
    public:
    Eigen::Vector2f start, end;
    std::vector<Eigen::Vector2f> start_points;
    std::vector<Eigen::Vector2f> end_points;

    void addStartEndPoint(const Eigen::Vector2f& start, const Eigen::Vector2f& end) {
        start_points.push_back(start);
        end_points.push_back(end);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeRhoThetaRhoTheta() = default;
    void computeError() override;

    virtual bool read(std::istream& is) override;
    virtual bool write(std::ostream& os) const override;
};

}  // namespace g2o