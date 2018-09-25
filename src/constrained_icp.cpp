//
// Created by visionlab on 2/19/18.
//

#include "constrained_icp.h"
#include <Eigen/Geometry>

namespace three
{
namespace cicp
{

double TransformationEstimationPointToPoint4DoF::ComputeRMSE(
    const PointCloud &source, const PointCloud &target,
    const CorrespondenceSet &corres) const
{
    if (corres.empty()) return 0.0;
    double err = 0.0;
    for (const auto &c : corres) {
        err += (source.points_[c[0]] - target.points_[c[1]]).squaredNorm();
    }
    return std::sqrt(err / (double)corres.size());
}

Eigen::Matrix4d TransformationEstimationPointToPoint4DoF::ComputeTransformation(
    const PointCloud &source, const PointCloud &target,
    const CorrespondenceSet &corres) const
{
    if (corres.empty()) return Eigen::Matrix4d::Identity();
    Eigen::MatrixXd source_mat(3, corres.size());
    Eigen::MatrixXd target_mat(3, corres.size());
    for (size_t i = 0; i < corres.size(); i++) {
        source_mat.block<3, 1>(0, i) = source.points_[corres[i][0]];
        target_mat.block<3, 1>(0, i) = target.points_[corres[i][1]];
    }
    return Eigen::umeyama(source_mat, target_mat, with_scaling_);
}

}   // namespace cicp

}   // namespace three
