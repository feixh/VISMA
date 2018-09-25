//
// Created by visionlab on 2/19/18.
//
// ICP with constraints.
#pragma once

#include "Core/Core.h"

namespace three
{
namespace cicp {

/// Estimate a transformation for point to point distance
class TransformationEstimationPointToPoint4DoF: public TransformationEstimation
{
public:
    TransformationEstimationPointToPoint4DoF(bool with_scaling = false) :
        with_scaling_(with_scaling) {}
    ~TransformationEstimationPointToPoint4DoF() override {}

public:
    double ComputeRMSE(const PointCloud &source, const PointCloud &target,
                       const CorrespondenceSet &corres) const override;
    Eigen::Matrix4d ComputeTransformation(const PointCloud &source,
                                          const PointCloud &target,
                                          const CorrespondenceSet &corres) const override;

public:
    bool with_scaling_ = false;
};

}   // namespace cicp

}   // namespace three
