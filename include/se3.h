//
// Created by feixh on 10/29/18.
//
#pragma once

#include "rodrigues.h"

namespace feh {

    template<typename T>
    class SO3Type {
    public:
        using BaseType = Eigen::Matrix<T, 3, 3>;
        using PointType = Eigen::Matrix<T, 3, 1>;
        using AxisType = PointType;

        SO3Type() : R_{BaseType::Identity()} {}

        SO3Type(const BaseType &R) : R_{R} {}

        SO3Type(const AxisType &axis, T angle) :
                R_{rodrigues(AxisType{axis / axis.norm() * angle})} {}

        SO3Type operator*(const SO3Type &other) {
            return BaseType{R_ * other.matrix()};
        }

        PointType operator*(const PointType &v) {
            return PointType{R_ * v};
        }

        SO3Type inv() const {
            return BaseType{R_.transpose()};
        }

        BaseType matrix() const {
            return R_;
        }

        AxisType log() const {
            return invrodrigues(R_);
        }

//        static AxisType log(const SO3Type &R) {
//            return invrodrigues(R.matrix());
//        }

        static SO3Type exp(const AxisType &w) {
            return rodrigues(w);
        }

        static SO3Type fitToSO3(const BaseType &R_approx) {
            Eigen::JacobiSVD<BaseType> svd(R_approx, Eigen::ComputeThinU | Eigen::ComputeThinV);
            return BaseType{svd.matrixU() * BaseType::Identity() * svd.matrixV().transpose()};
        }

    private:
        BaseType R_;
    };

    template<typename Type>
    class SE3Type {
    public:
        using PointType = Eigen::Matrix<Type, 3, 1>;

        SE3Type() : R_{}, T_{0, 0, 0} {}

        SE3Type(const Eigen::Matrix<Type, 4, 4> &RT):
                R_{RT.template block<3, 3>(0, 0)},
                T_{RT.template block<3, 1>(0, 3)} {}

        SE3Type(const SO3Type<Type> &R, const Eigen::Matrix<Type, 3, 1> &T) : R_{R}, T_{T} {}

        SE3Type operator*(const SE3Type &other) {
            return {R_ * other.so3(), this->so3() * other.translation() + T_};
        }

        PointType operator*(const PointType &v) {
            return R_ * v + T_;
        };

        SE3Type inv() const {
            return {R_.inv(), -(R_.inv() * T_)};
        }

        SO3Type<Type> so3() const {
            return R_;
        }

        SO3Type<Type>& so3() {
            return R_;
        }

        PointType translation() const {
            return T_;
        }

        PointType& translation() {
            return T_;
        }

        Eigen::Matrix<Type, 3, 4> matrix3x4() const {
            return (Eigen::Matrix<Type, 3, 4>{} <<
                                                R_.matrix(), T_).finished();
        }

        Eigen::Matrix<Type, 4, 4> matrix() const {
            Eigen::Matrix<Type, 4, 4> out;
            out.setIdentity();
            out.template block<3, 4>(0, 0) = matrix3x4();
            return out;
        };

    private:
        SO3Type<Type> R_;
        Eigen::Matrix<Type, 3, 1> T_;

    };

}
