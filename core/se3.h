//
// Created by feixh on 10/29/18.
//
#pragma once

#include "rodrigues.h"

namespace feh {

template<typename Type>
class SO3Type {
public:
    using BaseType = Eigen::Matrix<Type, 3, 3>;
    using PointType = Eigen::Matrix<Type, 3, 1>;
    using AxisType = PointType;

    SO3Type() : R_{BaseType::Identity()} {}

    SO3Type(const Eigen::Matrix<Type, 3, 3> &R):R_{R} {}

    SO3Type(const AxisType &axis, Type angle) :
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

    template <typename TT>
    SO3Type<TT> cast() const {
      return R_.template cast<TT>();
    }

    // factory methods
    template <typename Derived>
    static SO3Type from_matrix(const Eigen::MatrixBase<Derived> &other) {
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 3);
      Eigen::Matrix<Type, 3, 3> R = other.template cast<Type>();
      return R;
    }

private:
    BaseType R_;
};

template<typename Type>
class SE3Type {
public:
    using PointType = Eigen::Matrix<Type, 3, 1>;

    SE3Type() : R_{}, T_{0, 0, 0} {}

    /*
    SE3Type(const Eigen::Matrix<Type, 3, 4> &RT):
            R_{RT.template block<3, 3>(0, 0)},
            T_{RT.template block<3, 1>(0, 3)} {}
    */

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

    template <typename TT>
    SE3Type<TT> cast() {
      return SE3Type<TT>(R_.cast<TT>(), T_.cast<TT>());
    }

    // factory methods
    template <typename Derived>
    static SE3Type from_matrix3x4(const Eigen::MatrixBase<Derived> &other) {
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, 3, 4);
      Eigen::Matrix<Type, 3, 3> R = other.template block<3, 3>(0, 0).template cast<Type>();
      Eigen::Matrix<Type, 3, 1> T = other.template block<3, 1>(0, 3).template cast<Type>();
      return {R, T};
    }

    template <typename Derived1, typename Derived2>
    static SE3Type from_RT(const Eigen::MatrixBase<Derived1> &Rin, 
        const Eigen::MatrixBase<Derived2> &Tin) {
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived1, 3, 3);
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived2, 3, 1);

      Eigen::Matrix<Type, 3, 3> R = Rin.template cast<Type>();
      Eigen::Matrix<Type, 3, 1> T = Tin.template cast<Type>();
      return {R, T};
    }

private:
    SO3Type<Type> R_;
    Eigen::Matrix<Type, 3, 1> T_;

};

}
