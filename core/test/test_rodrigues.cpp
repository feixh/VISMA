#include <iostream>

#include "gtest/gtest.h"
#include "Eigen/Dense"

#include "rodrigues.h"

using ftype = double;
const ftype eps = 1e-8;

using namespace feh;

namespace {

class MatrixDifferentialTest : public ::testing::Test {
protected:
    MatrixDifferentialTest() {
        _A.setRandom();
        _B.setRandom();
    }
    ~MatrixDifferentialTest() override {}
    void SetUp() override {}
    void TearDown() override {}

public:
    Eigen::Matrix<ftype, 3, 4> _A;
    Eigen::Matrix<ftype, 4, 5> _B;
};
}

TEST_F(MatrixDifferentialTest, dAB_dA) {
    // setup the differential operator
    auto diff = dAB_dA(_A, _B);

    // now let's first compute derivative of C w.r.t. each component of A
    auto C = _A * _B;
    Eigen::Matrix<ftype, 15, 12> num_diff;
    num_diff.setZero();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            auto Ap(_A);
            Ap(i, j) += eps;
            Eigen::Matrix<ftype, 3, 5> D = (Ap * _B - C) / eps;
            num_diff.col(i*4+j) = Eigen::Map<Eigen::Matrix<ftype, 15, 1>>(D.data());
        }
    }
    ASSERT_LE((diff - num_diff).norm(), 1e-3) << "inconsistent analytical & numerical derivatives";
}

TEST_F(MatrixDifferentialTest, dAB_dB) {
    // setup the differential operator
    auto diff = dAB_dB(_A, _B);

    // now let's first compute derivative of C w.r.t. each component of A
    auto C = _A * _B;
    Eigen::Matrix<ftype, 15, 20> num_diff;
    num_diff.setZero();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 5; ++j) {
            auto Bp(_B);
            Bp(i, j) += eps;
            Eigen::Matrix<ftype, 3, 5> D = (_A * Bp - C) / eps;
            num_diff.col(i*5+j) = Eigen::Map<Eigen::Matrix<ftype, 15, 1>>(D.data());
        }
    }
    ASSERT_LE((diff - num_diff).norm(), 1e-3) << "inconsistent analytical & numerical derivatives";
}

TEST_F(MatrixDifferentialTest, dhat) {
    Eigen::Matrix<ftype, 3, 1> u;
    u.setRandom();
    Eigen::Matrix<ftype, 9, 1> product = dhat(u) * u;
    auto a = Eigen::Map<Eigen::Matrix<ftype, 3, 3>>(product.data());
    // c=a.T=-a
    auto c = a.transpose();
    auto b = hat(u);
    ASSERT_LE((a-b).norm(), 1e-10);
    ASSERT_LE((c+b).norm(), 1e-10);
}

TEST_F(MatrixDifferentialTest, dAt_dA) {
    Eigen::Matrix<ftype, 4, 4> A;
    A.setRandom();
    auto At = A.transpose();
    Eigen::Matrix<ftype, 16, 1> D = dAt_dA(A) * Eigen::Map<Eigen::Matrix<ftype, 16, 1>>(A.data());
    auto at = Eigen::Map<Eigen::Matrix<ftype, 4, 4>>(D.data());
    ASSERT_LE((At-at).norm(), 1e-10);

}

TEST_F(MatrixDifferentialTest, rodrigues) {
    Eigen::Matrix<ftype, 3, 1> w;
    w.setRandom();
    Eigen::Matrix<ftype, 9, 3> dR_dw;
    auto R = rodrigues(w, &dR_dw);
    auto RRt = R * R.transpose();
    ASSERT_LE((Eigen::Matrix<ftype, 3, 3>::Identity() - RRt).norm(), 1e-5);
    // std::cout << R << std::endl;
    // std::cout << "~~~~~~~~~~" << std::endl;

    Eigen::Matrix<ftype, 9, 3> num_dR_dw;
    num_dR_dw.setZero();
    for (int i = 0; i < 3; ++i) {
        Eigen::Matrix<ftype, 3, 1> wp = w;
        wp(i) += eps;
        num_dR_dw.col(i) = Eigen::Map<Eigen::Matrix<ftype, 9, 1>>(
            Eigen::Matrix<ftype, 3, 3>{(rodrigues(wp) - R) / eps}.data());
    }
    // std::cout << dR_dw << std::endl;
    // std::cout << "==========" << std::endl;
    // std::cout << num_dR_dw << std::endl;
    ASSERT_LE((num_dR_dw - dR_dw).norm(), 1e-5);
}

TEST_F(MatrixDifferentialTest, rodrigues_small_angle) {
    Eigen::Matrix<ftype, 3, 1> w;
    w.setRandom();
    w /= 1e10;
    Eigen::Matrix<ftype, 9, 3> dR_dw;
    auto R = rodrigues(w, &dR_dw);
    auto RRt = R * R.transpose();
    ASSERT_LE((Eigen::Matrix<ftype, 3, 3>::Identity() - RRt).norm(), 1e-5);
    // std::cout << R << std::endl;
    // std::cout << "~~~~~~~~~~" << std::endl;

    Eigen::Matrix<ftype, 9, 3> num_dR_dw;
    num_dR_dw.setZero();
    for (int i = 0; i < 3; ++i) {
        Eigen::Matrix<ftype, 3, 1> wp = w;
        wp(i) += eps;
        num_dR_dw.col(i) = Eigen::Map<Eigen::Matrix<ftype, 9, 1>>(
            Eigen::Matrix<ftype, 3, 3>{(rodrigues(wp) - R) / eps}.data());
    }
    // std::cout << dR_dw << std::endl;
    // std::cout << "==========" << std::endl;
    // std::cout << num_dR_dw << std::endl;
    ASSERT_LE((num_dR_dw - dR_dw).norm(), 1e-5);
}


TEST_F(MatrixDifferentialTest, invrodrigues) {
    Eigen::Matrix<ftype, 3, 1> w;
    w.setRandom();
    Eigen::Matrix<ftype, 3, 3> R = rodrigues(w);

    Eigen::Matrix<ftype, 3, 9> dw_dR;
    w = invrodrigues(R, &dw_dR);

    Eigen::Matrix<ftype, 3, 9> num_dw_dR;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Eigen::Matrix<ftype, 3, 3> Rp(R);
            // In theory, rotation matrix + delta is not necessarily a rotation matrix.
            Rp(i, j) += eps;
            Eigen::Matrix<ftype, 3, 1> wp = invrodrigues(Rp);
            num_dw_dR.col(i*3+j) = (wp - w) / eps;
        }
    }
    // std::cout << dw_dR << std::endl;
    // std::cout << "==========" << std::endl;
    // std::cout << num_dw_dR << std::endl;
    ASSERT_LE((dw_dR - num_dw_dR).norm(), 1e-5);
}


TEST_F(MatrixDifferentialTest, invrodrigues_small_angle) {
    Eigen::Matrix<ftype, 3, 1> w;
    w.setIdentity();
    w *= eps;
    Eigen::Matrix<ftype, 3, 3> R = rodrigues(w);
    // std::cout << R << std::endl;
    // std::cout << "~~~~~~~~~~" << std::endl;

    Eigen::Matrix<ftype, 3, 9> dw_dR;
    w = invrodrigues(R, &dw_dR);

    Eigen::Matrix<ftype, 3, 9> num_dw_dR;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Eigen::Matrix<ftype, 3, 3> Rp(R);
            // In theory, rotation matrix + delta is not necessarily a rotation matrix.
            Rp(i, j) += eps;
            Eigen::Matrix<ftype, 3, 1> wp = invrodrigues(Rp);
            num_dw_dR.col(i*3+j) = (wp - w) / eps;
        }
    }
    // std::cout << dw_dR << std::endl;
    // std::cout << "==========" << std::endl;
    // std::cout << num_dw_dR << std::endl;
    ASSERT_LE((dw_dR - num_dw_dR).norm(), 1e-5);
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
