#pragma once
#include "Eigen/Dense"
#include "math.h"

template <typename T>
Eigen::Matrix<T, 3, 3> hat(const Eigen::Matrix<T, 3, 1> &u) {
    return (Eigen::Matrix<T, 3, 3>{} 
            << 0, -u(2), u(1),
               u(2), 0, -u(0),
              -u(1), u(0), 0).finished();
}

template <typename T>
Eigen::Matrix<T, 9, 3> dhat() {
    return (Eigen::Matrix<T, 9, 3>{} 
            << 0,  0,  0,
               0,  0, -1,
               0,  1,  0,
               0,  0,  1,
               0,  0,  0,
               -1, 0,  0,
               0, -1,  0,
               1,  0,  0,
               0,  0,  0).finished();
}

template <typename T>
Eigen::Matrix<T, 9, 3> dhat(const Eigen::Matrix<T, 3, 1> &u) {
    return dhat<T>();
}

template <typename T>
Eigen::Matrix<T, 3, 1> vee(const Eigen::Matrix<T, 3, 3> &R) {
    return {R(2,1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1)};
}

template <typename T>
Eigen::Matrix<T, 3, 9> dvee() {
    return (Eigen::Matrix<T, 3, 9>{} 
            << 0, 0, 0, 0, 0, -1, 0, 1, 0,
               0, 0, 1, 0, 0, 0, -1, 0, 0,
               0, -1, 0, 1, 0, 0, 0, 0, 0).finished();
}

template <typename T>
Eigen::Matrix<T, 3, 9> dvee(const Eigen::Matrix<T, 3, 3> &R) {
    return dvee<T>();
}

template <typename T, int N, int M>
Eigen::Matrix<T, M*N, N*M> dAt_dA() {
    Eigen::Matrix<T, M*N, N*M> D;
    D.setZero();
    for (int m = 0; m < M; ++m) {
        for (int n = 0; n < N; ++n) {
            D(m*N+n, n*M+m) = 1;
        }
    }
    return D;
}

template <typename T, int N, int M>
Eigen::Matrix<T, M*N, N*M> dAt_dA(const Eigen::Matrix<T, N, M> &A) {
    return dAt_dA<T, N, M>();
}



// Note, by default the Eigen matrices arrange data in RowMajor order.
// This does not affect the way we index the element via () operator.
// But when using Map<> function to map raw internal data to matrices/vectors,
// we need to be careful about the order.
// dC_{n,p}/dA_{n,m}=B_{m,p}
template <typename T, int N, int M, int P>
Eigen::Matrix<T, N*P, N*M> dAB_dA(
        const Eigen::Matrix<T, N, M> &A,
        const Eigen::Matrix<T, M, P> &B) {
    Eigen::Matrix<T, N*P, N*M> D;
    D.setZero();
    for (int n = 0; n < N; ++n) {
        for (int p = 0; p < P; ++p) {
            for (int m = 0; m < M; ++m) {
                D(n*P+p, n*M+m) += B(m, p);
            }
        }
    }
    return D;
}

// dC_{n,p}/dB_{m,p}=A_{n,m}
template <typename T, int N, int M, int P>
Eigen::Matrix<T, N*P, M*P> dAB_dB(
        const Eigen::Matrix<T, N, M> &A,
        const Eigen::Matrix<T, M, P> &B) {
    Eigen::Matrix<T, N*P, M*P> D;
    D.setZero();
    for (int n = 0; n < N; ++n) {
        for (int p = 0; p < P; ++p) {
            for (int m = 0; m < M; ++m) {
                D(n*P+p, m*P+p) += A(n, m);
            }
        }
    }
    return D;
}

template <typename T>
Eigen::Matrix<T, 3, 3> rodrigues(const Eigen::Matrix<T, 3, 1> &w, 
        Eigen::Matrix<T, 9, 3> *dR_dw=nullptr) {
    Eigen::Matrix<T, 3, 3> R;

    T th = w.norm();

    if (th < 1e-8) {
        // R = I + hat(w)
        // std::cout << "small angle approximation" << std::endl;
        R = Eigen::Matrix<T, 3, 3>::Identity() + hat(w);
        if (dR_dw) {
            *dR_dw = dhat(w);
        }
        return R;
    }
    T inv_th = 1.0 / th;
    Eigen::Matrix<T, 3, 1> u = w * inv_th;

    // R = I + u.hat * sin(th) + (u.hat)^2 * (1-cos(th))
    T sin_th = sin(th);
    T cos_th = cos(th);
    Eigen::Matrix<T, 3, 3> uhat = hat(u);
    Eigen::Matrix<T, 3, 3> uhat2 = uhat * uhat;
    R = Eigen::Matrix<T, 3, 3>::Identity() + uhat * sin_th + uhat2 * (1-cos_th);
    if (dR_dw) {
        Eigen::Matrix<T, 9, 3> dR_du = sin_th * dhat(u) 
            + (1-cos_th) * (dAB_dA(uhat, uhat) + dAB_dB(uhat, uhat))* dhat(u);
        Eigen::Matrix<T, 3, 3> du_dw = inv_th * (Eigen::Matrix<T, 3, 3>::Identity() - u * u.transpose());
        Eigen::Matrix<T, 9, 1> dR_dth = Eigen::Map<Eigen::Matrix<T, 9, 1>>(
                Eigen::Matrix<T, 3, 3>{uhat * cos_th + uhat2 * sin_th}.data());
        // Eigen::Matrix<T, 1, 3> dth_dw = u.transpose();
        *dR_dw = dR_du * du_dw + dR_dth * u.transpose();
    }
    return R;
}

template <typename T>
Eigen::Matrix<T, 3, 1> invrodrigues(const Eigen::Matrix<T, 3, 3> &R,
        Eigen::Matrix<T, 3, 9> *dw_dR=nullptr) {

    Eigen::Matrix<T, 3, 1> w;

    T tmp = 0.5*(R.trace() - 1);
    Eigen::Matrix<T, 3, 1> vee_R = vee(R);
    if (tmp > 1.0 - 1e-10) {
        // std::cout << "small angle approximation" << std::endl;
        w = 0.5 * vee_R;
        if (dw_dR) {
            *dw_dR = 0.5 * dvee(R);
        }
        return w;
    }

    T th = acos(tmp);
    T sin_th = sin(th);
    T inv_sin_th = 1.0 / sin_th;
    Eigen::Matrix<T, 3, 1> u = 0.5 * vee_R * inv_sin_th;

    w = th * u;

    if (dw_dR) {
        Eigen::Matrix<T, 1, 9> dth_dR;
        T dth_dtmp = -1/sqrt(1-tmp*tmp);
        Eigen::Matrix<T, 1, 9> dtmp_dR;
        dtmp_dR << 1, 0, 0, 0, 1, 0, 0, 0, 1;    // d(trace(R)-1)_dR
        dtmp_dR *= 0.5;
        dth_dR = dth_dtmp * dtmp_dR;

        Eigen::Matrix<T, 3, 9> du_dR;
        // u = vee(R) / (2*sin(th));
        du_dR = 0.5*(dvee(R) * inv_sin_th - vee(R) * cos(th) * inv_sin_th * inv_sin_th * dth_dR);
        *dw_dR = u * dth_dR  + th * du_dR;
    }
    return w;
}


template <typename T>
Eigen::Matrix<T, 3, 3> projectSO3(const Eigen::Matrix<T, 3, 3> &R) {
    Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * Eigen::Matrix<T, 3, 3>::Identity() * svd.matrixV().transpose();
}

