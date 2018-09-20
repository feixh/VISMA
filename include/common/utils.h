//
// Created by feixh on 10/25/17.
//
// Common utilities.
#pragma once
#include "eigen_alias.h"

// stl
#include <chrono>
#include <unordered_map>
#include <map>
#include <memory>
#include <string>

namespace feh {

/// \brief colorful texts for terminal
struct TermColor {
    static const std::string red;
    static const std::string green;
    static const std::string blue;
    static const std::string cyan;
    static const std::string yellow;
    static const std::string magenta;
    static const std::string gray;
    static const std::string white;
    static const std::string bold;
    static const std::string end;
    static const std::string endl;
private:
    TermColor() = delete;
};


/// \brief timer
class Timer {
public:

    enum Unit {
        MILLISEC,
        MICROSEC,
        NANOSEC
    };

public:
    friend std::ostream &operator<<(std::ostream &os, const Timer &obj);

    Timer() :
        module_name_("default"),
        report_average(true) {}

    Timer(std::string module_name_) :
        module_name_(module_name_),
        report_average(true) {}

    void Tick() {
        Tick("anonymous event");
    }

    void Tick(const std::string &event) {
        start_[event] = std::chrono::high_resolution_clock::now();
    }

    float Tock() {
        return Tock("anonymous event");
    }

    /// \param: Return timing in milliseconds. Also record timing in look up table.
    float Tock(const std::string &event) {
        float timing(Elapsed(event).count());
        if (look_up_table_.count(event)) {
            look_up_table_[event] += timing;
            counter_[event] += 1;
        } else {
            look_up_table_[event] = timing;
            counter_[event] = 1;
        }
        return timing * 1e-6;
    }

    void Reset() {
        start_.clear();
        look_up_table_.clear();
        counter_.clear();
    }

    float LookUp(const std::string &event, const Unit unit = MILLISEC, bool average = false) const {
        if (!look_up_table_.count(event)) return -1;
        switch (unit) {
            case MILLISEC:
                return look_up_table_.at(event) * 1e-6 / (average ? counter_.at(event) + 0.01 : 1.0);
                break;
            case MICROSEC:
                return look_up_table_.at(event) * 1e-3 / (average ? counter_.at(event) + 0.01 : 1.0);
                break;
            case NANOSEC:
                return look_up_table_.at(event) / (average ? counter_.at(event) + 0.01 : 1.0);
                break;
            default:
                return -1;
                break;
        }
    }

    std::chrono::nanoseconds Elapsed(std::string event) const {
        auto tmp = std::chrono::high_resolution_clock::now();
        assert(start_.count(event) && ("event[" + event + "] not found").c_str());
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            tmp - start_.at(event));
    }

private:
    std::unordered_map<std::string, std::chrono::high_resolution_clock::time_point> start_;
    std::map<std::string, float> look_up_table_;
    std::unordered_map<std::string, int> counter_;
    std::string module_name_;

public:
    bool report_average;
};

/// \brief generate a random matrix of dimension N x M
template<int N, int M = N>
Eigen::Matrix<FloatType, N, M> RandomMatrix(
    FloatType meanVal = 0.0,
    FloatType stdVal = 1.0,
    std::shared_ptr<std::knuth_b> p_engine = nullptr) {

    using FloatType = FloatType;

    std::normal_distribution<FloatType> dist(meanVal, stdVal);
    Eigen::Matrix<FloatType, N, M> v;
    if (p_engine == nullptr) {
        std::default_random_engine engine;
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < M; ++j) {
                v(i, j) = dist(engine);
            }
    } else {
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < M; ++j) {
                v(i, j) = dist(*p_engine);
            }
    }
    return v;
};


/// \brief generate a random vecotr of dimension N
template<int N>
Eigen::Matrix<FloatType, N, 1> RandomVector(
    FloatType meanVal = 0.0,
    FloatType stdVal = 1.0,
    std::shared_ptr<std::knuth_b> p_engine = nullptr) {

    using FloatType = FloatType;

    std::normal_distribution<FloatType> dist(meanVal, stdVal);
    Eigen::Matrix<FloatType, N, 1> v;
    if (p_engine == nullptr) {
        std::default_random_engine engine;
        for (int i = 0; i < N; ++i) {
            v(i) = dist(engine);
        }
    } else {
        for (int i = 0; i < N; ++i) {
            v(i) = dist(*p_engine);
        }

    }
    return v;
};

/// \brief detect if any compoennt of a matrix is nan
template<int N, int M = 1>
bool anynan(const Eigen::Matrix<FloatType, N, M> &m) {
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < M; ++j)
            if (std::isnan(m(i, j)))
                return true;
    return false;
};

/// \brief print enum value as int
template <typename Enumeration>
typename std::underlying_type<Enumeration>::type as_integer(Enumeration const value) {
    using type = typename std::underlying_type<Enumeration>::type;
    return static_cast<type>(value);
}



inline constexpr int cube(int x) { return x * x * x; }
inline constexpr int square(int x) { return x * x; }

template <int L=8>
std::vector<std::array<uint8_t, 3>> GenerateRandomColorMap() {
    uint8_t q = 255 / L;
    std::vector<std::array<uint8_t,3>> cm(cube(8));
    auto generator = std::knuth_b(0);
    for (int i = 0; i < cube(L); ++i) {
        cm[i] = {q * (i%8u), q * ((i/8)%8u), q * ((i/64)%8u)};
        std::shuffle(cm[i].begin(), cm[i].end(), generator);
    }
    std::shuffle(cm.begin()+1, cm.end(), generator);
    return cm;
};

/// \brief: List files with the given extension.
/// \param path: Directory of files.
/// \param extension: File extension.
/// \param filenames: List of returned file names.
/// \return: True if reads the file list successfully.
bool Glob(const std::string &path, const std::string &extension, std::vector<std::string> &filenames);

bool Glob(const std::string &path,
          const std::string &extension,
          const std::string &prefix,
          std::vector<std::string> &filenames);

}
