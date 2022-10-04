
#pragma once

//STL
#include <utility>
#include <vector>

//OpenCV
#include <opencv2/core.hpp>

#include "gridStructure.h"

#include <iostream>

namespace StVO {

typedef std::pair<int, int> point_2d;
typedef std::pair<point_2d, point_2d> line_2d;

inline double dot(const std::pair<double, double> &a, const std::pair<double, double> &b) {
    return (a.first*b.first + a.second*b.second);
}

inline void normalize(std::pair<double, double> &v) {
    double magnitude = std::sqrt(dot(v, v));

    v.first /= magnitude;
    v.second /= magnitude;
}

int matchNNR(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12);

int match(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12);

int distance(const cv::Mat &a, const cv::Mat &b);

//Points
int matchGrid(const std::vector<point_2d> &points1, const cv::Mat &desc1, const GridStructure &grid, const cv::Mat &desc2, const GridWindow &w, std::vector<int> &matches_12);

//Lines
int matchGrid(const std::vector<line_2d> &lines1, const cv::Mat &desc1, const GridStructure &grid, const cv::Mat &desc2, const std::vector<std::pair<double, double>> &directions2, const GridWindow &w, std::vector<int> &matches_12);

} // namesapce StVO
