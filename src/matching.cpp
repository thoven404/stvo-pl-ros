
#include "matching.h"

//STL
#include <cmath>
#include <functional>
#include <future>
#include <limits>
#include <stdexcept>

//OpenCV
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

#include "config.h"
#include "gridStructure.h"

namespace StVO {

// 描述子knn匹配，保留最佳描述子对应的索引。该最描述子的距离 < 次佳描述子的距离 * nnr
int matchNNR(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12) {

    int matches = 0;
    matches_12.resize(desc1.rows, -1);

    std::vector<std::vector<cv::DMatch>> matches_;
    cv::Ptr<cv::BFMatcher> bfm = cv::BFMatcher::create(cv::NORM_HAMMING, false); // cross-check
    bfm->knnMatch(desc1, desc2, matches_, 2);

    if (desc1.rows != matches_.size())
        throw std::runtime_error("[matchNNR] Different size for matches and descriptors!");

    for (int idx = 0; idx < desc1.rows; ++idx) {
        if (matches_[idx][0].distance < matches_[idx][1].distance * nnr) {
            matches_12[idx] = matches_[idx][0].trainIdx;
            matches++;
        }
    }
    return matches;
}


// 描述子匹配
int match(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12) {

    if (Config::bestLRMatches()) {
        int matches;
        std::vector<int> matches_21;
        if (Config::lrInParallel()) {
            auto match_12 = std::async(std::launch::async, &matchNNR,
                                  std::cref(desc1), std::cref(desc2), nnr, std::ref(matches_12));
            auto match_21 = std::async(std::launch::async, &matchNNR,
                                  std::cref(desc2), std::cref(desc1), nnr, std::ref(matches_21));
            matches = match_12.get();   // 得到匹配数量

            match_21.wait();
        } else {
            matches = matchNNR(desc1, desc2, nnr, matches_12);
            matchNNR(desc2, desc1, nnr, matches_21);
        }

        for (int i1 = 0; i1 < matches_12.size(); ++i1) {    // 保留双向最佳匹配
            int &i2 = matches_12[i1];
            if (i2 >= 0 && matches_21[i2] != i1) {
                i2 = -1;
                matches--;
            }
        }

        return matches;
    } else{
        return matchNNR(desc1, desc2, nnr, matches_12);
    }
        
}


// 计算描述子的距离
int distance(const cv::Mat &a, const cv::Mat &b) {

    // adapted from: http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel

    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist = 0;
    for(int i = 0; i < 8; i++, pa++, pb++) {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

/**
 * @brief 网格匹配特征点，得到双向最佳匹配
 * 
 * @param points1   左目特征点在栅格中的坐标
 * @param desc1     左目特征点的描述子
 * @param grid      网格容器
 * @param desc2     右目特征点描述子
 * @param w         搜索窗口
 * @param matches_12    输出的特征匹配关系
 * @return int 匹配上的特征点数量
 */
int matchGrid(const std::vector<point_2d> &points1, const cv::Mat &desc1, const GridStructure &grid, const cv::Mat &desc2, const GridWindow &w, std::vector<int> &matches_12) {

    if (points1.size() != desc1.rows)
        throw std::runtime_error("[matchGrid] Each point needs a corresponding descriptor!");

    int matches = 0;
    matches_12.resize(desc1.rows, -1);  // 初始化匹配为 -1

    int best_d, best_d2, best_idx;      
    std::vector<int> matches_21, distances;

    if (Config::bestLRMatches()) {
        matches_21.resize(desc2.rows, -1);
        distances.resize(desc2.rows, std::numeric_limits<int>::max());
    }

    for (int i1 = 0; i1 < points1.size(); ++i1) {

        best_d = std::numeric_limits<int>::max();
        best_d2 = std::numeric_limits<int>::max();
        best_idx = -1;

        const std::pair<int, int> &coords = points1[i1];    // 特征点的网格坐标
        cv::Mat desc = desc1.row(i1);       // 该特征点对应的描述子

        std::unordered_set<int> candidates;
        grid.get(coords.first, coords.second, w, candidates);   // 将窗口内的所有右目特征点在右目特征点容器中的索引取出，得到 candidates

        if (candidates.empty()) continue;
        for (const int &i2 : candidates) {
            if (i2 < 0 || i2 >= desc2.rows) continue;

            const int d = distance(desc, desc2.row(i2));    // 计算左右目描述子的距离

            if (Config::bestLRMatches()) {
                if (d < distances[i2]) {
                    distances[i2] = d;
                    matches_21[i2] = i1;
                } else continue;
            }

            if (d < best_d) {
                best_d2 = best_d;
                best_d = d;
                best_idx = i2;
            } else if (d < best_d2)
                best_d2 = d;
        }

        if (best_d < best_d2 * Config::minRatio12P()) {     // 如果最小的距离大于此小的距离一定倍数，就认为匹配成功
            matches_12[i1] = best_idx;
            matches++;
        }
    }

    if (Config::bestLRMatches()) {      // 如果目标特征点有多个对应关系，删除该匹配
        for (int i1 = 0; i1 < matches_12.size(); ++i1) {
            int &i2 = matches_12[i1];
            if (i2 >= 0 && matches_21[i2] != i1) {
                i2 = -1;
                matches--;
            }
        }
    }

    return matches;
}

/**
 * @brief 网格匹配线段，得到双向最佳匹配
 * 
 * @param lines1    左目提取到的线段两个端点所在的网格坐标
 * @param desc1     左目线段的描述子
 * @param grid      图像网格，每个网格中保存经过该网格的右目中线段的索引
 * @param desc2     右目线段的描述子
 * @param directions2 右目线段的方向单位向量
 * @param w         搜索窗口
 * @param matches_12 
 * @return int 匹配上的线段的数量
 */
int matchGrid(const std::vector<line_2d> &lines1, const cv::Mat &desc1,
              const GridStructure &grid, const cv::Mat &desc2, const std::vector<std::pair<double, double>> &directions2,
              const GridWindow &w,
              std::vector<int> &matches_12) {

    if (lines1.size() != desc1.rows)
        throw std::runtime_error("[matchGrid] Each line needs a corresponding descriptor!");

    int matches = 0;
    matches_12.resize(desc1.rows, -1);

    int best_d, best_d2, best_idx;
    std::vector<int> matches_21, distances;

    if (Config::bestLRMatches()) {
        matches_21.resize(desc2.rows, -1);
        distances.resize(desc2.rows, std::numeric_limits<int>::max());
    }

    for (int i1 = 0; i1 < lines1.size(); ++i1) {

        best_d = std::numeric_limits<int>::max();
        best_d2 = std::numeric_limits<int>::max();
        best_idx = -1;

        const line_2d &coords = lines1[i1];     // 左目图像提取到线段的端点
        cv::Mat desc = desc1.row(i1);           // 该线段的描述子

        const point_2d sp = coords.first;       // 端点1
        const point_2d ep = coords.second;      // 端点2

        std::pair<double, double> v = std::make_pair(ep.first - sp.first, ep.second - sp.second);   // [delta x, delta y] 单位向量
        normalize(v);

        std::unordered_set<int> candidates;     // 保存w范围内所有线段索引为candidates
        grid.get(sp.first, sp.second, w, candidates);
        grid.get(ep.first, ep.second, w, candidates);

        if (candidates.empty()) continue;
        for (const int &i2 : candidates) {      // 匹配准则：角度偏差小，最佳描述子距离小于次佳描述子距离乘以一定倍数
            if (i2 < 0 || i2 >= desc2.rows) continue;

            if (std::abs(dot(v, directions2[i2])) < Config::lineSimTh())    // 计算左右目线段的点积，如果太小，说明角度偏差大，为错误的匹配
                continue;

            const int d = distance(desc, desc2.row(i2));

            if (Config::bestLRMatches()) {
                if (d < distances[i2]) {
                    distances[i2] = d;
                    matches_21[i2] = i1;
                } else continue;
            }

            if (d < best_d) {
                best_d2 = best_d;
                best_d = d;
                best_idx = i2;
            } else if (d < best_d2)
                best_d2 = d;
        }

        if (best_d < best_d2 * Config::minRatio12P()) {
            matches_12[i1] = best_idx;
            matches++;
        }
    }

    if (Config::bestLRMatches()) {
        for (int i1 = 0; i1 < matches_12.size(); ++i1) {
            int &i2 = matches_12[i1];
            if (i2 >= 0 && matches_21[i2] != i1) {
                i2 = -1;
                matches--;
            }
        }
    }

    return matches;
}

} //namesapce StVO
