
#include "gridStructure.h"

//STL
#include <algorithm>
#include <stdexcept>

#include "lineIterator.h"

namespace StVO {

// 由Bresenham的线条绘制算法，由线段的两个端点得到线段上所有的点
void getLineCoords(double x1, double y1, double x2, double y2, std::list<std::pair<int, int>> &line_coords) {
    line_coords.clear();

    LineIterator it(x1, y1, x2, y2);

    std::pair<int, int> p;
    while (it.getNext(p))
        line_coords.push_back(p);
}

GridStructure::GridStructure(int rows, int cols)
    : rows(rows), cols(cols) {

    if (rows <= 0 || cols <= 0)
        throw std::runtime_error("[GridStructure] invalid dimension");

    grid.resize(cols, std::vector<std::list<int>>(rows));
}

GridStructure::~GridStructure() {

}

std::list<int>& GridStructure::at(int x, int y) {

    if (x >= 0 && x < cols &&
            y >= 0 && y < rows)
        return grid[x][y];
    else
        return out_of_bounds;
}

/**
 * @brief 将搜索窗口内的所有特征点取出，作为候选匹配特征点
 * 
 * @param x 特征点网格x坐标
 * @param y 特征点网格y坐标
 * @param w 窗口大小
 * @param indices 候选匹配特征点
 */
void GridStructure::get(int x, int y, const GridWindow &w, std::unordered_set<int> &indices) const {

    int min_x = std::max(0, x - w.width.first);
    int max_x = std::min(cols, x + w.width.second + 1);

    int min_y = std::max(0, y - w.height.first);
    int max_y = std::min(rows, y + w.height.second + 1);

    for (int x_ = min_x; x_ < max_x; ++x_)
        for (int y_ = min_y; y_ < max_y; ++y_)
            indices.insert(grid[x_][y_].begin(), grid[x_][y_].end());
}

void GridStructure::clear() {

    for (int x = 0; x < cols; ++x)
        for (int y = 0; y < rows; ++y)
            grid[x][y].clear();
}

} //namesapce StVO
