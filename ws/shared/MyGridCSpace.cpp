#include "MyGridCSpace.h"

amp::MyGridCSpace::MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
            : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max) {}

std::pair<std::size_t, std::size_t> amp::MyGridCSpace::getCellFromPoint(double x0, double x1) const{
    std::pair<double, double> x0MinMax = x0Bounds();
    std::pair<double, double> x1MinMax = x1Bounds();
    std::pair<std::size_t, std::size_t> cSpaceSize = size();

    double x0Steps = (x0MinMax.second - x0MinMax.first)/cSpaceSize.first;
    double x1Steps = (x1MinMax.second - x1MinMax.first)/cSpaceSize.second;

    int i = std::floor((x0+std::abs(x0MinMax.first))/x0Steps);
    int j = std::floor((x1+std::abs(x1MinMax.first))/x1Steps);

    std::pair<int, int> indices = {i, j};

    return indices;
}