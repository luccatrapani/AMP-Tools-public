#include "AMPCore.h"

namespace amp{
class MyGridCSpace : public amp::GridCSpace2D {
    public:
        //MyGridCSpace() : amp::GridCSpace2D(1, 1, 0.0, 1.0, 0.0, 1.0){}
        MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max);

        virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;
};
}