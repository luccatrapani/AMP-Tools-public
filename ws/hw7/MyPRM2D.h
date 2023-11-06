#include "AMPCore.h"
#include "pointAgentCSpace.h"
#include "hw/HW7.h"
#include "genericPRM.h"

namespace amp {
    class MyPRM2D : public PRM2D, public genericPRM {
        public:
            MyPRM2D(const Eigen::Vector2d& lower_bounds, const Eigen::Vector2d& upper_bounds);

            virtual Path2D plan(const amp::Problem2D& problem) override;

            const Eigen::VectorXd m_lower_bounds;
            const Eigen::VectorXd m_upper_bounds;
    };
}