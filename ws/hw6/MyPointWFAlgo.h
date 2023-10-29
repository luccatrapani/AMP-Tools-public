#include "hw/HW6.h"

#include "AMPCore.h"
#include "MyCSpaceCtor.h"
#include "CollisionChecker.h"

namespace amp{

class MyPointWFAlgo : public amp::PointWaveFrontAlgorithm {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override;

        // This is just to get grade to work, you DO NOT need to override this method
        //virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;
};

}