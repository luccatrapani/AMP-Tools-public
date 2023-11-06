#include "AMPCore.h"
#include "CollisionChecker.h"

namespace amp{
    class pointAgentCSpace : public ConfigurationSpace {
        public:
            pointAgentCSpace(const Problem2D& problem, const Eigen::VectorXd& lower_bounds, const Eigen::VectorXd& upper_bounds);

            virtual bool inCollision(const Eigen::VectorXd& cspace_state) const override;

            const Problem2D& getProblem() const {return m_problem;};

            const Problem2D& m_problem;
    };
}