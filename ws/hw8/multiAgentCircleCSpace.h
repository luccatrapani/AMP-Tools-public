#include "AMPCore.h"

namespace amp{
    class multiAgentCircleCSpace : public ConfigurationSpace {
        public:
            multiAgentCircleCSpace(const amp::MultiAgentProblem2D& problem, const Eigen::VectorXd& lower_bounds, const Eigen::VectorXd& upper_bounds, int numAgents);

            virtual bool inCollision(const Eigen::VectorXd& cspace_state) const override;

            const amp::MultiAgentProblem2D& m_problem;
        private:
            int m_numAgents;

    };
}