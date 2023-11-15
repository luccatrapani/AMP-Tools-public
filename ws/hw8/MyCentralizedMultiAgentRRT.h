#include "hw/HW8.h"
#include "AMPCore.h"
#include "genericGoalBiasRRT.h"
#include "multiAgentCircleCSpace.h"

namespace amp{
    class MyCentralizedMultiAgentRRT : public CentralizedMultiAgentRRT {
        public:
            MyCentralizedMultiAgentRRT(const Eigen::VectorXd& lower_bounds, const Eigen::VectorXd& upper_bounds, int n, double r, double p, double e);

            virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

            int m_RRTTree = 0;
            double m_time = 0;

        private:
            int m_n;
            double m_r;
            double m_p;
            double m_e;
            Eigen::VectorXd m_lower_bounds;
            Eigen::VectorXd m_upper_bounds;
    };
}