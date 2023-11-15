#include "AMPCore.h"
#include "MyAStarAlgo.h"

namespace amp{
    class genericGoalBiasRRT {
        public:
        genericGoalBiasRRT(int n, double r, double p, double e);

        amp::Path planND(const Eigen::VectorXd& init_state, 
            const Eigen::VectorXd& goal_state, const std::unique_ptr<amp::ConfigurationSpace>& cspace);

        int m_treeSize = 0;

        private: 
        int m_n; 
        double m_r; 
        double m_p; 
        double m_e;
    };
}