#include "AMPCore.h"
#include "MyAStarAlgo.h"


namespace amp{
    class genericPRM {
        public:
            Path planND(const Eigen::VectorXd& init_state, 
                       const Eigen::VectorXd& goal_state, 
                       const std::unique_ptr<ConfigurationSpace>& cspace);
            
    };
}