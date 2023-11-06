#include "hw/HW7.h"
#include "CollisionChecker.h"
#include "pointAgentCSpace.h"
#include "genericGoalBiasRRT.h"

namespace amp {
    class MyGoalBiasRRT2D : public GoalBiasRRT2D {
        public:
            MyGoalBiasRRT2D(const Eigen::Vector2d& lower_bounds, const Eigen::Vector2d& upper_bounds, 
                int n, double r, double p, double e);

            virtual amp::Path2D plan(const amp::Problem2D& problem) override;

            const Eigen::VectorXd m_lower_bounds;
            const Eigen::VectorXd m_upper_bounds;

        private: 
            int m_n; 
            double m_r; 
            double m_p; 
            double m_e;
    };
}