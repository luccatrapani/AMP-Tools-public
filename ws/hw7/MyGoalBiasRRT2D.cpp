#include "MyGoalBiasRRT2D.h"

amp::MyGoalBiasRRT2D::MyGoalBiasRRT2D(const Eigen::Vector2d& lower_bounds, const Eigen::Vector2d& upper_bounds,
    int n, double r, double p, double e) :
    m_upper_bounds(Eigen::VectorXd::Map(upper_bounds.data(), upper_bounds.size())), 
    m_lower_bounds(Eigen::VectorXd::Map(lower_bounds.data(), lower_bounds.size())),
    m_n(n), m_r(r), m_p(p), m_e(e) {}


amp::Path2D amp::MyGoalBiasRRT2D::plan(const amp::Problem2D& problem) {
    // construct cspace
    amp::Path2D path;
    // Build CSpace

    std::unique_ptr<amp::ConfigurationSpace> cspace_ptr = 
        std::make_unique<amp::pointAgentCSpace>(problem, m_lower_bounds, m_upper_bounds);



    amp::genericGoalBiasRRT algo(m_n, m_r, m_p, m_e);
    amp::Path pathND = algo.planND(problem.q_init, problem.q_goal, cspace_ptr);
    // Turn this path into 2D path
    int pathSize = pathND.waypoints.size();

    for (int i = 0; i<pathSize; i++) {
        Eigen::VectorXd state = pathND.waypoints[i];
        Eigen::Vector2d state2d = {state[0], state[1]};
        path.waypoints.push_back(state2d);
    }

    

    return path;
}