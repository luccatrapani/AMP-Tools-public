#include "MyPRM2D.h"

amp::MyPRM2D::MyPRM2D(const Eigen::Vector2d& lower_bounds, const Eigen::Vector2d& upper_bounds) :
    m_upper_bounds(Eigen::VectorXd::Map(upper_bounds.data(), upper_bounds.size())), 
    m_lower_bounds(Eigen::VectorXd::Map(lower_bounds.data(), lower_bounds.size())) {}

amp::Path2D amp::MyPRM2D::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    // Build CSpace

    std::unique_ptr<amp::ConfigurationSpace> cspace_ptr = std::make_unique<amp::pointAgentCSpace>(problem, m_lower_bounds, m_upper_bounds);

    //cspace_ptr->m_problem.print();


    // Call generic planner on problem with cspace
    amp::genericPRM PRMalgo;
    amp::Path pathND = PRMalgo.planND(problem.q_init, problem.q_goal, cspace_ptr);

    
    // Turn this path into 2D path
    int pathSize = pathND.waypoints.size();

    for (int i = 0; i<pathSize; i++) {
        Eigen::VectorXd state = pathND.waypoints[i];
        Eigen::Vector2d state2d = {state[0], state[1]};
        path.waypoints.push_back(state2d);
    }

    return path;
}