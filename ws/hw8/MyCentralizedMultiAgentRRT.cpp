#include "MyCentralizedMultiAgentRRT.h"

amp::MyCentralizedMultiAgentRRT::MyCentralizedMultiAgentRRT(const Eigen::VectorXd& lower_bounds, const Eigen::VectorXd& upper_bounds, int n, double r, double p, double e) :
    m_upper_bounds(upper_bounds), 
    m_lower_bounds(lower_bounds),
    m_n(n), m_r(r), m_p(p), m_e(e){}

amp::MultiAgentPath2D amp::MyCentralizedMultiAgentRRT::plan(const amp::MultiAgentProblem2D& problem) {
    //Set up timer
    amp::Timer multTime("mult");
    
    // Make Path
    amp::MultiAgentPath2D path;
    int numAgents = problem.numAgents();

    std::unique_ptr<amp::ConfigurationSpace> cspace_ptr = 
        std::make_unique<amp::multiAgentCircleCSpace>(problem, m_lower_bounds, m_upper_bounds, numAgents);


    // Instantiate goal bias RRT algo
    amp::genericGoalBiasRRT algo(m_n, m_r, m_p, m_e);

    // Create init state
    Eigen::VectorXd init_state(numAgents*2);
    Eigen::VectorXd goal_state(numAgents*2);
    int i = 0;
    for (int agent = 0; agent<numAgents; agent++){
        init_state(i) = problem.agent_properties[agent].q_init[0];
        init_state(i+1) = problem.agent_properties[agent].q_init[1];

        goal_state(i) = problem.agent_properties[agent].q_goal[0];
        goal_state(i+1) = problem.agent_properties[agent].q_goal[1];

        i+=2;
    }

    
    // Plan
    amp::Path pathND = algo.planND(init_state, goal_state, cspace_ptr);

    if (!pathND.valid) {
        std::cout << "RRT Could not give a result of Centralized Multi agent problem" << std::endl;
        path.valid = false;
        return path;
    }
    // Get the treesize
    m_RRTTree = algo.m_treeSize;

    
    // Now that I have a path, put it into type multiAgent2D path

    i = 0;
    for (int agent = 0; agent<numAgents; agent++) {
        amp::Path2D path_i;
        Eigen::VectorXd marker_k;
        Eigen::Vector2d marker2d;
        for (int k = 0; k<pathND.waypoints.size(); k++) {
            marker_k = pathND.waypoints[k];
            marker2d = {marker_k[i], marker_k[i+1]};
            path_i.waypoints.push_back(marker2d);
        }
        path.agent_paths.push_back(path_i);
        i+=2;
    }


    // Check the timer
    m_time = multTime.now();
    path.valid = true;
    return path;

    
}