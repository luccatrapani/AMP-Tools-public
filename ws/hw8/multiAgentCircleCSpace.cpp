#include "multiAgentCircleCSpace.h"

amp::multiAgentCircleCSpace::multiAgentCircleCSpace(const amp::MultiAgentProblem2D& problem, const Eigen::VectorXd& lower_bounds, const Eigen::VectorXd& upper_bounds, int numAgents) : 
    amp::ConfigurationSpace::ConfigurationSpace(lower_bounds, upper_bounds), m_problem(problem), m_numAgents(numAgents){}


bool amp::multiAgentCircleCSpace::inCollision(const Eigen::VectorXd& cspace_state) const {
    // This state vector is going to come in as 1 big long state
    // Break up into a vector
    bool notValid = false;
    double buffer = 0;

    std::vector<Eigen::Vector2d> agentStates;
    for (int i = 0; i<cspace_state.size(); i+=2){
        Eigen::Vector2d currentAgent = {cspace_state[i], cspace_state[i+1]};
        agentStates.push_back(currentAgent);
    }

    // Loop through each agent: Check for collisions with other agents, then check for collisions with obstacles
    for (int agent = 0; agent<m_numAgents; agent++) {

        int numObstacles = m_problem.obstacles.size(); //Get number of obstacles
        Eigen::Vector2d q_i = agentStates[agent];
        double R_i = m_problem.agent_properties[agent].radius;

        // Check for collisions with other agents
        for (int checkAgent = agent+1; checkAgent<m_numAgents; checkAgent++){
            double R_check = m_problem.agent_properties[checkAgent].radius;
            Eigen::Vector2d vecCheck = q_i - agentStates[checkAgent];

            if (vecCheck.norm() < (R_i + R_check + buffer/2)){
                notValid = true;
                return notValid;
            }

        }

        // Now check for collisions with obstacles
        for (int k=0; k<numObstacles; k++){
            //For each polygon, get CCW ordered list of vertices
            std::vector<Eigen::Vector2d> vertices = m_problem.obstacles[k].verticesCCW();
            //Find number of vertices
            int numVertices = vertices.size();

            // Loop through the vertices and perform collision checking 
            //j vertex is always previous vertex, i vertex is always current vertex
            for(int i=0, j = numVertices-1; i<numVertices; j = i++){
                Eigen::Vector2d vertex_i = vertices[i]; Eigen::Vector2d vertex_j = vertices[j];
                double x_i = vertex_i[0]; double x_j = vertex_j[0]; double y_i = vertex_i[1]; double y_j = vertex_j[1];
                double x_c = q_i[0]; double y_c = q_i[1];

                //std::cout << x_i << " " << y_i << "  :  " << x_j << " " << y_j << std::endl;


                if (x_i == x_j) {
                    double max_Y = std::max(y_i, y_j);
                    double min_Y = std::min(y_i, y_j);
                    double closestX = x_i;
                    double closestY = std::max(std::min(y_c, max_Y), min_Y);

                    double distanceSq = (x_c - closestX) * (x_c - closestX) + (y_c - closestY) * (y_c - closestY);

                    if (distanceSq < R_i*R_i  + buffer) {
                        notValid = true;
                        return notValid;
                    }
                } else if(y_i == y_j) {
                    //std::cout << "Entered the horz" << std::endl;
                    double max_X = std::max(x_i, x_j);
                    double min_X = std::min(x_i, x_j);
                    double closestX = std::max(std::min(x_c, max_X), min_X);
                    double closestY = y_i;

                    double distanceSq = (x_c - closestX) * (x_c - closestX) + (y_c - closestY) * (y_c - closestY);

                    if (distanceSq < R_i*R_i + buffer) {
                        notValid = true;
                        return notValid;
                    }

                } else {

                    // Calculate the squared distance from the circle center to the line segment
                    double segmentLengthSq = (x_j - x_i) * (x_j - x_i) + (y_j - y_i) * (y_j - y_i);
                    
                    double dotProduct = ((x_c - x_i) * (x_j - x_i) + (y_c - y_i) * (y_j - y_i)) / segmentLengthSq;
        
                    // Find the closest point on the line segment to the circle center
                    double closestX = x_i + dotProduct * (x_j - x_i);
                    double closestY = y_i + dotProduct * (y_j - y_i);
        
                    // Calculate the squared distance from the closest point to the circle center
                    double distanceSq = (x_c - closestX) * (x_c - closestX) + (y_c - closestY) * (y_c - closestY);
        
                    // Check if this distance is less than radius of the circle
                    if (distanceSq < R_i*R_i + buffer) {
                        //std::cout << "Distance sq: " << distanceSq << std::endl;
                        //std::cout << "R sq: " << R_i*R_i << std::endl;
                        notValid = true;
                        return notValid;
                    }

                }

                
            }

        }
    }

    return notValid;
}




 