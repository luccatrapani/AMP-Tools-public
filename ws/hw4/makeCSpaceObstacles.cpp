#include "makeCSpaceObstacles.h"

namespace amp{
makeCSpaceObstacles::CSpaceObstacle makeCSpaceObstacles::makeObstacles(const Obstacle2D& obstacle, const Obstacle2D& agent, const int numConfigurations){
    // Initialize struct
    CSpaceObstacle cspaceobstacles;

    
    // Get obstacle and agent
    std::vector<Eigen::Vector2d> obstacleVerts = obstacle.verticesCCW();
    std::vector<Eigen::Vector2d> agentVerts = agent.verticesCCW();
    obstacleVerts.push_back(obstacleVerts[0]);

    int numObsVerts = obstacleVerts.size();
    int numAgentVerts = agentVerts.size();
    
    double rotationStep = 2*std::numbers::pi/numConfigurations;
    double rotate = 0;

    for (int k=0; k<numConfigurations; k++){
        // Rotate agent
        std::vector<Eigen::Vector2d> agentVertsRotated;
        Eigen::Matrix2d R;
        R << std::cos(rotate), -std::sin(rotate), std::sin(rotate), std::cos(rotate);
        for (int t=0; t<numAgentVerts; t++){
            agentVertsRotated.push_back(R*agentVerts[t]);
        };


        //Flip agent about reference point
        for (int t=1; t<numAgentVerts; t++){
            agentVertsRotated[t] = -1*agentVertsRotated[t];
        };

        // Put these back into Polygon so I can get them back with min y-pos first
        double minY=1000000;
        double minX=1000000;
        int index;
        for (int t=0; t<numAgentVerts; t++){
            Eigen::Vector2d vert = agentVertsRotated[t];
            if (vert[1]<minY && vert[0]<minX) {index = t; minY = vert[1]; minX = vert[0];};
        };

        std::vector<Eigen::Vector2d> agentVertsFinal;
        for (int t=0; t<numAgentVerts; t++){
            agentVertsFinal.push_back(agentVertsRotated[index]);

            index++;

            if (index==numAgentVerts){index=0;};
        };
        agentVertsFinal.push_back(agentVertsFinal[0]);


        int i=0; int j=0;
        std::vector<Eigen::Vector2d> cSpaceVertices;
        while ((i<numObsVerts) && (j<numAgentVerts+1)){
            // Add O_i, A_j
            Eigen::Vector2d O_i = obstacleVerts[i]; Eigen::Vector2d A_j = agentVertsFinal[j];
            Eigen::Vector2d O_i1 = obstacleVerts[i+1]; Eigen::Vector2d A_j1 = agentVertsFinal[j+1];

            // Add O_i, A_j to list of vertices
            cSpaceVertices.push_back(O_i+A_j);


            // Find angles vertices make
            double angle_O = std::atan2((O_i1[1]-O_i[1]), (O_i1[0]-O_i[0]));
            double angle_A = std::atan2((A_j1[1]-A_j[1]), (A_j1[0]-A_j[0]));

            // Round the angles to positive if necessary
            if (angle_O<0){angle_O = angle_O+2*std::numbers::pi;};
            if (angle_A<0){angle_A = angle_A+2*std::numbers::pi;};


            if (angle_O<angle_A){
                i++;
            } else if(angle_O>angle_A){
                j++;
            } else{
                i++; j++;
                std::cout << "Angles are the same" << std::endl;
            }
            
        }
        cspaceobstacles.obstacles.push_back(Polygon(cSpaceVertices));
        cspaceobstacles.heights.push_back(rotate);
        rotate = rotate+rotationStep;
    }
    
    return cspaceobstacles;

    };

}
