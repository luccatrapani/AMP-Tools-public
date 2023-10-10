 #pragma once

 #include <vector>
 #include "AMPCore.h"

namespace amp{

class makeCSpaceObstacles{
    public:
        struct CSpaceObstacle{
            std::vector<Polygon> obstacles;

            std::vector<double> heights;

        };

        static CSpaceObstacle makeObstacles(const Obstacle2D& obstacle, const Obstacle2D& agent, const int numConfigurations);
        

};

 }