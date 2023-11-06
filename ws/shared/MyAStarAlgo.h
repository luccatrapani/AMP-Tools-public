#include "hw/HW6.h"
#include "AMPCore.h"
#include <algorithm>

namespace amp{
    class MyAStarAlgo : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;

        
        struct PriorityQueue {
            amp::Node currentNode;
            amp::Node parentNode;
            int pathToNode;
            int priority;   
        };

};
}