#include "hw/HW6.h"
#include "AMPCore.h"

namespace amp{
    class MyAStarAlgo : public amp::AStar {
    public:
        virtual GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
};
}