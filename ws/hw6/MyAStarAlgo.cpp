#include "MyAStarAlgo.h"

bool cmp(const amp::MyAStarAlgo::PriorityQueue& a, const amp::MyAStarAlgo::PriorityQueue& b);

amp::MyAStarAlgo::GraphSearchResult amp::MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic){
    amp::Graph<double> graph = (*problem.graph);
    // Start two vector
    std::vector<amp::MyAStarAlgo::PriorityQueue> priorityList = 
        {{problem.init_node, problem.init_node, 0, 0}};
    std::vector<amp::MyAStarAlgo::PriorityQueue> processedList;
    bool goalReached = false;

    int iterations = 0;

    while (!goalReached) {
        amp::Node nBest = priorityList[0].currentNode;
        // Check if goal is reached
        if (nBest == problem.goal_node) {
            processedList.push_back(priorityList[0]);
            goalReached = true;
            break;
        }

        int g_n = priorityList[0].pathToNode;
        int lengthPriority = priorityList.size();
        // Get children of top of priority
        std::vector<amp::Node> childNodes = graph.children(nBest);
        std::vector<double> childEdges = graph.outgoingEdges(nBest);
        int lengthChildren = childNodes.size();

        // Make sure the child is not in the list already
        for (int i = 0; i<lengthChildren; i++){
            bool nodeExist = false;
            int existInd;
            for (int j = 0; j<lengthPriority; j++){
                if (childNodes[i] == priorityList[j].currentNode) {nodeExist = true; existInd = j;};
            }

            int pri = g_n + childEdges[i] + heuristic(childNodes[i]);
            int g_child = g_n + childEdges[i];

            if (!nodeExist) {
                priorityList.push_back({childNodes[i], nBest, g_child, pri});
            } else if ((g_n+childEdges[i]) < g_child){
                priorityList[existInd].parentNode = nBest;
                priorityList[existInd].pathToNode = g_n + childEdges[i];
                priorityList[existInd].priority = pri;
            }
        }

        // Now pop the top and sort
        processedList.push_back(priorityList[0]);
        priorityList.erase(priorityList.begin());

        std::sort(priorityList.begin(), priorityList.end(), cmp);
        iterations++;
        if (iterations > 1000) { return GraphSearchResult();};
    }
    std::cout << "A* iterations: " << iterations << std::endl;

    amp::MyAStarAlgo::GraphSearchResult result;
    result.success = true;
    int lengthProcess = processedList.size();
    amp::Node nC = processedList[lengthProcess-1].currentNode;
    amp::Node nP = processedList[lengthProcess-1].parentNode;
    std::vector<amp::Node> path;
    path.push_back(nC);

    bool pathFound = false;
    int r = 0;
    while(!pathFound) {
        for (int k = 0; k<lengthProcess; k++){
            if (nP == processedList[k].currentNode) {
                path.push_back(processedList[k].currentNode);
                nP = processedList[k].parentNode;
                if (processedList[k].currentNode == problem.init_node) {
                    pathFound = true;
                    break;
                }
                break;
            }

        }

        r++;
        if (r > 1000) {std::cout << "Broke" << std::endl; return GraphSearchResult();};

    }

    //reverse path
    std::reverse(path.begin(), path.end());

    int pathLength = path.size();

    std::cout << "Path is: ";
    for (int k = 0; k < pathLength; k++){
        result.node_path.push_back(path[k]);
        std::cout << path[k] << "->";
    }
    std::cout << std::endl;

    double cost  = processedList[lengthProcess-1].pathToNode;
    result.path_cost = cost;
    std::cout << "Path cost is: " << cost << std::endl;
    
    return result;
}


bool cmp(const amp::MyAStarAlgo::PriorityQueue& a, const amp::MyAStarAlgo::PriorityQueue& b)
{
    // smallest comes first
    return a.priority < b.priority;
}