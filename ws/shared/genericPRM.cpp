#include "genericPRM.h"

amp::Path amp::genericPRM::planND(const Eigen::VectorXd& init_state, 
    const Eigen::VectorXd& goal_state, const std::unique_ptr<amp::ConfigurationSpace>& cspace){

    amp::Path path;

    // First get boundaries of the cspace
    Eigen::VectorXd lowerBounds = (*cspace).lowerBounds();
    Eigen::VectorXd upperBounds = (*cspace).upperBounds();

    //Get dimensionality
    int N = (*cspace).dimension();

    int n = 200; double r = 1;

    

    // Initialize sample storage
    std::map<amp::Node, Eigen::VectorXd> samples;
    
    samples[0] = init_state;
    

    std::cout << lowerBounds << upperBounds << std::endl;
    // Samples
    double nodeNum = 1;
    for (int i = 1; i<=n; i++){
        Eigen::VectorXd randomSample(N);
        for(int j = 0; j<N; j++) {
            
            double sample1D = amp::RNG::srandd(lowerBounds[j], upperBounds[j]);
            randomSample[j] = sample1D;
            
        }
        bool collision = (*cspace).inCollision(randomSample);
        if (!collision){
            samples[nodeNum] = randomSample;
            nodeNum++;
        }
        
    }

    std::cout << nodeNum<< std::endl;
    samples[nodeNum] = goal_state;
    amp::Node initNode = 0;
    amp::Node goalNode = nodeNum;

    

    int numberOfNodes = nodeNum+1;
    // Now I have my random samples, let make some connections
    amp::Graph<double> graph;

    // Step size to walk along edge
    double walkDis = 5;
    for (int i = 0; i<numberOfNodes; i++) {
        amp::Node currentNode = i;
        for (int j = i; j<numberOfNodes; j++) {
            if (i==j) {
                continue;
            };

            // Check if nodes are close enough, continue if not
            amp::Node checkNode = j;
            
            //std::cout << i << " " << j << std::endl;
            Eigen::VectorXd nodeVec = samples[currentNode] - samples[checkNode];
            double nodeDist = nodeVec.norm();
            if (nodeDist > r) {continue;};

            // If they are close enough, try to connect them by walking along connection and check for collisions
            bool collision = false;
            for (int k = 1; k<walkDis; k++) {
                Eigen::VectorXd walkLoc = (1 - k/walkDis)*samples[currentNode] + (k/walkDis)*samples[checkNode];
                collision = (*cspace).inCollision(walkLoc);
                if (collision) {break;};
            }

            if (collision) {
                continue;
            } else {
                //Make connection
                if (checkNode == goalNode) {
                    std::cout << "Connected to goal node" << std::endl;
                }
                graph.connect(currentNode, checkNode, nodeDist);
            }


        }
    }

    
    

    // Now I have my graph, call the visualizer to view connectivity

    //amp::Visualizer::makeFigure(cspace->m_problem, graph, samples);

    // Now ask AStar to find me a path
    amp::MyAStarAlgo aStarAlgo;
    amp::ShortestPathProblem PRMProblem;
    std::shared_ptr<amp::Graph<double>> graph_ptr = std::make_shared<amp::Graph<double>>(graph);
    
    PRMProblem.graph = graph_ptr;
    PRMProblem.init_node = initNode;
    PRMProblem.goal_node = goalNode;
    
    amp::SearchHeuristic heuristic;
    amp::AStar::GraphSearchResult result = aStarAlgo.search(PRMProblem, heuristic);

    
    
    //Now pull out the points

    std::vector<amp::Node> node_path = {result.node_path.begin(), result.node_path.end()};
    int pathSize = node_path.size();
    for (int i = 0; i<pathSize; i++){
        path.waypoints.push_back(samples[node_path[i]]);
    }

    return path;




}