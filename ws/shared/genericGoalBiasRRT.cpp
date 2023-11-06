#include "genericGoalBiasRRT.h"

amp::genericGoalBiasRRT::genericGoalBiasRRT(int n, double r, double p, double e) : 
    m_n(n), m_r(r), m_p(p), m_e(e) {}

amp::Path amp::genericGoalBiasRRT::planND(const Eigen::VectorXd& init_state, 
const Eigen::VectorXd& goal_state, const std::unique_ptr<amp::ConfigurationSpace>& cspace) {

    amp::Path path;

    // First get boundaries of the cspace
    Eigen::VectorXd lowerBounds = (*cspace).lowerBounds();
    Eigen::VectorXd upperBounds = (*cspace).upperBounds();

    //Get dimensionality
    int N = (*cspace).dimension();

    int n = m_n; double r = m_r; double p = m_p; double e = m_e;    

    // Start at init_node
    bool goalReached =  false;
    amp::Graph<double> tree;
    std::map<amp::Node, Eigen::VectorXd> nodes;
    amp::Node initNode = 0;
    nodes[initNode] = init_state;

    int nodeNum = 1;

    int walkDis = 5;


    for (int i = 0; i<n; i++) {
        Eigen::VectorXd q_rand(N);

        for (int j = 0; j<N; j++) {
            bool acceptSample = false;
            double sample;
            while (!acceptSample) {
                sample = amp::RNG::nsrand(goal_state[j], 1-p);

                if (sample > lowerBounds[j] && sample < upperBounds[j]) {
                    acceptSample = true;
                }
            }
            q_rand[j] = sample;
        }

        // Check if sample is in collision
        bool collision = (*cspace).inCollision(q_rand);
        if (collision) {continue;};

        // If not in collision find nearest node
        double nodeDistMin = 10000;
        double nodeDist;
        amp::Node node_near;
        Eigen::VectorXd q_near;
        Eigen::VectorXd walkDir;
        for (int j = 0; j<nodeNum; j++) {
            Eigen::VectorXd vec2Node = q_rand - nodes[j];
            if (vec2Node.norm() < nodeDistMin) {
                nodeDistMin = vec2Node.norm();
                nodeDist = vec2Node.norm();
                walkDir = vec2Node/vec2Node.norm();
                node_near = j;
                q_near = nodes[j];
            }
        }

       

        // Now get q_new
        Eigen::VectorXd q_new = q_near + r*walkDir;

        // Now walk to q_new and make sure collision free
        collision = false;

        for (int k = 1; k<walkDis; k++) {
            Eigen::VectorXd walkLoc = (1 - k/walkDis)*q_near + (k/walkDis)*q_new;
            collision = (*cspace).inCollision(walkLoc);
            if (collision) {break;};
            
        }

        

        if (collision) {
            continue;
        } else {
            //Make connection
            tree.connect(node_near, nodeNum, nodeDist);
            nodes[nodeNum] = q_new;
            nodeNum++;

            Eigen::VectorXd vec2Goal = goal_state - q_new;

            if (vec2Goal.norm()< e) {
                tree.connect(nodeNum-1, nodeNum, vec2Goal.norm());
                nodes[nodeNum] = goal_state;
                std::cout << "Connected to goal" << std::endl;
                break;
            }


        }

    }

    

    amp::Node goalNode = nodeNum;

    // Now ask AStar to find me a path
    amp::MyAStarAlgo aStarAlgo;
    amp::ShortestPathProblem PRMProblem;
    std::shared_ptr<amp::Graph<double>> graph_ptr = std::make_shared<amp::Graph<double>>(tree);
    
    PRMProblem.graph = graph_ptr;
    PRMProblem.init_node = initNode;
    PRMProblem.goal_node = goalNode;
    
    amp::SearchHeuristic heuristic;
    amp::AStar::GraphSearchResult result = aStarAlgo.search(PRMProblem, heuristic);

    
    
    //Now pull out the points

    std::vector<amp::Node> node_path = {result.node_path.begin(), result.node_path.end()};
    int pathSize = node_path.size();
    for (int i = 0; i<pathSize; i++){
        path.waypoints.push_back(nodes[node_path[i]]);
    }

    return path;




}