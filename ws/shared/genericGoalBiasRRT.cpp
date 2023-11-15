#include "genericGoalBiasRRT.h"

bool inGoal(Eigen::VectorXd& q_i, Eigen::VectorXd& q_goal, int N, double e);

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
    std::cout << "Dimension:  " << N << std::endl;


    int n = m_n; double r = m_r; double p = m_p; double e = m_e;    

    // Start at init_node
    bool RRTPassed =  false;
    amp::Graph<double> tree;
    std::map<amp::Node, Eigen::VectorXd> nodes;
    amp::Node initNode = 0;
    nodes[initNode] = init_state;
    Eigen::VectorXd q_goal = goal_state;

    int nodeNum = 1;

    int walkDis = 20;

    for (int i = 0; i<n; i++) {

        Eigen::VectorXd q_rand(N);

        double pSample = amp::RNG::srandd(0, 1);
        if (pSample <= p) {
            q_rand = goal_state;
        } else {
            for (int j = 0; j<N; j++) {
                q_rand[j] = amp::RNG::srandd(lowerBounds[j], upperBounds[j]);
            }
        }

        

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
        // Check if sample is in collision
        bool collision = (*cspace).inCollision(q_new);
        if (collision) {continue;};
        

        // Now walk to q_new and make sure collision free
        collision = false;

        for (int k = 0; k<=walkDis; k++) {
            Eigen::VectorXd walkLoc = (1 - k/walkDis)*q_near + (k/walkDis)*q_new;
            collision = (*cspace).inCollision(walkLoc);
            if (collision) {/*std::cout << "WALK " << i << std::endl; std::cout << walkLoc << std::endl;*/ break;};
            
        }

        //std::cout << "i :  " << i << "   norm:   " << nodeDist << std::endl;

        if (collision) {
            //std::cout << "Collision " << std::endl;
            /*std::cout << "Collide q_near : " << std::endl;
            std::cout << q_near << std::endl;
            std::cout << "q_rand" << std::endl;
            std::cout << q_rand << std::endl;*/
            continue;
            
        } else {
            //Make connection
            tree.connect(node_near, nodeNum, nodeDist);
            nodes[nodeNum] = q_new;
            nodeNum++;

            Eigen::VectorXd vec2Goal = goal_state - q_new;

            //std::cout << "i:   " << i << "   norm goal:   " << vec2Goal.norm() << std::endl;
            bool goalReached = inGoal(q_new, q_goal, N, e);
            if (goalReached) {
                tree.connect(nodeNum-1, nodeNum, vec2Goal.norm());
                nodes[nodeNum] = goal_state;
                std::cout << "Connected to goal" << std::endl;
                RRTPassed = true;
                break;
            }


        }


    }

    if (!RRTPassed) {
        std::cout << "RRT Did Not Return A Result" << std::endl;
        path.valid = false;
        return path;
    }

    
    amp::Visualizer::showFigures();
    amp::Node goalNode = nodeNum;
    m_treeSize = nodeNum+1;

    // Now ask AStar to find me a path
    amp::MyAStarAlgo aStarAlgo;
    amp::ShortestPathProblem PRMProblem;
    std::shared_ptr<amp::Graph<double>> graph_ptr = std::make_shared<amp::Graph<double>>(tree);
    
    PRMProblem.graph = graph_ptr;
    PRMProblem.init_node = initNode;
    PRMProblem.goal_node = goalNode;
    
    amp::SearchHeuristic heuristic;
    
    amp::AStar::GraphSearchResult result = aStarAlgo.search(PRMProblem, heuristic);
    
    if (!result.success) {
        std::cout << "A Star Could not give a solution" << std::endl;
        path.valid = false;
        return path;
    }
    
    
    //Now pull out the points

    std::vector<amp::Node> node_path = {result.node_path.begin(), result.node_path.end()};
    int pathSize = node_path.size();
    for (int i = 0; i<pathSize; i++){
        path.waypoints.push_back(nodes[node_path[i]]);
    }
    path.valid = true;

    return path;




}

bool inGoal(Eigen::VectorXd& q_i, Eigen::VectorXd& q_goal, int N, double e){
    bool goalReached = false;
    Eigen::VectorXd vec2Goal = q_i - q_goal;

    if (vec2Goal.norm() < e){
        goalReached = true;
    } else {
        bool inGoal = true;
        for (int j = 0; j<N; j+=2){
            Eigen::Vector2d q_iInd = {q_i[j], q_i[j+1]};
            Eigen::Vector2d q_GoalInd = {q_goal[j], q_goal[j+1]};
            Eigen::Vector2d vec2GoalInd = q_iInd - q_GoalInd;

            /*std::cout << "q_i :  " << std::endl;
            std::cout << q_iInd << std::endl;
            std::cout << "q_goal " << std::endl;
            std::cout << q_GoalInd << std::endl;*/

            if (vec2GoalInd.norm() >= e){
                inGoal = false;
            }
        }

        goalReached = inGoal;
    }

    return goalReached;

}