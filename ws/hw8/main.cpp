#include "AMPCore.h"
#include "hw/HW8.h"
#include "MyCentralizedMultiAgentRRT.h"

using namespace amp;

void benchmark();

void benchmark() {
    std::vector<double> meanTree;
    std::vector<double> meanTime;

    std::list<std::vector<double>> treeSizePlot;
    std::list<std::vector<double>> timePlot;

    std::vector<std::string> plotLabels = {"2", "3", "4", "5", "6"};

    for (int m = 2; m<7; m++){
        Eigen::VectorXd lowerBounds(2*m);
        Eigen::VectorXd upperBounds(2*m);
        for (int k = 0; k<2*m; k++){
            lowerBounds[k] = 0;
            upperBounds[k] = 16;
        }

        std::vector<double> treeSize;
        std::vector<double> time;
        std::list<std::vector<double>> plotList;

        for (int i = 1; i<101; i++){
            int n = 30000; double r = .5; double p = .05; double e = .25;
            MyCentralizedMultiAgentRRT algo(lowerBounds, upperBounds, n, r, p, e);
            MultiAgentProblem2D problem1 = HW8::getWorkspace1(m);
            //Visualizer::makeFigure(problem1);
            MultiAgentPath2D path1 = algo.plan(problem1);

            if (!path1.valid) {
                treeSize.push_back(n);
                time.push_back(algo.m_time);
            } else{
                treeSize.push_back(algo.m_RRTTree);
                time.push_back(algo.m_time);
            }

        }
        treeSizePlot.push_back(treeSize);
        timePlot.push_back(time);

        meanTree.push_back(std::accumulate(treeSize.begin(), treeSize.end(), 0.0) / treeSize.size());
        meanTime.push_back(std::accumulate(time.begin(), time.end(), 0.0) / time.size());



    }


    Visualizer::makeBoxPlot(treeSizePlot, plotLabels, "Treesize For m agents over 100 iterations", "m agents", "Treesize");
    Visualizer::makeBoxPlot(timePlot, plotLabels, "Time For m agents over 100 iterations", "m agents", "Time");
    
    Visualizer::makeBarGraph(meanTree, plotLabels, "Mean Tree Size", "m agents", "Mean Tree Size");
    Visualizer::makeBarGraph(meanTime, plotLabels, "Mean Time", "m agents", "Mean Time");
}


int main(int argc, char** argv) {
    amp::RNG::seed(amp::RNG::randiUnbounded());
    
    /*
    Eigen::VectorXd lowerBounds(4);
    lowerBounds << 0, 0, 0, 0;
    Eigen::VectorXd upperBounds(4);
    upperBounds << 16, 16, 16, 16;
    int n = 7500; double r = .5; double p = .05; double e = .25;
    MyCentralizedMultiAgentRRT algo(lowerBounds, upperBounds, n, r, p, e);
    MultiAgentProblem2D problem1 = HW8::getWorkspace1(2);
    //Visualizer::makeFigure(problem1);
    MultiAgentPath2D path1 = algo.plan(problem1);
    Visualizer::makeFigure(problem1, path1);
    
    bool res = HW8::check(path1, problem1);

    std::cout << "Tree: " << algo.m_RRTTree << "  Time: " << algo.m_time << std::endl;
    */

    benchmark();
    Visualizer::showFigures();

    return 0;
}

