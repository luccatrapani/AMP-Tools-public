#include "BugTools.h"

bool BugTools::checkCollision(const amp::Problem2D& problem, const Eigen::Vector2d currentPosition){
    int numObstacles = problem.obstacles.size(); //Get number of obstacles
    double x = currentPosition[0]; double y = currentPosition[1];  //Extract current position

    bool inside;
    
    //Loop through each obstacle (polygon)
    for (int k=0; k<numObstacles; k++){
        //For each polygon, get CCW ordered list of vertices
        std::vector<Eigen::Vector2d> vertices = problem.obstacles[k].verticesCCW();
        //Find number of vertices
        int numVertices = vertices.size();

        // Initialize boolean collision value
        inside = false;

        // Loop through the vertices and perform collision checking 
        //j vertex is always previous vertex, i vertex is always current vertex
        for(int i=0, j = numVertices-1; i<numVertices; j = i++){
            // If statement to check if current position is innside
            /*First check if position is between y coordinates of the edge, if it is not, it cannot be inside
            Then check if the point lies to the left or right of the edge, imagine casting a ray to the right
            If position is to the left of the edge, cast ray has 1 crossing, change bool inside to true
            If repeated with another edge and another crossing, Gauss Jordan thereom, change inside to false*/
            Eigen::Vector2d vertex_i = vertices[i]; Eigen::Vector2d vertex_j = vertices[j];
            double x_i = vertex_i[0]; double x_j = vertex_j[0]; double y_i = vertex_i[1]; double y_j = vertex_j[1];

            if ((y_i > y) != (y_j > y) && (x < (x_j - x_i) * (y - y_i) / (y_j - y_i) + x_i)){
                inside = !inside;
            }
        }

        if (inside == true){
            break;
        }

    }


    return inside;

}


Eigen::Vector2d BugTools::TurnRight(const Eigen::Vector2d currentHeading){
    double dTheta = -.0175;  //radian
    Eigen::Matrix2d R;
    R << std::cos(dTheta), -std::sin(dTheta), std::sin(dTheta), std::cos(dTheta);

    Eigen::Vector2d heading = R*currentHeading;
    
    return heading;
}

Eigen::Vector2d BugTools::TurnLeft(const Eigen::Vector2d currentHeading){
    double dTheta = .0175;  //radian
    Eigen::Matrix2d R;
    R << std::cos(dTheta), -std::sin(dTheta), std::sin(dTheta), std::cos(dTheta);

    Eigen::Vector2d heading = R*currentHeading;
    
    return heading;
}


