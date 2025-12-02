#ifndef POLYGON_H
#define POLYGON_H

#include <Eigen/Dense>
#include <iostream>
#include <iit/commons/dog/leg_data_map.h>

typedef struct Vertex
{
    Eigen::Vector3d pos;
    iit::dog::LegID legID;
};

typedef struct Edge
{
    Vertex v1;
    Vertex v2;
    double length;
    Eigen::Vector3d A;
    double b;
};

typedef struct Polygon
{
    int numberOfVertices;
    Vertex v1;
    Vertex v2;
    Eigen::Matrix<double, numberOfVertices, 3> A;
    Eigen::Vector<double, numberOfVertices> b;
};


#endif // POLYGON_H
