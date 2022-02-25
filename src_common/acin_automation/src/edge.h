#ifndef EDGE_H
#define EDGE_H

class Vertex;
#include <moveit/move_group_interface/move_group_interface.h>

class Edge {
public:
    Vertex *start;
    Vertex *end;
    double length;
    bool exact;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    Edge(Vertex *start, Vertex *end);
    void calculateApproxLength();
    void calculateExactLength();
};

#endif
