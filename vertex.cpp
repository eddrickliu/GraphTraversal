

#include <climits>

#include "vertex.h"


#include <functional>
#include <map>
#include <string>

#include "edge.h"


////////////////////////////////////////////////////////////////////////////////
// This is 80 characters - Keep all lines under 80 characters                 //
////////////////////////////////////////////////////////////////////////////////


/** Creates an unvisited vertex, gives it a label, and clears its
 adjacency list.
 NOTE: A vertex must have a unique label that cannot be changed. */
Vertex::Vertex(std::string label) {
    adjacencyList.clear();
    //checks to see if inputs are wrong
    if(label == "" || label == " " ){
        return;
    }
    vertexLabel = label;
}

/** @return  The label of this vertex. */
std::string Vertex::getLabel() const {
    return vertexLabel;
}

/** Marks this vertex as visited. */
void Vertex::visit() {
    visited = true;
}

/** Marks this vertex as not visited. */
void Vertex::unvisit() {
    visited = false;
}

/** Returns the visited status of this vertex.
 @return  True if the vertex has been visited, otherwise
 returns false/ */
bool Vertex::isVisited() const {
    return visited;
    
}

/** Adds an edge between this vertex and the given vertex.
 Cannot have multiple connections to the same endVertex
 Cannot connect back to itself
 @return  True if the connection is successful. */
bool Vertex::connect(const std::string& endVertex, const int edgeWeight) {
    //connects if the edgeweight is correct and its not connecting back
    //to itself
    if (edgeWeight < 0||endVertex==getLabel()){
        return false;
    }
    //also checks if it is already there
    if (adjacencyList.count(endVertex)==1){
        return false;
    }
    //if(adjacencyList)
    Edge newEdge(endVertex, edgeWeight);
    //not sure if you are supposed to put the endVertex here for now
    //adjacencyList.insert(std::pair<std::string,Edge>(endVertex,newEdge));
    adjacencyList[endVertex] = newEdge;
    resetNeighbor();
    return true;
}

/** Removes the edge between this vertex and the given one.
 @return  True if the removal is successful. */
bool Vertex::disconnect(const std::string& endVertex) {
    //cant disconnect if you cant find it
    if (adjacencyList.count(endVertex)==0){
        return false;
    }
    adjacencyList.erase(adjacencyList.find(endVertex));
    resetNeighbor();
    return true;
}

/** Gets the weight of the edge between this vertex and the given vertex.
 @return  The edge weight. This value is zero for an unweighted graph and
 is negative if the .edge does not exist */
int Vertex::getEdgeWeight(const std::string& endVertex) const {
    //cant getEdgeWeight when there its not there
    if (adjacencyList.count(endVertex)==0){
        return -1;
    }
    return adjacencyList.at(endVertex).getWeight();
}

/** Calculates how many neighbors this vertex has.
 @return  The number of the vertex's neighbors. */
int Vertex::getNumberOfNeighbors() const {
    return (int)adjacencyList.size();
}

/** Sets current neighbor to first in adjacency list. */
void Vertex::resetNeighbor() {
    //reseting is very helpful
    currentNeighbor = adjacencyList.begin();
}

/** Gets this vertex's next neighbor in the adjacency list.
 Neighbors are automatically sorted alphabetically via map
 Returns the vertex label if there are no more neighbors
 @return  The label of the vertex's next neighbor. */
std::string Vertex::getNextNeighbor() {
    //references algoithm from slides
    if(currentNeighbor != adjacencyList.end()){
        std::string next = currentNeighbor->second.getEndVertex();
        ++currentNeighbor;
        return next;
    }else{
        return getLabel();
    }
}

/** Sees whether this vertex is equal to another one.
 Two vertices are equal if they have the same label. */
bool Vertex::operator==(const Vertex& rightHandItem) const {
    if (rightHandItem.getLabel() == this->getLabel()){
        return true;
    }
    return false;
}

/** Sees whether this vertex is < another one.
 Compares vertexLabel. */
bool Vertex::operator<(const Vertex& rightHandItem) const {
    if (this->getLabel() < rightHandItem.getLabel()){
        return true;
    }
    return false;
}

