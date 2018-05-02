#include <queue>
#include <climits>
#include <set>
#include <iostream>
#include <fstream>
#include <map>

#include "graph.h"

/**
 * A graph is made up of vertices and edges
 * A vertex can be connected to other vertices via weighted, directed edge
 */


////////////////////////////////////////////////////////////////////////////////
// This is 80 characters - Keep all lines under 80 characters                 //
////////////////////////////////////////////////////////////////////////////////


/** constructor, empty graph */
Graph::Graph() {
    //initialize all member variables
    numberOfVertices = 0;
    numberOfEdges = 0;
    vertices = {};
}

/** destructor, delete all vertices and edges
 only vertices stored in map
 no pointers to edges created by graph */
Graph::~Graph() {
    //delete all edges and vertexes
    for(auto & vertex: this->vertices){
        //only vertexes use the new method so they are only ones needed to be
        //deleted
        delete vertex.second;
    }
}

/** return number of vertices */
int Graph::getNumVertices() const {
    //return int that keeps track of the number
    return numberOfVertices;
}

/** return number of vertices */
int Graph::getNumEdges() const {
    //return int that is retreived from file
    return numberOfEdges;
}

/** add a new edge between start and end vertex
 if the vertices do not exist, create them
 calls Vertex::connect
 a vertex cannot connect to itself
 or have multiple edges to another vertex */
bool Graph::add(std::string start, std::string end, int edgeWeight) {
    //cannot add something that is linked to itself like whut bro
    if (start == end){
        return false;
    }
    //4 different cases, when both dont exist, when one exists
    //when one does and one doesnt for start and end.
    if( vertices.count(start)==0 && vertices.count(end)==0 ){
        Vertex* s = new Vertex(start);
        Vertex* e = new Vertex(end);
        s->connect(end,edgeWeight);
        vertices.insert(std::pair<std::string,Vertex*>(start,s));
        vertices.insert(std::pair<std::string,Vertex*>(end,e));
        numberOfVertices += 2;
        return true;
    }else if( vertices.count(start)==1 && vertices.count(end)==0){
        Vertex* e = new Vertex(end);
        vertices.at(start)->connect(end,edgeWeight);
        vertices.insert(std::pair<std::string,Vertex*>(end,e));
        numberOfVertices += 1;
        return true;
    }else if( vertices.count(start)==0 && vertices.count(end)==1){
        Vertex* s = new Vertex(start);
        s->connect(end,edgeWeight);
        vertices.insert(std::pair<std::string,Vertex*>(start,s));
        numberOfVertices += 1;
        return true;
    }else {
        return vertices.at(start)->connect(end,edgeWeight);
    }
}

/** return weight of the edge between start and end
 returns INT_MAX if not connected or vertices don't exist */
int Graph::getEdgeWeight(std::string start, std::string end) const {
    //gets checks to see if they exist or there is an edge
    if( vertices.count(start)==0 || vertices.count(end)==0
       ||vertices.at(start)->getEdgeWeight(end)==-1){
        return INT_MAX;
    }
    return vertices.at(start)->getEdgeWeight(end);
}

/** read edges from file
 the first line of the file is an integer, indicating number of edges
 each edge line is in the form of "string string int"
 fromVertex  toVertex    edgeWeight */
void Graph::readFile(std::string filename) {
    //I hate file IO
    std::ifstream graphFile;
    graphFile.open(filename);
    int numberE = 0;
    std::string start;
    std::string end;
    int weight = 0;
    if(graphFile.fail()){
        std::cerr << "Error opening file" << std::endl;
        exit(1);
    }
    if(graphFile >> numberE){
        //std::cout <<"Edges" << numberE;
        numberOfEdges = numberE;
    }
    while(graphFile>>start>>end>>weight){
        this->add(start,end,weight);
    }
    
    graphFile.close();
    
}

/** depth-first traversal starting from startLabel
 call the function visit on each vertex label */
void Graph::depthFirstTraversal(std::string startLabel,
                                void visit(const std::string&)) {
    //references algorithmn on ass3 page
    unvisitVertices();
    depthFirstTraversalHelper(vertices.at(startLabel),visit);
    
}

/** breadth-first traversal starting from startLabel
 call the function visit on each vertex label */
void Graph::breadthFirstTraversal(std::string startLabel,
                                  void visit(const std::string&)) {
    //references algorithmn on ass3 page
    unvisitVertices();
    std::queue<Vertex*> holder;
    holder.push(vertices.at(startLabel));
    vertices.at(startLabel)->visit();
    visit(startLabel);
    while (!holder.empty()){
        Vertex* m = holder.front();
        holder.pop();
        for (int i = 0; i < m->getNumberOfNeighbors();i++){
            Vertex *n = findVertex(m->getNextNeighbor());
            if (n->isVisited()){
                //skips this value
            }else if(!n->isVisited()){
                visit(n->getLabel());
                n->visit();
                holder.push(n);
            }
        }
    }
}

/** find the lowest cost from startLabel to all vertices that can be reached
 using Djikstra's shortest-path algorithm
 record costs in the given map weight
 weight["F"] = 10 indicates the cost to get to "F" is 10
 record the shortest path to each vertex using given map previous
 previous["F"] = "C" indicates get to "F" via "C"

 cpplint gives warning to use pointer instead of a non-const map
 which I am ignoring for readability */
void Graph::djikstraCostToAllVertices(std::string startLabel,
                                      std::map<std::string,
                                      int>& weight,
                                      std::map<std::string,
                                      std::string>& previous) {
    //weight.emplace(startLabel,0);
    //previous.emplace(startLabel,startLabel);
    //references algorithmn on ass3 page
    weight.clear();
    previous.clear();
    std::priority_queue<std::pair<std::string,int>,
    std::vector<std::pair<std::string, int>>,
    std::greater<std::pair<std::string, int>>> pq;
    unvisitVertices();
    std::set<std::string> vertexSet;
    for (int i = 0; i < findVertex(startLabel)->getNumberOfNeighbors();i++){
        std::string u = findVertex(startLabel)->getNextNeighbor();
        weight[u] = findVertex(startLabel)->getEdgeWeight(u);
        previous[u] = startLabel;
        pq.push(std::pair<std::string,int>(u,weight[u]));
    }
    vertexSet.insert(startLabel);
    while(!pq.empty()){
        std::string v = pq.top().first;
        pq.pop();
        if (vertexSet.count(v)==0){
            for (int i = 0; i < findVertex(v)->getNumberOfNeighbors();i++){
                std::string u = findVertex(v)->getNextNeighbor();
                int v2ucost = getEdgeWeight(v, u);
                if(weight.count(u)==0&&u!=startLabel){
                    weight[u] = weight[v]+v2ucost;
                    previous[u]=v;
                    pq.push(std::pair<std::string,int>(u,weight[u]));
                }
                else{
                    if(weight[u]>weight[v]+v2ucost&&u!=startLabel){
                        weight[u] = weight[v] + v2ucost;
                        previous[u] = v;
                        pq.push(std::pair<std::string,int>(u,weight[u]));
                    }else{
                    }
                }
            }
        vertexSet.insert(v);
        }
    }
    weight.erase(startLabel);
}

/** helper for depthFirstTraversal */
void Graph::depthFirstTraversalHelper(Vertex* startVertex,
                                      void visit(const std::string&)) {
    //references algorithmn on ass3 page
    startVertex->visit();
    visit(startVertex->getLabel());
    for( int i = 0; i < startVertex->getNumberOfNeighbors(); i++ ){
        Vertex *n = findVertex(startVertex->getNextNeighbor());
        if(!n->isVisited()){
            depthFirstTraversalHelper(n, visit);
        }
    }
}

/** mark all verticies as unvisited */
void Graph::unvisitVertices() {
    //goes through the whole thing to call the unvisit
    for(auto & vertex: this->vertices){
        vertex.second->unvisit();
        vertex.second->resetNeighbor();
    }
}

/** find a vertex, if it does not exist return nullptr */
Vertex* Graph::findVertex(const std::string& vertexLabel) const {
    if(vertices.count(vertexLabel)==1){
        return vertices.at(vertexLabel);
    }
    return nullptr;
}

/** find a vertex, if it does not exist create it and return it */
Vertex* Graph::findOrCreateVertex(const std::string& vertexLabel) {
    if (findVertex(vertexLabel)==nullptr){
        Vertex* v = new Vertex(vertexLabel);
        //I was not sure if we had to create it and then add it to the
        //map,
        //vertices.insert(std::pair<std::string,Vertex*>(vertexLabel,v));
        //numberOfVertices+=1;
        return v;
    }
    return findVertex(vertexLabel);
}
