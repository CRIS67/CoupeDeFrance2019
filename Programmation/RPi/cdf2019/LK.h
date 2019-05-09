#ifndef LK_H
#define LK_H

#include <utility>
#include <list>
#include <vector>
#include <cmath>

// node (rank,id)
struct node {
    int rank;
    int id;
    std::pair<int,int> coord; 
};

// an edge (u,v)
struct edge {
    node u;
    node v;
};


class LK {
    private:
        int m_count; /*< number of k-opt moves to do */
        std::vector<node> m_solution; 
    public:
        LK(int c):m_count(c){};
        
        // optimize
        void optimize( std::vector<node> intialTour);
        // choose x1 
        bool choose_x1(node *tour, int n, bool *visited, node *t1,
                edge *x1);
        // choose y1
        bool choose_y1( node *tour, int n, bool *visited, 
                edge x1, node *t2, node *t3, edge *y1, int *G1);

        // choose xi
        bool choose_xi(node *tour, int n, bool *visited, node *t3, 
                edge *x2);
       
        // choose yi
        bool choose_yi( node *tour,int n, bool *visited,
                node *t4, edge x2, edge *y2,int *G2);

        // flip construct resulting tour
        void flip(node *tour, int n, std::list<edge> X);
       
        // store tour in tour_
        void store(node *tour_, node *tour, int n);

        // Returns distance between two nodes 
        double distance(node n1, node n2); 
        
        // Returns the solution 
        std::vector<node> getSolution(); 

        // Prints solution
        void printSolution(); 

        // Gets solution distance 
        double getDistance(); 

        double calculateDistance(std::vector<node> tour); 


};


#endif
