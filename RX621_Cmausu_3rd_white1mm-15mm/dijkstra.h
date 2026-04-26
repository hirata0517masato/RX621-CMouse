#ifndef DIJKSTRA_H
#define DIJKSTRA_H



typedef struct {
    short to;
    short cost;
    short next;
} Edge;

typedef struct {
    short vertex;
    short dist;
} HeapNode;


void init_dijkstra(void);
void add_edge(short, short, short );
void run_dijkstra(short);
short get_dist(short);

#endif