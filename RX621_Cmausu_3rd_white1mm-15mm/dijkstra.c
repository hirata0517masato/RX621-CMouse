#include"dijkstra.h"

#define MAX_V 810     //16x16x3 + 16 + 16
#define MAX_E 4096  // 16x16x8 x2 無向グラフなので2倍のエッジ数を確保
#define DIST_MAX 9999


// グローバル変数（メモリ確保）
Edge edges[MAX_E];
int head[MAX_V];
int edge_idx;
HeapNode heap[MAX_E];
    
short dist[MAX_V];
int prev[MAX_V];

// --- 初期化 ---
void init_dijkstra() {
    for (int i = 0; i < MAX_V; i++) {
        head[i] = -1;
        dist[i] = DIST_MAX;
        prev[i] = -1;
    }
    edge_idx = 0;
}

// --- 辺の追加（無向）---
void add_edge(short from, short to, short cost) {
    edges[edge_idx] = (Edge){to, cost, head[from]};
    head[from] = edge_idx++;
    edges[edge_idx] = (Edge){from, cost, head[to]};
    head[to] = edge_idx++;
}

// --- ヒープ操作 ---
void push(HeapNode heap[], int* size, HeapNode node) {
    int i = (*size)++;
    while (i > 0) {
        int p = (i - 1) / 2;
        if (heap[p].dist <= node.dist) break;
        heap[i] = heap[p];
        i = p;
    }
    heap[i] = node;
}

HeapNode pop(HeapNode heap[], int* size) {
    HeapNode top = heap[0];
    HeapNode last = heap[--(*size)];
    int i = 0;
    while (i * 2 + 1 < *size) {
        int a = i * 2 + 1;
        int b = i * 2 + 2;
        int min = a;
        if (b < *size && heap[b].dist < heap[a].dist) min = b;
        if (heap[min].dist >= last.dist) break;
        heap[i] = heap[min];
        i = min;
    }
    heap[i] = last;
    return top;
}

// --- ダイクストラ ---
void run_dijkstra(short start) {

    int size = 0;

    dist[start] = 0;
    push(heap, &size, (HeapNode){start, 0});

    while (size > 0) {
        HeapNode hn = pop(heap, &size);
        int v = hn.vertex;
        if (dist[v] < hn.dist) continue;

        for (int i = head[v]; i != -1; i = edges[i].next) {
            short u = edges[i].to;
            short cost = edges[i].cost;
            if (dist[u] > dist[v] + cost) {
                dist[u] = dist[v] + cost;
                prev[u] = v;
                push(heap, &size, (HeapNode){u, dist[u]});
            }
        }
    }
}

short get_dist(short v){
    return dist[v];
}

// --- 経路復元（逆順で表示）---
void print_path(int to) {
    if (to == -1) return;
    print_path(prev[to]);
    //printf("%d ", to);
}