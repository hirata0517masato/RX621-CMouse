#include"Queue.h"

short queue[MAXqueue] = {0};
short queue_front = 0,queue_end = 0;
short queue_num = 0;

char queue_empty(){
    return (queue_num == 0);
}

void queue_reset(){
	queue_front = 0;
	queue_end = 0;
 	queue_num = 0;
}

void enqueue(short n){
    if(queue_num >= MAXqueue)return ;
    queue[queue_end++] = n;
    if(queue_end == MAXqueue)queue_end = 0;
    queue_num++;
}

short dequeue(){
    if(queue_empty())return -1;
    int r = queue[queue_front++];
    if(queue_front == MAXqueue)queue_front = 0;
    queue_num--;
    return r;
}

short queue_next(int n){
    n += queue_front-1;
    n %= MAXqueue;
    if(queue_empty())return -99;
    return queue[n];
}
