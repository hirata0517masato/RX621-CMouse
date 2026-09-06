#ifndef QUEUE_H
#define QUEUE_H

#define MAXqueue 256

char queue_empty(void);
void queue_reset(void);
void enqueue(short);
short dequeue(void);
short queue_next(int);

#endif