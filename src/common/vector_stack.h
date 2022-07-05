#pragma once

#define REXQUAD_QUEUE_SIZE 20

typedef const double* ConstVector;

typedef struct {
  ConstVector _data[REXQUAD_QUEUE_SIZE];
  int front;
  int back;
} rexquad_VectorQueue;

rexquad_VectorQueue rexquad_VectorQueueCreate();

int rexquad_VectorQueueSize(const rexquad_VectorQueue* queue);

void rexquad_VectorQueuePush(rexquad_VectorQueue* queue, ConstVector vec);

void rexquad_VectorQueuePop(rexquad_VectorQueue* queue);

ConstVector rexquad_VectorQueueGet(const rexquad_VectorQueue* queue, int index);
