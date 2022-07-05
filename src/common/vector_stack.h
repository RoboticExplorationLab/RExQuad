#pragma once

#define REXQUAD_QUEUE_SIZE 20

typedef double* Vector;

typedef struct {
  Vector _data[REXQUAD_QUEUE_SIZE];
  int front;
  int back;
  int vector_length;
} rexquad_VectorQueue;

rexquad_VectorQueue rexquad_VectorQueueCreate(void* buf, int vector_length);

int rexquad_VectorQueueSize(const rexquad_VectorQueue* queue);

void rexquad_VectorQueuePush(rexquad_VectorQueue* queue, const double* vec);

void rexquad_VectorQueuePop(rexquad_VectorQueue* queue);

const double* rexquad_VectorQueueGet(const rexquad_VectorQueue* queue, int index);
