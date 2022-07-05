#include "vector_stack.h"

#include <string.h>

 rexquad_VectorQueue rexquad_VectorQueueCreate(void* buf, int vector_length) {
  rexquad_VectorQueue q;
  q.front = 0;
  q.back = 0;
  q.vector_length = vector_length;

  // Assign vector data from a pre-allocated memory buffer
  double* dbuf = (double*)buf;
  for (int i = 0; i < REXQUAD_QUEUE_SIZE; ++i) {
    q._data[i] = dbuf + i * vector_length;
  }
  return q;
}
 int rexquad_VectorQueueSize(const rexquad_VectorQueue* queue) {
   int back = queue->back;
   int front = queue->front;
   if (back < front) {
     back += REXQUAD_QUEUE_SIZE;
   }
   return back - front;
 }
 void rexquad_VectorQueuePush(rexquad_VectorQueue* queue, const double* vec) {
   ++queue->back;
   if (queue->back >= REXQUAD_QUEUE_SIZE) {
     queue->back = queue->back % REXQUAD_QUEUE_SIZE;
   }
   // Copy vector into queue
   double* dest = queue->_data[queue->back];
   memcpy(dest, vec, queue->vector_length * sizeof(double));
 }
 void rexquad_VectorQueuePop(rexquad_VectorQueue* queue) {
   ++queue->front;
   if (queue->front >= REXQUAD_QUEUE_SIZE) {
     queue->front = queue->front % REXQUAD_QUEUE_SIZE;
   }
 }
const double* rexquad_VectorQueueGet(const rexquad_VectorQueue* queue, int index) {
   int len = rexquad_VectorQueueSize(queue);
   if (index >= len) {
     return NULL;
   }
   int i = (queue->back - index + REXQUAD_QUEUE_SIZE) % REXQUAD_QUEUE_SIZE;
   return queue->_data[i];
 }
