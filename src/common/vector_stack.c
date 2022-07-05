#include "vector_stack.h"

#include <string.h>

 rexquad_VectorQueue rexquad_VectorQueueCreate() {
  rexquad_VectorQueue q;
  memset(q._data, 0, REXQUAD_QUEUE_SIZE * sizeof(double*));
  q.front = 0;
  q.back = 0;
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
 void rexquad_VectorQueuePush(rexquad_VectorQueue* queue, ConstVector vec) {
   ++queue->back;
   if (queue->back >= REXQUAD_QUEUE_SIZE) {
     queue->back = queue->back % REXQUAD_QUEUE_SIZE;
   }
   queue->_data[queue->back] = vec;
 }
 void rexquad_VectorQueuePop(rexquad_VectorQueue* queue) {
   ++queue->front;
   if (queue->front >= REXQUAD_QUEUE_SIZE) {
     queue->front = queue->front % REXQUAD_QUEUE_SIZE;
   }
 }
 ConstVector rexquad_VectorQueueGet(const rexquad_VectorQueue* queue, int index) {
   int len = rexquad_VectorQueueSize(queue);
   if (index >= len) {
     return NULL;
   }
   int i = (queue->back - index + REXQUAD_QUEUE_SIZE) % REXQUAD_QUEUE_SIZE;
   return queue->_data[i];
 }
