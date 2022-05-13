#include <zmq.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <assert.h>

int main (void)
{
    //  Socket to talk to clients
    void *context = zmq_ctx_new();
    void *sub = zmq_socket (context, ZMQ_SUB);
    int rc;

    int conflate = 1;
    rc = zmq_setsockopt(sub, ZMQ_CONFLATE, &conflate, sizeof(conflate));
    if (rc != 0) {
      printf("Failed to set conflate.\n");
      return 1;
    }
    rc = zmq_connect(sub, "tcp://127.0.0.1:5558");
    if (rc != 0) {
      printf("Failed to connect.\n");
      return 1;
    }
    rc = zmq_setsockopt(sub, ZMQ_SUBSCRIBE, "", 0);
    if (rc != 0) {
      printf("Failed to set subscriber.\n");
      return 1;
    }

    printf("Waiting for messages...\n");
    int i = 0;
    // int event;
    // size_t event_size = sizeof(event);
    while (1) {
        char buffer [10];
        zmq_recv (sub, buffer, 10, 0);
        i = buffer[5];
        printf ("Received Hello %d\n", i);
        sleep(1);
    }
    zmq_close(sub);
    zmq_ctx_destroy(context);
    return 0;
}
