#include <zmq.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>

int main (void)
{
    printf ("Connecting to hello world server…\n");
    void *context = zmq_ctx_new();
    void *pub = zmq_socket(context, ZMQ_PUB);
    int rc = zmq_bind(pub, "tcp://127.0.0.1:5558");
    if (rc != 0) {
      printf("Failed to connect.\n");
      return 1;
    }

    int request_nbr;
    char msg[7] = {'H', 'e', 'l', 'l', 'o', ' ', '\0'};
    int i = 0;
    for (request_nbr = 0; request_nbr != 40; request_nbr++) {
        // char buffer [10];
        printf ("Sending Hello %d…\n", request_nbr);
        msg[5] = i;
        zmq_send (pub, msg, 7, 0);
        ++i;
        usleep(1000*200);
    }
    zmq_close (pub);
    zmq_ctx_destroy (context);
    return 0;
}
