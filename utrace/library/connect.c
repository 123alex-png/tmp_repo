// TODO: change to dynamic link
// TODO: change to dynamic link
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

static int portSocket(int port) {
    printf("start client\n");
    if (!access("/tmp/utrace", F_OK))
        return -1;
    // Create socket for client
    int client_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket < 0) {
        perror("socket");
        return -1;
    }
    // Connect to server
    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(port);
    server_address.sin_addr.s_addr = INADDR_ANY;
    if (connect(client_socket, (struct sockaddr*)&server_address,
                sizeof(server_address)) < 0) {
        perror("connect");
        return -1;
    }
    return client_socket;
}

static int unixDomainSocket(const char* port) {
    int client_socket = socket(AF_UNIX, SOCK_STREAM, 0);
    if (client_socket == -1)
        return -1;

    // Specify the socket address
    struct sockaddr_un addr;
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, "/tmp/utrace", sizeof(addr.sun_path) - 1);

    // Connect to the server
    if (connect(client_socket, (struct sockaddr*)(&addr), sizeof(addr)) == -1) {
        perror("connect");
        printf("FAIL!\n");
        close(client_socket);
        return -1;
    }
    return client_socket;
}

static void start_client() {
    const char* port = getenv("UTRACE_PORT");
    if (!port)
        return;
    char* endptr = NULL;
    int port_num = strtol(port, &endptr, 10);
    int client_socket = -1;
    if (endptr && endptr[0] == '\0')
        client_socket = portSocket(port_num);
    else
        client_socket = unixDomainSocket(port);
    if (client_socket == -1)
        return;
    // Send message to server
    struct {
        int pid;
        unsigned magicnumber;
    } data = {getpid(), 0xdeadbeef};
    send(client_socket, &data, sizeof(data), 0);

    // Receive message from server
    unsigned succ;
    do {
        recv(client_socket, &succ, sizeof(succ), 0);
    } while (succ != 0xdeadbeef);

    close(client_socket);
    return;
}

void __attribute__((constructor)) init() { start_client(); }
