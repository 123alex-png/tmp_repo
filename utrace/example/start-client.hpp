#ifndef START_CLIENT_HPP
#define START_CLIENT_HPP
#include <cstdio>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

void start_client(int port) {
    printf("%d\n", getpid());
    // Create socket for client
    int client_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (client_socket < 0) {
        perror("socket");
        return;
    }

    // Connect to server
    struct sockaddr_in server_address;
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(port);
    server_address.sin_addr.s_addr = INADDR_ANY;
    if (connect(client_socket, (struct sockaddr*)&server_address,
                sizeof(server_address)) < 0) {
        perror("connect");
        return;
    }

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

#endif
