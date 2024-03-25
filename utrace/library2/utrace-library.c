#include "utrace.h"
#include <errno.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

static int socketfd = -2;

static const unsigned long long magic1 = 0x12345678, magic2 = 0x87654321,
                                magic3 = 0xdeadbeef;

static void cleanup() {
    if (socketfd >= 0)
        close(socketfd);
    socketfd = -2;
}

static int portSocket(int port) {
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
    strncpy(addr.sun_path, port, sizeof(addr.sun_path) - 1);

    // Connect to the server
    if (connect(client_socket, (struct sockaddr*)(&addr), sizeof(addr)) == -1) {
        perror("connect");
        close(client_socket);
        return -1;
    }
    return client_socket;
}

static void safe_write(int fd, const void* buf, size_t count) {
    while (count > 0) {
        ssize_t written = write(fd, buf, count);
        if (written < 0) {
            if (errno == EINTR)
                continue;
            perror("write");
            return;
        }
        count -= written;
        buf = (const char*)buf + written;
    }
}

static int startClient() {
    const char* port = getenv("UTRACE_PORT");
    if (!port)
        return -1;
    char* endptr = NULL;
    int port_num = strtol(port, &endptr, 10);
    int socketfd = -1;
    if (endptr && endptr[0] == '\0')
        socketfd = portSocket(port_num);
    else
        socketfd = unixDomainSocket(port);
    if (socketfd == -1)
        perror("socket");
    else {
        atexit(cleanup);
        safe_write(socketfd, &magic1, sizeof(magic1));
        pid_t pid = getpid();
        safe_write(socketfd, &pid, sizeof(pid));
    }

    // Receive message from server
    unsigned succ;
    do {
        recv(socketfd, &succ, sizeof(succ), 0);
    } while (succ != 0x12345678);
    return socketfd;
}

void utraceTime(const unsigned long long rawTime, const char* msg) {
    if (socketfd == -2)
        socketfd = startClient();
    if (socketfd == -1)
        return;

    safe_write(socketfd, &magic1, sizeof(magic1));
    safe_write(socketfd, &rawTime, sizeof(rawTime));
    safe_write(socketfd, &magic2, sizeof(magic2));
    unsigned long long len = strlen(msg);
    safe_write(socketfd, &len, sizeof(len));
    safe_write(socketfd, msg, strlen(msg));
    safe_write(socketfd, &magic3, sizeof(magic3));
}

void utrace(const char* msg) {
    time_t t = time(NULL);
    utraceTime(t, msg);
}