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

static void cleanup() {
    if (socketfd >= 0)
        close(socketfd);
    socketfd = -2;
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

static void safe_read(int fd, void* buf, size_t count) {
    while (count > 0) {
        ssize_t read_bytes = read(fd, buf, count);
        if (read_bytes < 0) {
            if (errno == EINTR)
                continue;
            perror("read");
            return;
        }
        count -= read_bytes;
        buf = (char*)buf + read_bytes;
    }
}

static int start_client() {
    int client_socket = unixDomainSocket("/tmp/utrace.sock");
    if (client_socket == -1)
        return -1;
    // Send message to server
    struct {
        int pid;
        unsigned magicnumber;
    } data = {getpid(), 0xdeadbeef};
    safe_write(client_socket, &data, sizeof(data));

    // Receive message from server
    unsigned succ;
    do {
        safe_read(client_socket, &succ, sizeof(succ));
    } while (succ != 0xdeadbeef);

    atexit(cleanup);
    return client_socket;
}

void utraceTime(const unsigned long long rawTime, const char* msg) {
    if (socketfd == -2)
        socketfd = start_client();
    if (socketfd == -1)
        return;

    static const unsigned long long magic1 = 0x12345678, magic2 = 0x87654321,
                                    magic3 = 0xdeadbeef;

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