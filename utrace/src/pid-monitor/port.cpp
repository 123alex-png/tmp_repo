#include <arpa/inet.h>
#include <common.hh>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>
#include <watch.hh>

using std::string;

static int recvSignal = 0, sock = 0;
static string filepath = "/tmp/pid-monitor";
static void on_exit() {
    remove(filepath.c_str());
    close(sock);
    std::ofstream o2("/home/cao/Desktop/log2.txt", std::ios::out);
    if (!o2.is_open()) {
        perror("open");
        return;
    }
    o2 << "port_watch exit" << std::endl;
    o2.close();
}

int portPidWatch(config* cfg, const string& outputFile, const int port,
                 int maxClient) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("socket");
        return -1;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return -1;
    }

    if (listen(sock, maxClient) < 0) {
        perror("listen");
        return -1;
    }

    std::ofstream o(filepath, std::ios::out);
    if (!o.is_open()) {
        perror("open");
        return -1;
    }
    o << port << std::endl;
    o.close();

    atexit(on_exit);
    at_quick_exit(on_exit);
    signal(SIGINT, [](int x) {
        remove(filepath.c_str());
        recvSignal = x;
    });
    signal(SIGTERM, [](int x) {
        remove(filepath.c_str());
        recvSignal = x;
    });
    signal(SIGKILL, [](int x) {
        remove(filepath.c_str());
        recvSignal = x;
    });

    std::cout << "Listening on port " << port << std::endl;
    return sock;
}