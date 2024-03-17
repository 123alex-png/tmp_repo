#include <connection/receive.hh>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

// implementation of class receive

receive::receive(trace* trc, int maxClient, int port)
    : connection("receive", {}, new simpleProcessFilter(), trc, maxClient,
                 port) {}

receive::receive(trace* trc, int maxClient, const std::string& path)
    : connection("receive", {}, new simpleProcessFilter(), trc, maxClient,
                 path) {}

void receive::watch() {
    int sockfd = getSockfd();
    while (true) {
        struct sockaddr_in clientAddress;
        socklen_t clientAddressLength = sizeof(clientAddress);
        int client = accept(sockfd, (struct sockaddr*)&clientAddress,
                            &clientAddressLength);
        if (client < 0)
            throw std::runtime_error("accept failed");

        while (true) {

            auto magichk = [&](unsigned long long MAGIC) {
                unsigned long long magic;
                int len = recv(client, &magic, sizeof(magic), 0);
                if (len == 0)
                    return 0;
                if (len < 0)
                    throw std::runtime_error("recv failed");
                if (magic != MAGIC)
                    throw std::runtime_error("invalid magic number");
                return len;
            };

            if (magichk(0x12345678) == 0)
                break;

            unsigned long long time;
            if (recv(client, &time, sizeof(time), 0) < 0)
                throw std::runtime_error("recv failed");

            if (magichk(0x87654321) == 0)
                break;

            long long length;
            if (recv(client, &length, sizeof(length), 0) < 0)
                throw std::runtime_error("recv failed");
            std::stringstream ss;
            char buffer[1024];
            while (length > 0) {
                int n =
                    recv(client, buffer,
                         sizeof(buffer) > length ? length : sizeof(buffer), 0);
                if (n < 0)
                    throw std::runtime_error("recv failed");
                ss.write(buffer, n);
                length -= n;
            }

            if (magichk(0xdeadbeef) == 0)
                break;

            jsonData data(ss.str(), time);
            getOutput()->write(data);
        }
    }
}
