#include <arpa/inet.h>
#include <connect/portSocket.hh>
#include <cstring>
#include <iostream>
#include <sys/socket.h>
#include <unistd.h>

using std::string;

// implementation of portSocket class

portSocket::portSocket(const std::string& name,
                       const std::vector<std::string>& args,
                       processFilter* filter, trace* trc, int maxClient,
                       int port)
    : connection(name, args, filter, trc, maxClient) {

    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        throw std::runtime_error("socket");

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        throw std::runtime_error("bind");
    }

    if (listen(sockfd, maxClient) < 0)
        throw std::runtime_error("listen");

    std::cout << "Listening on port " << port << std::endl;
    setSockfd(sockfd);
}
