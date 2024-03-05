#include <connect/unixDomainSocket.hh>
#include <fcntl.h>
#include <iostream>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

using std::string;

// implementation of unixDomainSocket class

unixDomainSocket::unixDomainSocket(const std::string& name,
                                   const std::vector<std::string>& args,
                                   processFilter* filter, trace* trc,
                                   int maxClient, const std::string& path)
    : connection(name, args, filter, trc, maxClient) {

    // Create a UNIX domain socket
    int sockfd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sockfd == -1)
        throw std::runtime_error("socket");

    // Create the directory if it does not exist
    std::string directoryPath = path.substr(0, path.find_last_of('/'));
    int status =
        mkdir(directoryPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (status != 0 && errno != EEXIST)
        throw std::runtime_error("mkdir");

    // Specify the socket address
    struct sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);

    // Bind the socket to the address
    if (bind(sockfd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) ==
        -1) {
        perror("bind");
        throw std::runtime_error("bind");
    }

    // Set appropriate permissions for the socket file
    if (chmod(addr.sun_path, 0666) == -1)
        throw std::runtime_error("chmod");

    // Listen for incoming connections
    if (listen(sockfd, maxClient) == -1)
        throw std::runtime_error("listen");

    setSockfd(sockfd);
}