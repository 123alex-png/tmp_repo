#include <common.hh>
#include <fcntl.h>
#include <iostream>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

using std::string;
int unixDomainSocketWatch(config* cfg, const string& outputFile,
                          const string& file, int maxClient) {
    // Create a UNIX domain socket
    int sockfd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sockfd == -1) {
        std::cerr << "Failed to create socket." << std::endl;
        return -1;
    }

    // Create the directory if it does not exist
    std::string directoryPath = file.substr(0, file.find_last_of('/'));
    int status =
        mkdir(directoryPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (status != 0 && errno != EEXIST) {
        std::cout << "Failed to create directory!" << std::endl;
        return -1;
    }

    // Specify the socket address
    struct sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, file.c_str(), sizeof(addr.sun_path) - 1);

    // Bind the socket to the address
    if (bind(sockfd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) ==
        -1) {
        std::cerr << "Failed to bind socket to address." << std::endl;
        close(sockfd);
        return -1;
    }

    // Set appropriate permissions for the socket file
    if (chmod(addr.sun_path, 0666) == -1) {
        std::cerr << "Failed to set socket file permissions." << std::endl;
        close(sockfd);
        return -1;
    }

    // Listen for incoming connections
    if (listen(sockfd, 5) == -1) {
        std::cerr << "Failed to listen for connections." << std::endl;
        close(sockfd);
        return -1;
    }

    return sockfd;
}