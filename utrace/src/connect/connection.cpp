#include <connection/connection.hh>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

using std::string;

// implementation of class simpleProcessFilter

bool simpleProcessFilter::check(const int& pid) { return true; }

// implementation of class connection

bool connection::check(const pid_t& pid) {
    std::ifstream comm("/proc/" + std::to_string(pid) + "/cmdline");
    if (comm.is_open()) {
        std::stringstream buffer;
        buffer << comm.rdbuf();
        std::string comm = buffer.str();
        for (auto& s : whiteList)
            if (comm.find(s) != std::string::npos)
                return true;
        return false;
    } else
        return false;

    std::vector<std::string> arguments;
    std::ifstream cmdline("/proc/" + std::to_string(pid) + "/cmdline");
    if (cmdline.is_open()) {
        std::stringstream buffer;
        buffer << cmdline.rdbuf();

        std::string cmdline = buffer.str();
        std::istringstream iss(cmdline);
        std::string argument;

        while (getline(iss, argument, '\0')) {
            arguments.push_back(argument);
        }
    } else
        return false;
    for (const auto& arg : args) {
        bool flag = false;
        for (const auto& argument : arguments) {
            if (argument.find(arg) != std::string::npos) {
                flag = true;
                break;
            }
        }
        if (!flag)
            return false;
    }

    if (!filter->check(pid))
        return false;

    return true;
}

int connection::getSockfd() { return sockfd; }
output* connection::getOutput() { return trc->getOutput(); }

// portSocket
connection::connection(const std::string& name,
                       const std::vector<std::string>& whiteList,
                       const std::vector<std::string>& args,
                       processFilter* filter, trace* trc, int maxClient,
                       int port)
    : name(name), whiteList(whiteList), args(args), filter(filter), trc(trc),
      maxClient(maxClient) {
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
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
}

// unixDomainSocket
connection::connection(const std::string& name,
                       const std::vector<std::string>& whiteList,
                       const std::vector<std::string>& args,
                       processFilter* filter, trace* trc, int maxClient,
                       const std::string& path)
    : name(name), whiteList(whiteList), args(args), filter(filter), trc(trc),
      maxClient(maxClient) {
    // Create a UNIX domain socket
    sockfd = socket(AF_UNIX, SOCK_STREAM, 0);
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
}

void connection::watch() {
    while (true) {
        struct sockaddr_in clientAddress;
        socklen_t clientAddressLength = sizeof(clientAddress);
        int client = accept(sockfd, (struct sockaddr*)&clientAddress,
                            &clientAddressLength);
        if (client < 0)
            throw std::runtime_error("accept failed");

        struct {
            int pid;
            unsigned magicnumber;
        } data;
        if (recv(client, &data, sizeof(data), 0) < 0)
            throw std::runtime_error("recv failed");
        if (data.magicnumber != 0xdeadbeef)
            throw std::runtime_error("invalid magic number");

        if (!check(data.pid)) {
            unsigned fail = 0xdeadbeef;
            if (send(client, &fail, sizeof(fail), 0) < 0)
                throw std::runtime_error("send failed");
            close(client);
            continue;
        }

        trc->work(data.pid, client);
    }
}
