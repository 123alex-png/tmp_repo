#include <config/config.hh>
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
#include <thread>
#include <unistd.h>

using std::string;

// implementation of class connection

std::string connection::getConfig(const pid_t& pid) {
    std::string cwdfile = "/proc/" + std::to_string(pid) + "/cwd";
    static char buffer[4096];

    // Read the symbolic link pointing to the current working directory
    ssize_t len = readlink(cwdfile.c_str(), buffer, sizeof(buffer) - 1);

    if (len == -1)
        throw std::runtime_error("readlink");

    buffer[len] = '\0';
    std::string path = std::string(buffer);

    // search the "utrace-config.json" file in the current working directory
    while (true) {
        std::string configPath = path + "/utrace-config.json";
        if (std::filesystem::exists(configPath))
            return configPath;
        if (path == "/")
            break;
        path = std::filesystem::path(path).parent_path();
    }
    // throw std::runtime_error("utrace-config.json not found");
    return "";
}

connection::connection(int maxClient, const std::string& path)
    : maxClient(maxClient) {
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

        auto work = [&](int client) {
            struct {
                int pid;
                unsigned magicnumber;
            } data;
            if (recv(client, &data, sizeof(data), 0) < 0)
                throw std::runtime_error("recv failed");
            if (data.magicnumber != 0xdeadbeef)
                throw std::runtime_error("invalid magic number");

            auto exit_sliently = [&]() {
                unsigned fail = 0xdeadbeef;
                if (send(client, &fail, sizeof(fail), 0) < 0)
                    throw std::runtime_error("send failed");
                close(client);
            };

            std::string cfg_path = getConfig(data.pid);
            if (cfg_path.empty()) {
                exit_sliently();
                return;
            }

            config cfg = config(cfg_path);

            if (!cfg.check(data.pid)) {
                exit_sliently();
                return;
            }

            std::cout << "Tracing process " << data.pid << std::endl;
            cfg.getTrace()->work(data.pid, client);
        };

        std::thread(work, client).detach();
    }
}
