#include <connect/connect.hh>
#include <fstream>
#include <iostream>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

using std::string;

// implementation of class simpleProcessFilter

bool simpleProcessFilter::check(const int& pid) { return true; }

// implementation of class connection

bool connection::check(const pid_t& pid) {
    std::vector<std::string> arguments;
    std::ifstream file("/proc/" + std::to_string(pid) + "/cmdline");
    if (file.is_open()) {
        std::stringstream buffer;
        buffer << file.rdbuf();

        std::string cmdline = buffer.str();
        std::istringstream iss(cmdline);
        std::string argument;

        while (getline(iss, argument, '\0')) {
            arguments.push_back(argument);
        }
    }
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

void connection::setSockfd(int sockfd) { this->sockfd = sockfd; }

connection::connection(const std::string& name,
                       const std::vector<std::string>& args,
                       processFilter* filter, trace* trc, int maxClient)
    : name(name), args(args), filter(filter), trc(trc), maxClient(maxClient) {}

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

        trc->work(data.pid);

        // std::cout << "Attaching to pid " << data.pid << std::endl;

        unsigned succ = 0xdeadbeef;
        if (send(client, &succ, sizeof(succ), 0) < 0)
            throw std::runtime_error("send failed");
        // std::cout << "send " << succ << std::endl;
        close(client);
    }
}
