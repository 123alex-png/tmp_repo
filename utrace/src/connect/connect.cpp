#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <netinet/in.h>
#include <output.hh>
#include <signal.h>
#include <sstream>
#include <sys/socket.h>
#include <trace.hh>
#include <unistd.h>
#include <watch.hh>

using std::string;
bool check(int pid, config* cfg) {
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
    for (const auto& arg : cfg->getArguments()) {
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

    if (!cfg->chk(pid))
        return false;

    return true;
}

int portWatch(config* cfg, const string& outputFile, const string& portFile,
              int maxClient) {
    int sock = 0;
    try {
        int port = std::stoi(portFile);
        sock = portPidWatch(cfg, outputFile, port, maxClient);
    } catch (std::invalid_argument& e) {
        // std::cout << "port is not a number" << std::endl;
        sock = unixDomainSocketWatch(cfg, outputFile, portFile, maxClient);
    } catch (std::out_of_range& e) {
        std::cout << "port is out of range" << std::endl;
        return -1;
    }
    if (sock < 0)
        return -1;

    while (true) {
        struct sockaddr_in clientAddress;
        socklen_t clientAddressLength = sizeof(clientAddress);
        int client = accept(sock, (struct sockaddr*)&clientAddress,
                            &clientAddressLength);
        if (client < 0) {
            perror("accept");
            return -1;
        }

        struct {
            int pid;
            unsigned magicnumber;
        } data;
        if (recv(client, &data, sizeof(data), 0) < 0) {
            perror("recv");
            return -1;
        }
        if (data.magicnumber != 0xdeadbeef)
            continue;

        if (!check(data.pid, cfg)) {
            unsigned fail = 0xdeadbeef;
            if (send(client, &fail, sizeof(fail), 0) < 0) {
                perror("send");
                return -1;
            }
            close(client);
            continue;
        }

        if (cfg->getCmdline())
            cmdline(data.pid, outputFile);

        socketClose sync(client, true);

        if (!cfg->getStreams().empty())
            runMonitor(cfg->getStreams(), data.pid, std::vector<string>(),
                       sync);

        // std::cout << "Attaching to pid " << data.pid << std::endl;

        unsigned succ = 0xdeadbeef;
        if (send(client, &succ, sizeof(succ), 0) < 0) {
            perror("send");
            return -1;
        }
        // std::cout << "send " << succ << std::endl;
        close(client);
        sync.notify();
    }
    return 0;
}
