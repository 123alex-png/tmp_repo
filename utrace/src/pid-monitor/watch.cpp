#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <signal.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>
#include <watch.hh>

using std::string;
bool check(string filename, config* cfg) {
    if (!cfg)
        return true;

    static char buf[1024];

    FILE* fp = fopen(("/proc/" + filename + "/comm").c_str(), "r");
    if (fp == NULL)
        return false;

    fgets(buf, 1024, fp);
    fclose(fp);
    string comm = buf;
    comm.erase(comm.find_last_not_of(" \n\r\t") + 1);
    if (comm != cfg->getName())
        return false;

    std::vector<std::string> arguments;
    std::ifstream file("/proc/" + filename + "/cmdline");
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

    if (!cfg->chk())
        return false;

    return true;
}

void cmdlineGen(const string& pid, const string& outputFile) {

    //    std::cout << std::endl<< "Generating cmdline for " << pid << " at "<<
    //    std::ctime(new time_t);
    std::ifstream file("/proc/" + pid + "/cmdline");
    std::vector<string> arguments;
    if (file.is_open()) {
        std::stringstream buffer;
        buffer << file.rdbuf();
        // output all the command line
        std::string cmdline = buffer.str();
        std::istringstream iss(cmdline);
        std::string argument;
        while (getline(iss, argument, '\0')) {
            arguments.push_back(argument);
        }
    }
    using json = nlohmann::json;
    json j;
    j["cmdline"] = arguments;
    j["time"] = std::time(nullptr);
    if (!outputFile.empty()) {
        std::ofstream o(outputFile, std::ios::app);
        o << j.dump() << std::endl;
    } else {
        std::cout << j.dump() << std::endl;
    }
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

        if (!check(std::to_string(data.pid), cfg)) {
            unsigned fail = 0xdeadbeef;
            if (send(client, &fail, sizeof(fail), 0) < 0) {
                perror("send");
                return -1;
            }
            close(client);
            continue;
        }

        cmdlineGen(std::to_string(data.pid), outputFile);

        socketClose sync(client, true);

        if (cfg)
            runMonitor(cfg->getStreams(), std::to_string(data.pid),
                       std::vector<string>(), sync);

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
