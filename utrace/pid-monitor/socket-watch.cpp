#include <arpa/inet.h>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>
#include <watch.hpp>
#include <nlohmann/json.hpp>
#include <signal.h>

using std::string;
bool check(string filename, config* cfg) {
    if(!cfg)
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

void cmdline_gen(const string& pid, const string& outputFile) {

//    std::cout << std::endl<< "Generating cmdline for " << pid << " at "<< std::ctime(new time_t);
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
    if(!outputFile.empty()) {
        std::ofstream o(outputFile, std::ios::app);
        o << j.dump() << std::endl;
    } else {
        std::cout << j.dump() << std::endl;
    }
}

static int recv_signal = 0;

int port_watch(config* cfg, const string &outputFile,int port, int max_client) {
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

    if (listen(sock, max_client) < 0) {
        perror("listen");
        return -1;
    }

    string filepath = "/tmp/pid-monitor";
    std::ofstream o(filepath, std::ios::out);
    if (!o.is_open()) {
        perror("open");
        return -1;
    }
    o << port << std::endl;
    o.close();
    signal(SIGINT, [](int x) { recv_signal = x; });
    signal(SIGTERM, [](int x) { recv_signal = x; });
    signal(SIGKILL, [](int x) { recv_signal = x; });

    std::cout << "Listening on port " << port << std::endl;

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

        cmdline_gen(std::to_string(data.pid), outputFile);

        socketClose sync(client, true);

        if(cfg) 
            run_monitor(cfg->getStreams(), std::to_string(data.pid),
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
        if(recv_signal) break;
    }
    remove(filepath.c_str());
    close(sock);
    std::ofstream o2("/home/cao/Desktop/log2.txt", std::ios::out);
    if(o2.is_open()){
        o2 << "port_watch exit" << std::endl;
        o2.close();
    }   
    return 0;
}
