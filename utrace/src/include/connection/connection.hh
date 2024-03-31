#pragma once

#include <trace/trace.hh>

class connection {
private:
    int sockfd, maxClient;
    std::string getConfig(const pid_t& pid);

public:
    connection(int maxClient, const std::string& path);
    void watch();
};
