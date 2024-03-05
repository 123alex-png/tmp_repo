#pragma once

#include <trace/trace.hh>

class processFilter {
public:
    virtual bool check(const int&) = 0;
};

class simpleProcessFilter : public processFilter {
public:
    bool check(const int& pid) override;
};

class connection {
private:
    std::string name;
    std::vector<std::string> args;
    processFilter* filter;
    trace* trc;
    int sockfd, maxClient;
    bool check(const pid_t& pid);

protected:
    void setSockfd(int sockfd);

public:
    connection(const std::string& name, const std::vector<std::string>& args,
               processFilter* filter, trace* trc, int maxClient);
    void watch();
};
