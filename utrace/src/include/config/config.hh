#pragma once

#include <trace/debugger-trace.hh>

class processFilter {
public:
    virtual bool check(const int&) = 0;
};

class simpleProcessFilter : public processFilter {
public:
    bool check(const int& pid) override;
};

class config {
private:
    enum dbgType {GDB, NONE};
    std::string name;
    //filter
    std::vector<std::string> whiteList;
    std::vector<std::string> args;
    std::unique_ptr<processFilter> filter;

    // trace
    bool cmdLine;
    std::vector<stream> streams;
    dbgType type;

    //output
    std::string outputSock;

    std::shared_ptr<output> getOutput();

public:
    config(const std::string& configFile);
    bool check(const pid_t& pid);
    std::unique_ptr<trace> getTrace();
};
