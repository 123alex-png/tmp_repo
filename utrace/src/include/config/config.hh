#pragma once

#include <connect/connect.hh>
#include <output/output.hh>
#include <trace/trace.hh>

class config {
private:
    connection* conn;
    trace* trc;
    output* out;

public:
    config(output* out);
    void setConnect(connection* conn);
    connection* getConnect();
    void setTrace(trace* trc);
    trace* getTrace();
    output* getOutput();
};

class configInit {
private:
    config* conf;

public:
    configInit(const std::string& configFile, const std::string& portFile,
               output* out);
    config* getConfig();
};
