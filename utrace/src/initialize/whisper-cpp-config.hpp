#ifndef WHISPER_CPP_CONFIG_HPP
#define WHISPER_CPP_CONFIG_HPP

#include "common-config.hpp"
#include <fstream>
#include <sstream>

class whisper_cpp_check : public processFilter {
public:
    bool check(const int& pid) override {
        std::ifstream file("/proc/" + std::to_string(pid) + "/comm");
        if (!file.is_open())
            return false;
        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string comm = buffer.str();
        comm.erase(comm.find_last_not_of(" \n\r\t") + 1);
        if (comm.find("stream") != std::string::npos)
            return true;
        if (comm.find("main") != std::string::npos)
            return true;
        return false;
    }
};

class whisper_cpp_breakpoint_handler : public breakpointhandler {
public:
    data* handle(const string& breakpoint, interface& inter) override {
        std::string text = inter.expEval("text");
        return new jsonData(breakpoint, {text});
    }
};

config* whisperCppConfig() {
    vector<string> args{};
    vector<stream> streams;
    streams.push_back(stream("whisper.cpp", "whisper.cpp:5707",
                             new whisper_cpp_breakpoint_handler()));
    streams.push_back(stream("whisper.cpp", "whisper.cpp:5754",
                             new whisper_cpp_breakpoint_handler()));
    return new config("whisper-cpp", args, new whisper_cpp_check(), streams,
                      false);
}

#endif
