#pragma once

#include <boost/process.hpp>
#include <boost/process/io.hpp>
#include <trace/trace.hh>
#include <unordered_map>

class gdb : public debugger {
private:
    boost::process::child gdbProcess;
    boost::process::opstream gdbInput;
    boost::process::ipstream gdbOutput;
    std::unordered_map<std::string, std::string> breakpoints;
    std::string currentBreakpoint;
    enum class state { running, stopped } state;

    void sendCommand(const std::string& command);

    std::string readGDB();

    std::string getStopReason(const std::string& output);

    std::string getBreakpointNumber(const std::string& output);

public:
    gdb();

    ~gdb();

    void start(const pid_t& pid) override;

    std::string addBreakpoint(const std::string& breakpoint) override;

    std::string expEval(const std::string& exp) override;

    bool continueExec() override;

    std::string currentBreakpointHit() const override;
};
