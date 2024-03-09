#pragma once

#include <boost/process.hpp>
#include <boost/process/io.hpp>
#include <trace/trace.hh>

class gdb : public debugger {
private:
    boost::process::child gdbProcess;
    boost::process::opstream gdbInput;
    boost::process::ipstream gdbOutput;
    std::unordered_map<std::string, std::string> breakpoints;

    void sendCommand(const std::string& command);

    std::string readGDB();

    std::string getStopReason(const std::string& output);

    std::string getBreakpointNumber(const std::string& output);

    void handleSignal(std::string &reason, const std::string &output);

public:
    gdb(pid_t pid);

    ~gdb();

    std::string addBreakpoint(const std::string& breakpoint) override;

    std::string evaluateExpression(const std::string& exp) override;

    std::pair<std::string, std::string> continueExec() override;

};
