#include <boost/asio/execution_context.hpp>
#include <iostream>
#include <regex>
#include <trace/gdb.hh>

namespace bp = boost::process;

void gdb::sendCommand(const std::string& command) {
    // Send command to GDB
    gdbInput << command << std::endl;
    gdbInput.flush();

    // std::cout << "GDB command: " << command << std::endl;
}

std::string gdb::readGDB() {
    // Read GDB output
    std::string output;
    while (output.empty()) {
        std::getline(gdbOutput, output);
    }
    // std::cout << "GDB output: " << output << std::endl;
    return output;
}

std::string gdb::getStopReason(const std::string& output) {
    // Extract the stop reason from GDB output using regex
    std::regex regex("\\*stopped,reason=\"([^\"]*)\"");
    std::smatch match;

    if (std::regex_search(output, match, regex) && match.size() == 2) {
        return match[1].str();
    }

    return "";
}

std::string gdb::getBreakpointNumber(const std::string& output) {
    // Extract the breakpoint number from GDB output using regex
    std::regex regex("\\*stopped,reason=\"breakpoint-hit\",disp=\"[^,]*\","
                     "bkptno=\"([0-9]+)\"");
    std::smatch match;

    if (std::regex_search(output, match, regex) && match.size() == 2) {
        return match[1].str();
    }

    return "";
}

void gdb::handleSignal(std::string& reason, const std::string& output) {
    if (reason != "signal-received")
        return;
    reason = "";
    std::regex regex(
        "\\*stopped,reason=\"signal-received\",signal-name=\"([^\"]*)\"");
    std::smatch match;
    if (std::regex_search(output, match, regex) && match.size() == 2) {
        sendCommand("signal " + match[1].str());
    }
}

gdb::gdb(pid_t pid) : gdbProcess(), gdbInput(), gdbOutput() {
    // Launch GDB process
    std::string command = "gdb --interpreter=mi -p " + std::to_string(pid);
    gdbProcess =
        bp::child(command, bp::std_in<gdbInput, bp::std_out> gdbOutput);
    // Wait for GDB to start
    std::string output;
    while (!output.empty() && output[0] != '~' && output[0] != '@' &&
           output[0] != '&') {
        std::getline(gdbOutput, output);
        // std::cout << "GDB output: " << output << std::endl;
    }
    // std::cout << command << std::endl;
};

gdb::~gdb() {
    // Terminate GDB process
    sendCommand("-gdb-exit");
    gdbProcess.wait();
    gdbInput.close();
    gdbOutput.close();
}

std::string gdb::addBreakpoint(const std::string& breakpoint) {
    sendCommand("-break-insert " + breakpoint);
    std::string ret = "";
    do {
        std::string output = readGDB();
        // if (output.find("error") != string::npos) {
        //     throw std::runtime_error("Error adding breakpoint");
        // }
        std::regex regex("\\^done,bkpt=\\{number=\"([0-9]+)\"");
        std::smatch match;
        if (std::regex_search(output, match, regex) && match.size() == 2) {
            ret = match[1].str();
            breakpoints[ret] = breakpoint;
        }
    } while (ret.empty());
    return ret;
}

std::string gdb::evaluateExpression(const std::string& exp) {
    sendCommand(static_cast<std::string>("-data-evaluate-expression \"") + exp +
                "\"");
    std::string value = "";
    do {
        std::string output = readGDB();
        if (output.find("error") != std::string::npos) {
            throw std::runtime_error("Error reading variable");
        }
        std::regex valueRegex("\\^done,value=\"(.*)\"");
        std::smatch valueMatch;

        if (std::regex_search(output, valueMatch, valueRegex) &&
            valueMatch.size() == 2) {
            value = valueMatch[1].str();
        }
    } while (value.empty());
    return value;
}

std::pair<std::string, std::string> gdb::continueExec() {
    // Send continue command to GDB
    sendCommand("-exec-continue");
    std::string reason = "";
    do {
        // Read GDB output
        std::string output = readGDB();

        // Analyze the reason for stopping
        reason = getStopReason(output);

        // Handle signals
        handleSignal(reason, output);

        // If GDB hit a breakpoint, analyze the breakpoint number
        if (reason == "breakpoint-hit") {
            std::string breakpointNumber = getBreakpointNumber(output);
            return {reason, breakpoints[breakpointNumber]};
        }
    } while (reason.empty());
    return {reason, ""};
}

// gdbTrace implementation
std::unique_ptr<debugger> gdbTrace::getDebugger(pid_t pid) {
    return std::make_unique<gdb>(pid);
}

gdbTrace::gdbTrace(bool cmdlineOutput, std::shared_ptr<output> out,
                   const std::vector<stream>& streams)
    : debuggerTrace(cmdlineOutput, out, streams) {}
