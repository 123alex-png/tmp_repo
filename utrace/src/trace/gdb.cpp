#include <boost/asio/execution/context.hpp>
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

gdb::gdb() : gdbProcess(), gdbInput(), gdbOutput(), state(state::stopped){};

gdb::~gdb() {
    // Terminate GDB process
    sendCommand("-gdb-exit");
    gdbProcess.terminate();
    gdbProcess.wait();
    gdbInput.close();
    gdbOutput.close();
}

void gdb::start(const pid_t& pid) {
    // Launch GDB process
    std::string command = "gdb --interpreter=mi -p " + std::to_string(pid);
    state = state::running;
    gdbProcess =
        bp::child(command, bp::std_in<gdbInput, bp::std_out> gdbOutput);
    // Wait for GDB to start
    std::string output;
    while (!output.empty()) {
        std::getline(gdbOutput, output);
        // std::cout << "GDB output: " << output << std::endl;
    }
    // std::cout << command << std::endl;
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

std::string gdb::expEval(const std::string& exp) {
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

bool gdb::continueExec() {
    // Send continue command to GDB
    if (state == state::stopped)
        sendCommand("-exec-run");
    else
        sendCommand("-exec-continue");
    state = state::running;
    std::string reason = "";
    do {
        // Read GDB output
        std::string output = readGDB();

        // Analyze the reason for stopping
        reason = getStopReason(output);

        // If GDB hit a breakpoint, analyze the breakpoint number
        if (reason == "breakpoint-hit") {
            std::string breakpointNumber = getBreakpointNumber(output);
            currentBreakpoint = breakpointNumber;
            return true;
        }
    } while (reason.empty());
    return false;
}

std::string gdb::currentBreakpointHit() const {
    return !currentBreakpoint.empty() ? breakpoints.at(currentBreakpoint) : "";
}
