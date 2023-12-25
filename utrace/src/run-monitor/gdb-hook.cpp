#ifndef GDB_HOOK_HPP
#define GDB_HOOK_HPP

#include <boost/asio/execution/context.hpp>
#include <boost/process.hpp>
#include <boost/process/io.hpp>
#include <common.hh>
#include <iostream>
#include <regex>
#include <unordered_map>

namespace bp = boost::process;
using std::string;

class gdbInterface : public interface {
private:
    bp::child gdbProcess;
    bp::opstream gdbInput;
    bp::ipstream gdbOutput;
    std::unordered_map<string, string> breakpoints;
    string currentBreakpoint;
    enum class state { running, stopped } state;

    void sendCommand(const string& command) {
        // Send command to GDB
        gdbInput << command << std::endl;
        gdbInput.flush();

        // std::cout << "GDB command: " << command << std::endl;
    }

    string readGDB() {
        // Read GDB output
        string output;
        while (output.empty()) {
            std::getline(gdbOutput, output);
        }
        // std::cout << "GDB output: " << output << std::endl;
        return output;
    }

    std::string getStopReason(const std::string& output) {
        // Extract the stop reason from GDB output using regex
        std::regex regex("\\*stopped,reason=\"([^\"]*)\"");
        std::smatch match;

        if (std::regex_search(output, match, regex) && match.size() == 2) {
            return match[1].str();
        }

        return "";
    }

    std::string getBreakpointNumber(const std::string& output) {
        // Extract the breakpoint number from GDB output using regex
        std::regex regex("\\*stopped,reason=\"breakpoint-hit\",disp=\"[^,]*\","
                         "bkptno=\"([0-9]+)\"");
        std::smatch match;

        if (std::regex_search(output, match, regex) && match.size() == 2) {
            return match[1].str();
        }

        return "";
    }

public:
    gdbInterface(const std::string& pid, const std::vector<string>& argv)
        : gdbProcess(), gdbInput(), gdbOutput() {
        // Launch GDB process
        string command = "gdb --interpreter=mi -p " + pid;
        state = state::running;
        if (pid.empty()) {
            command = "gdb --interpreter=mi " + argv[0] + " --args ";
            for (const auto& arg : argv)
                command += " " + arg;
            state = state::stopped;
        }
        gdbProcess =
            bp::child(command, bp::std_in<gdbInput, bp::std_out> gdbOutput);
        // Wait for GDB to start
        string output;
        while (!output.empty()) {
            std::getline(gdbOutput, output);
            // std::cout << "GDB output: " << output << std::endl;
        }
        // std::cout << command << std::endl;
    }

    ~gdbInterface() {
        // Terminate GDB process
        sendCommand("-gdb-exit");
        gdbProcess.terminate();
        gdbProcess.wait();
        gdbInput.close();
        gdbOutput.close();
    }

    string addBreakpoint(const string& breakpoint) override {
        sendCommand("-break-insert " + breakpoint);
        string ret = "";
        do {
            string output = readGDB();
            if (output.find("error") != string::npos) {
                throw std::runtime_error("Error adding breakpoint");
            }
            std::regex regex("\\^done,bkpt=\\{number=\"([0-9]+)\"");
            std::smatch match;
            if (std::regex_search(output, match, regex) && match.size() == 2) {
                ret = match[1].str();
                breakpoints[ret] = breakpoint;
            }
        } while (ret.empty());
        return ret;
    }

    string expEval(const string& exp) override {
        sendCommand((string) "-data-evaluate-expression \"" + exp + "\"");
        string value = "";
        do {
            string output = readGDB();
            if (output.find("error") != string::npos) {
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

    bool continueExec() override {
        // Send continue command to GDB
        if (state == state::stopped)
            sendCommand("-exec-run");
        else
            sendCommand("-exec-continue");
        state = state::running;
        string reason = "";
        do {
            // Read GDB output
            string output = readGDB();

            // Analyze the reason for stopping
            reason = getStopReason(output);

            // If GDB hit a breakpoint, analyze the breakpoint number
            if (reason == "breakpoint-hit") {
                string breakpointNumber = getBreakpointNumber(output);
                currentBreakpoint = breakpointNumber;
                return true;
            }
        } while (reason.empty());
        return false;
    }

    string currentBreakpointHit() const override {
        return !currentBreakpoint.empty() ? breakpoints.at(currentBreakpoint)
                                          : "";
    }
};

interface* gdbInterfacebuild(const string& pid,
                             const std::vector<string>& argv) {
    return new gdbInterface(pid, argv);
}
#endif
