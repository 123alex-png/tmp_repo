#ifndef GDB_HOOK_HPP
#define GDB_HOOK_HPP

#include <boost/asio/execution/context.hpp>
#include <boost/process.hpp>
#include <boost/process/io.hpp>
#include <common.hpp>
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
	gdbInterface(const std::string& pid)
		: gdbProcess(), gdbInput(), gdbOutput() {
		// Launch GDB process
		string command = "gdb --interpreter=mi -p " + pid;
		gdbProcess =
			bp::child(command, bp::std_in<gdbInput, bp::std_out> gdbOutput);
		std::cout << "GDB process launched: " << command << std::endl;
		// Wait for GDB to start
		string output;
		while (!output.empty()) {
			std::getline(gdbOutput, output);
			// std::cout << "GDB output: " << output << std::endl;
		}
	}

	~gdbInterface() {
		// Terminate GDB process
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

	std::pair<string, string> readVar(const string& var) override {
		sendCommand("-var-create myvar * " + var);
		sendCommand("-var-evaluate-expression myvar");
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
		sendCommand("-var-info-type myvar");
		string type = "";
		do {
			string output = readGDB();
			if (output.find("error") != string::npos) {
				throw std::runtime_error("Error reading variable");
			}
			std::regex typeRegex("\\^done,type=\"(.*)\"");
			std::smatch typeMatch;

			if (std::regex_search(output, typeMatch, typeRegex) &&
				typeMatch.size() == 2) {
				type = typeMatch[1].str();
			}
		} while (type.empty());
		sendCommand("-var-delete myvar");
		return {type, value};
	}

	bool continueExec() override {
		// Send continue command to GDB
		sendCommand("-exec-continue");

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

interface* gdbInterfacebuild(const string& pid) {
	return new gdbInterface(pid);
}
#endif
