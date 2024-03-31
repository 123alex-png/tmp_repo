#include <config/config.hh>
#include <fstream>
#include <trace/gdb.hh>
#include <trace/none-debugger-trace.hh>

// implementation of class simpleProcessFilter

bool simpleProcessFilter::check(const int& pid) { return true; }

// implementation of class config

std::shared_ptr<output> config::getOutput() {
    return std::make_shared<output>(outputFile);
}

config::config(const std::string& configFile) {
    std::ifstream file(configFile);
    if (!file.is_open())
        throw std::invalid_argument("Invalid config file");
    auto extention = std::filesystem::path(configFile).extension();
    if (extention == ".json") {
        nlohmann::json j;
        file >> j;
        if (j.contains("name"))
            name = j["name"];
        else
            throw std::invalid_argument("No name in config file");

        // filter
        if (j.contains("whiteList"))
            whiteList = j["whiteList"].get<std::vector<std::string>>();
        else
            whiteList = {};
        if (j.contains("args"))
            args = j["args"].get<std::vector<std::string>>();
        else
            args = {};
        filter = std::make_unique<simpleProcessFilter>();

        // trace
        cmdLine = false;
        if (j.contains("cmdLine"))
            cmdLine = j["cmdLine"];
        type = dbgType::NONE;
        if (j.contains("trace")) {
            std::string tr = j["trace"];
            if (tr == "GDB" || tr == "gdb")
                type = dbgType::GDB;
            else if (tr == "NONE" || tr == "none")
                type = dbgType::NONE;
            else
                throw std::invalid_argument("Invalid trace type");
        }
        if (j.contains("streams")) {
            for (auto& s : j["streams"]) {
                std::string name = s["name"];
                std::string breakpoint = s["breakpoint"];
                std::vector<std::string> vars = {};
                if (s.contains("vars"))
                    vars = s["vars"].get<std::vector<std::string>>();

                streams.push_back(
                    stream(name, breakpoint,
                           std::make_shared<simpleBreakpointHandler>(vars)));
            }
        }

        // output
        if (j.contains("output"))
            outputFile = j["output"];
        else
            throw std::invalid_argument("No output in config file");

    } else
        throw std::invalid_argument("Invalid config file");
}

bool config::check(const pid_t& pid) {
    if (!whiteList.empty()) {
        std::ifstream comm("/proc/" + std::to_string(pid) + "/cmdline");
        if (!comm.is_open())
            return false;
        std::stringstream buffer;
        buffer << comm.rdbuf();
        std::string str = buffer.str();
        bool flag = false;
        for (auto& s : whiteList)
            if (str.find(s) != std::string::npos) {
                flag = true;
                break;
            }
        if (!flag)
            return false;
    }

    if (!args.empty()) {
        std::vector<std::string> arguments;
        std::ifstream cmdline("/proc/" + std::to_string(pid) + "/cmdline");
        if (!cmdline.is_open())
            return false;
        std::stringstream buffer;
        buffer << cmdline.rdbuf();

        std::istringstream iss(buffer.str());
        std::string argument;

        while (getline(iss, argument, '\0')) {
            arguments.push_back(argument);
        }
        for (const auto& arg : args) {
            bool flag = false;
            for (const auto& argument : arguments) {
                if (argument.find(arg) != std::string::npos) {
                    flag = true;
                    break;
                }
            }
            if (!flag)
                return false;
        }
    }
    if (!filter->check(pid))
        return false;

    return true;
}

std::unique_ptr<trace> config::getTrace() {
    switch (type) {
    case dbgType::GDB:
        return std::make_unique<gdbTrace>(cmdLine, getOutput(), streams);
    case dbgType::NONE:
        return std::make_unique<nonDebuggerTrace>(cmdLine, getOutput());
    }
    throw std::invalid_argument("Invalid trace type");
}
