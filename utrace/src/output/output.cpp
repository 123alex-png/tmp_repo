#include <ctime>
#include <fstream>
#include <output/output.hh>

data::data() : time(std::time(nullptr)) {}
time_t data::getTime() const { return time; }

cmdlineData::cmdlineData(const pid_t pid) {
    std::ifstream file("/proc/" + std::to_string(pid) + "/cmdline");
    std::vector<std::string> arguments;
    if (file.is_open()) {
        std::stringstream buffer;
        buffer << file.rdbuf();
        // output all the command line
        std::string cmdline = buffer.str();
        std::istringstream iss(cmdline);
        std::string argument;
        while (getline(iss, argument, '\0')) {
            arguments.push_back(argument);
        }
    }
    this->args = arguments;
}

nlohmann::json cmdlineData::toJson() const {
    using json = nlohmann::json;
    json j;
    j["time"] = getTime();
    j["cmdline"] = this->args;
    return j;
}

simpleData::simpleData(const std::string breakpoint,
                       const std::vector<std::string>& vals)
    : data(), breakpoint(breakpoint), vals(vals) {}

nlohmann::json simpleData::toJson() const {
    using json = nlohmann::json;
    json j;
    j["time"] = getTime();
    j["breakpoint"] = this->breakpoint;
    j["values"] = this->vals;
    return j;
}

output::output(const std::string& outputFile) {
    file = std::ofstream(outputFile, std::ios::app);
    outputBuffer = file.rdbuf();
}

output::output() : file() { outputBuffer = std::cout.rdbuf(); }

void output::write(const data& d) {
    std::ostream output(outputBuffer);
    output << d.toJson().dump() << std::endl;
}

void output::close() {
    if (outputBuffer != std::cout.rdbuf())
        file.close();
}