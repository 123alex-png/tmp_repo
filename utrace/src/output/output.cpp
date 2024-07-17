#include <ctime>
#include <fstream>
#include <output/output.hh>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

data::data() : time(std::time(nullptr)) {}
time_t data::getTime() const { return time; }

void data::setTime(const time_t time) { this->time = time; }

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

stringData::stringData(const std::string& s, const time_t time) : data(), s(s) {
    setTime(time);
}

nlohmann::json stringData::toJson() const {
    using json = nlohmann::json;
    json j;
    j["time"] = getTime();
    j["string"] = s;
    return j;
}

simpleData::simpleData(const std::string breakpoint,
                       const std::unordered_map<std::string, std::string>& vals)
    : data(), breakpoint(breakpoint), vals(vals) {}

nlohmann::json simpleData::toJson() const {
    using json = nlohmann::json;
    json j;
    j["time"] = getTime();
    j["breakpoint"] = this->breakpoint;
    j["values"] = this->vals;
    return j;
}

jsonData::jsonData(const nlohmann::json& j, const time_t time) : data(), j(j) {
    setTime(time);
}

nlohmann::json jsonData::toJson() const {
    using json = nlohmann::json;
    json j;
    j["time"] = getTime();
    j["what"] = this->j;
    return j;
}

output::output(const std::string& outputSock) {
    // UNIX Domain Socket
    sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock == -1)
        throw std::runtime_error("socket failed");

    struct sockaddr_un addr {};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, outputSock.c_str(), sizeof(addr.sun_path) - 1);

    if (connect(sock, (struct sockaddr*)(&addr), sizeof(addr)) == -1)
        throw std::runtime_error("connect failed");
}

void output::write(const data& d) {
    auto s = d.toJson().dump() + "\n";
    if (send(sock, s.c_str(), s.size(), 0) != s.size())
        throw std::runtime_error("send failed");
}

output::~output() { close(sock); }