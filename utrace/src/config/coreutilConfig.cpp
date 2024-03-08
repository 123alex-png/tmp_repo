#include <config/coreutilConfig.hh>
#include <connect/portSocket.hh>
#include <connect/unixDomainSocket.hh>
#include <fstream>
#include <sstream>
#include <string>

bool coreUtilFilter::check(const pid_t& pid) {
    std::ifstream file("/proc/" + std::to_string(pid) + "/comm");
    if (!file.is_open())
        return false;
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string comm = buffer.str();
    comm.erase(comm.find_last_not_of(" \n\r\t") + 1);
    for (const auto& coreutil : coreutilList) {
        if (comm.find(coreutil) != std::string::npos)
            return true;
    }
    return false;
}

coreutilConfig::coreutilConfig(output* out, const std::string& portFile)
    : config(out) {
    trace* trc = new trace({}, true, getOutput(), trace::dbgType::GDB);
    setTrace(trc);
    connection* conn = nullptr;
    try {
        int port = std::stoi(portFile);
        conn =
            new portSocket("coreUtil", {}, new coreUtilFilter(), trc, 1, port);
    } catch (const std::invalid_argument& e) {
        conn = new unixDomainSocket("coreUtil", {}, new coreUtilFilter(), trc,
                                    1, portFile);
    }
    setConnect(conn);
}