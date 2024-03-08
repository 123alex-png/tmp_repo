#include <config/whisperCppConfig.hh>
#include <connect/portSocket.hh>
#include <connect/unixDomainSocket.hh>
#include <fstream>
#include <sstream>

bool whisperCppCheck::check(const pid_t& pid) {
    std::ifstream file("/proc/" + std::to_string(pid) + "/comm");
    if (!file.is_open())
        return false;
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string comm = buffer.str();
    comm.erase(comm.find_last_not_of(" \n\r\t") + 1);
    if (comm.find("stream") != std::string::npos)
        return true;
    if (comm.find("main") != std::string::npos)
        return true;
    return false;
}

whisperCppConfig::whisperCppConfig(output* out, const std::string& portFile)
    : config(out) {
    std::vector<stream> streams;
    streams.push_back(stream("whisper.cpp", "whisper.cpp:5707",
                             new simpleBreakpointHandler({"text"})));
    streams.push_back(stream("whisper.cpp", "whisper.cpp:5754",
                             new simpleBreakpointHandler({"text"})));
    trace* trc = new trace(streams, false, getOutput(), trace::dbgType::GDB);
    setTrace(trc);
    connection* conn = nullptr;
    try {
        int port = std::stoi(portFile);
        conn = new portSocket("whisper.cpp", {}, new whisperCppCheck(), trc, 1,
                              port);
    } catch (const std::invalid_argument& e) {
        conn = new unixDomainSocket("whisper.cpp", {}, new whisperCppCheck(),
                                    trc, 1, portFile);
    }
    setConnect(conn);
}
