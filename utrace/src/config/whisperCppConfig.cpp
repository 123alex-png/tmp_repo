#include <config/whisperCppConfig.hh>
#include <fstream>
#include <sstream>

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
        conn = new connection("whisper.cpp", {"stream", "main"}, {},
                              new simpleProcessFilter(), trc, 1, port);
    } catch (const std::invalid_argument& e) {
        conn = new connection("whisper.cpp", {"stream", "main"}, {},
                              new simpleProcessFilter(), trc, 1, portFile);
    }
    setConnect(conn);
}
