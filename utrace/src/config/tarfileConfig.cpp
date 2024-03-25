#include <config/tarfileConfig.hh>

tarfileConfig::tarfileConfig(output* out, const std::string& portFile)
    : config(out) {
    std::vector<stream> streams;
    streams.push_back(stream("fopen", "xfuncs_printf.c:137",
                             new simpleBreakpointHandler({"path", "fp"})));
    streams.push_back(stream("xopen3", "xfuncs_printf.c:148",
                             new simpleBreakpointHandler({"pathname", "ret"})));
    streams.push_back(stream("open3_or_warn", "xfuncs_printf.c:166",
                             new simpleBreakpointHandler({"pathname", "ret"})));

    trace* trc = new trace(streams, false, getOutput(), trace::dbgType::GDB);
    setTrace(trc);
    connection* conn = nullptr;
    try {
        int port = std::stoi(portFile);
        conn = new connection("tarfile", {"tar"}, {"tar"},
                              new simpleProcessFilter(), trc, 1, port);
    } catch (const std::invalid_argument& e) {
        conn = new connection("tarfile", {"tar"}, {"tar"},
                              new simpleProcessFilter(), trc, 1, portFile);
    }
    setConnect(conn);
}
