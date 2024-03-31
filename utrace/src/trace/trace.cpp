#include <trace/trace.hh>

trace::trace(bool cmdlineOutput, std::shared_ptr<output> out)
    : cmdlineOutput(cmdlineOutput), out(out) {}

void trace::outputWrite(const data& data) { out->write(data); }

void trace::cmdLine(const pid_t& pid) {
    if (cmdlineOutput)
        out->write(cmdlineData(pid));
}