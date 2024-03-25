#include <sys/socket.h>
#include <thread>
#include <trace/gdb.hh>
#include <trace/trace.hh>
#include <unistd.h>
#include <unordered_map>

// implementation of class stream

stream::stream(const std::string& name, const std::string& breakpoint,
               breakpointhandler* handler)
    : name(name), breakpoint(breakpoint), handler(handler) {}

std::string stream::getName() const { return name; }

std::string stream::getBreakPoint() const { return breakpoint; }

simpleData stream::handle(debugger& inter) const {
    return handler->handle(breakpoint, inter);
}

// implementation of class simpleBreakpointHandler

simpleBreakpointHandler::simpleBreakpointHandler(
    const std::vector<std::string>& vars)
    : vars(vars) {}

simpleData simpleBreakpointHandler::handle(const std::string& breakpoint,
                                           debugger& inter) {
    std::vector<std::string> vals;
    for (auto& var : vars)
        vals.push_back(inter.evaluateExpression(var));
    return simpleData(breakpoint, vals);
}

// implementation of class simpleEventHandler

std::pair<eventHandler::action, std::optional<simpleData>>
simpleEventHandler::handle() {
    return {eventHandler::action::EXIT, std::nullopt};
}

// implementation of class trace

trace::trace(const std::vector<stream>& streams, bool cmdlineOutput,
             output* out, dbgType dbg)
    : cmdlineOutput(cmdlineOutput), streams(streams), debuggerType(dbg),
      out(out) {}

void trace::work(const pid_t& pid, const int sockfd) {

    auto cleanUpSocket = [&]() {
        unsigned succ = 0xdeadbeef;
        if (send(sockfd, &succ, sizeof(succ), 0) != sizeof(succ))
            throw std::runtime_error("send failed");
        // std::cout << "send " << succ << std::endl;
        close(sockfd);
    };

    if (cmdlineOutput)
        out->write(cmdlineData(pid));

    if (streams.empty()) {
        cleanUpSocket();
        return;
    }

    auto exeRun = [&]() {
        debugger* dbg = nullptr;
        switch (debuggerType) {
        case dbgType::GDB:
            dbg = new gdb(pid);
            break;
        default:
            throw std::runtime_error("Debugger not supported");
        }
        std::unordered_map<std::string, const stream&> breakpoints;
        for (auto& stream : streams) {
            if (dbg->addBreakpoint(stream.getBreakPoint()).empty()) {
                continue;
            }
            breakpoints.insert({stream.getBreakPoint(), stream});
        }

        // std::cout << "Attaching to pid " << data.pid << std::endl;

        cleanUpSocket();

        while (1) {
            auto [reason, breakpoint] = dbg->continueExec();
            if (reason == "breakpoint-hit")
                out->write(breakpoints.at(breakpoint).handle(*dbg));
            else {
                auto it = eventHandlers.find(reason);
                if (it == eventHandlers.end())
                    eventHandlers.insert({reason, new simpleEventHandler()});
                auto [action, data] = eventHandlers.at(reason)->handle();
                if (data.has_value())
                    out->write(data.value());
                if (action == eventHandler::action::EXIT)
                    break;
            }
        }
        delete dbg;
    };

    std::thread t(exeRun);

    t.detach();
    return;
}

output* trace::getOutput() { return out; }
