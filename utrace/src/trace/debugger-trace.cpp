#include <sys/socket.h>
#include <trace/debugger-trace.hh>
#include <trace/gdb.hh>
#include <unistd.h>
#include <unordered_map>

// implementation of class stream

stream::stream(const std::string& name, const std::string& breakpoint,
               std::shared_ptr<breakpointhandler> handler)
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
    std::unordered_map<std::string, std::string> vals;
    for (auto& var : vars)
        vals.emplace(var, inter.evaluateExpression(var));
    return simpleData(breakpoint, vals);
}

// implementation of class simpleEventHandler

std::pair<eventHandler::action, std::optional<simpleData>>
simpleEventHandler::handle() {
    return {eventHandler::action::EXIT, std::nullopt};
}

// implementation of class debuggerTrace

debuggerTrace::debuggerTrace(bool cmdlineOutput, std::shared_ptr<output> out,
                             const std::vector<stream>& streams)
    : trace(cmdlineOutput, out), streams(streams) {}

void debuggerTrace::work(const pid_t& pid, const int sockfd) {

    auto cleanUpSocket = [&]() {
        unsigned succ = 0xdeadbeef;
        if (send(sockfd, &succ, sizeof(succ), 0) != sizeof(succ))
            throw std::runtime_error("send failed");
        close(sockfd);
    };

    cmdLine(pid);

    if (streams.empty()) {
        cleanUpSocket();
        return;
    }

    std::unique_ptr<debugger> dbg = getDebugger(pid);

    std::unordered_map<std::string, const stream&> breakpoints;
    for (auto& stream : streams) {
        if (dbg->addBreakpoint(stream.getBreakPoint()).empty()) {
            continue;
        }
        breakpoints.insert({stream.getBreakPoint(), stream});
    }

    cleanUpSocket();

    while (1) {
        auto [reason, breakpoint] = dbg->continueExec();
        if (reason == "breakpoint-hit")
            outputWrite(breakpoints.at(breakpoint).handle(*dbg));
        else {
            auto it = eventHandlers.find(reason);
            if (it == eventHandlers.end())
                eventHandlers.insert(
                    {reason, std::make_shared<simpleEventHandler>()});
            auto [action, data] = eventHandlers.at(reason)->handle();
            if (data.has_value())
                outputWrite(data.value());
            if (action == eventHandler::action::EXIT)
                break;
        }
    }

    return;
}
