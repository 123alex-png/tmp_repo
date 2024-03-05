#include <condition_variable>
#include <mutex>
#include <thread>
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
        vals.push_back(inter.expEval(var));
    return simpleData(breakpoint, vals);
}

// implementation of class trace

trace::trace(const std::vector<stream>& streams, bool cmdlineOutput,
             output* out, debugger* dbg)
    : cmdlineOutput(cmdlineOutput), streams(streams), dbg(dbg), out(out) {}

void trace::work(const pid_t& pid) {
    if (cmdlineOutput)
        out->write(cmdlineData(pid));

    if (streams.empty())
        return;

    std::mutex mutex;
    std::condition_variable cv;
    bool ready = false;

    auto exeRun = [&]() {
        dbg->start(pid);
        std::unordered_map<std::string, const stream&> breakpoints;
        for (auto& stream : streams) {
            if (dbg->addBreakpoint(stream.getBreakPoint()).empty()) {
                continue;
            }
            breakpoints.insert({stream.getBreakPoint(), stream});
        }

        {
            std::unique_lock<std::mutex> lock(mutex);
            ready = true;
            cv.notify_all();
        }

        while (dbg->continueExec()) {
            std::string breakpoint = dbg->currentBreakpointHit();
            if (breakpoint == "")
                continue;
            out->write(breakpoints.at(breakpoint).handle(*dbg));
        }
        delete dbg;
    };

    std::thread t(exeRun);
    t.detach();

    {
        std::unique_lock<std::mutex> lock(mutex);
        while (!ready)
            cv.wait(lock);
    }
    return;
}
