#include <common.hpp>
#include <condition_variable>
#include <thread>
#include <unistd.h>
#include <unordered_map>

using std::string;

extern interface* gdbInterfacebuild(const string& pid,
                                    const std::vector<string>& argv);

void run_monitor(const std::vector<stream>& streams, const string& pid,
                 const std::vector<string>& argv, socketClose& sync) {
    std::mutex mutex;
    std::condition_variable cv;
    bool ready = false;
    auto exeRun = [&]() {
        interface* dbg = gdbInterfacebuild(pid, argv);
        std::unordered_map<string, const stream&> breakpoints;
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

        if (sync.inuse()) {
            sync.wait();
            sync.close_all();
        }

        while (dbg->continueExec()) {
            string breakpoint = dbg->currentBreakpointHit();
            if (breakpoint == "")
                continue;
            breakpoints.at(breakpoint).handle(breakpoint, *dbg)->output();
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
