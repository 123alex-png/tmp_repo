#include <common.hh>
// implementation of common.hh

// implementation of class stream
stream::stream(const std::string& name, const std::string& breakpoint,
               breakpointhandler* handler)
    : name(name), breakpoint(breakpoint), handler(handler) {}

std::string stream::getName() const { return name; }

std::string stream::getBreakPoint() const { return breakpoint; }

data* stream::handle(const std::string& breakpoint, interface& inter) const {
    return handler->handle(breakpoint, inter);
}

// implementation of class config
config::config(const std::string& name,
               const std::vector<std::string>& arguments, processFilter* filter,
               const std::vector<stream>& streams, bool cmdline)
    : name(name), arguments(arguments), filter(filter), streams(streams),
      cmdline(cmdline) {}

std::string config::getName() const { return name; }

std::vector<std::string> config::getArguments() const { return arguments; }

bool config::getCmdline() const { return cmdline; }

bool config::chk(int pid) const { return filter->check(pid); }

std::vector<stream> config::getStreams() const { return streams; }

// implementation of class socketClose

socketClose::socketClose(int clientSocket, bool use)
    : clientSocket(clientSocket), use(use) {}
void socketClose::wait() {
    std::unique_lock<std::mutex> lock(mutex);
    cv.wait(lock);
}

bool socketClose::inuse() const { return use; }

void socketClose::notify() {
    std::unique_lock<std::mutex> lock(mutex);
    ready = true;
    cv.notify_all();
}

void socketClose::closeAll() { close(clientSocket); }
