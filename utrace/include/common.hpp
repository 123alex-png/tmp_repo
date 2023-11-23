#ifndef COMMON_HPP
#define COMMON_HPP

#include <condition_variable>
#include <mutex>
#include <string>
#include <unistd.h>
#include <vector>

class data {
public:
	virtual void output() const = 0;
};

class interface {
public:
	virtual std::string addBreakpoint(const std::string& breakPoint) = 0;
	virtual bool continueExec() = 0;
	virtual std::string currentBreakpointHit() const = 0;
	virtual std::string expEval(const std::string& exp) = 0;
};

class breakpointhandler {
public:
	virtual data* handle(const std::string& breakpoint, interface& inter) = 0;
};

class stream {
private:
	std::string name, breakpoint;
	breakpointhandler* handler;

public:
	stream(const std::string& name, const std::string& breakpoint,
		   breakpointhandler* handler)
		: name(name), breakpoint(breakpoint), handler(handler) {}
	std::string getName() const { return name; }
	std::string getBreakPoint() const { return breakpoint; }
	data* handle(const std::string& breakpoint, interface& inter) const {
		return handler->handle(breakpoint, inter);
	}
};

class processFilter {
public:
	virtual bool check(const std::string& filename) = 0;
};

class config {
private:
	std::string name;
	std::vector<std::string> arguments;
	processFilter* filter;

	std::vector<stream> streams;

public:
	config(const std::string& name, const std::vector<std::string>& arguments,
		   processFilter* filter, const std::vector<stream>& streams)
		: name(name), arguments(arguments), filter(filter), streams(streams) {}
	std::string getName() const { return name; }
	std::vector<std::string> getArguments() const { return arguments; }
	bool chk() const { return filter->check(name); }
	std::vector<stream> getStreams() const { return streams; }
};

class socketClose {
private:
	std::mutex mutex;
	std::condition_variable cv;
	int client_socket;
	bool ready = false, use;

public:
	socketClose(int client_socket, bool use)
		: client_socket(client_socket), use(use) {}
	void wait() {
		std::unique_lock<std::mutex> lock(mutex);
		while (!ready)
			cv.wait(lock);
	}
	bool inuse() { return use; }
	void notify() {
		std::unique_lock<std::mutex> lock(mutex);
		ready = true;
		cv.notify_all();
	}
	void close_all() { close(client_socket); }
};

#endif
