#ifndef COMMON_HPP
#define COMMON_HPP

#include <functional>
#include <string>
#include <vector>

class data {
	public:
	virtual void output() const = 0;
};

class breakpointhandler {
	public:
	virtual data*
	handle(const std::string& breakpoint,
		   std::function<std::string(const std::string&)> readVar) = 0;
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
	data* handle(std::function<std::string(const std::string&)> readVar) const {
		return handler->handle(breakpoint, readVar);
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

#endif
