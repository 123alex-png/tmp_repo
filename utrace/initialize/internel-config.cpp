#include <common.hpp>
#include <ctime>
#include <iostream>

using std::cout;
using std::endl;
using std::string;
using std::vector;

class simpleData : public data {
	private:
	string breakpoint;
	vector<string> vals;

	public:
	simpleData(const string breakpoint, const vector<string>& vals)
		: breakpoint(breakpoint), vals(vals) {}
	void output() const {
		cout << time(0) << ": Breakpoint: " << breakpoint << ": ";
		for (auto& val : vals)
			cout << val << ' ';
		cout << endl;
	}
};

class simpleBreakpointHandler : public breakpointhandler {
	private:
	vector<string> vars;

	public:
	simpleBreakpointHandler(const vector<string>& vars) : vars(vars) {}
	data* handle(const string& breakpoint,
				 std::function<string(const string&)> readVar) override {
		vector<string> vals;
		for (auto& var : vars)
			vals.push_back(readVar(var));
		return new simpleData(breakpoint, vals);
	}
};

class simpleProcessFilter : public processFilter {
	public:
	bool check(const string& filename) { return true; }
};

config* simpleConfigInitialize() {
	vector<string> args;
	args.push_back("a.out");
	vector<string> vars;
	vars.push_back("x");
	vars.push_back("y");
	vector<stream> streams;
	streams.push_back(
		stream("stdout", "a.cpp:8", new simpleBreakpointHandler(vars)));
	return new config("a.out", args, new simpleProcessFilter(), streams);
}