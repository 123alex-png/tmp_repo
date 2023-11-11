#include <common.hpp>
#include <iostream>

using std::cout;
using std::endl;
using std::string;
using std::vector;

class simpleData : public data {
	private:
	vector<string> vals;

	public:
	simpleData(const vector<string>& vals) : vals(vals) {}
	void output() const {
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
		return new simpleData(vals);
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