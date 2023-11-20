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
	data* handle(const string& breakpoint, interface& inter) override {
		vector<string> vals;
		for (auto& var : vars)
			vals.push_back(inter.readVar(var).second);
		return new simpleData(breakpoint, vals);
	}
};

class simpleProcessFilter : public processFilter {
public:
	bool check(const string& filename) { return true; }
};

config* tarFile() {
	vector<string> args{"busybox_unstripped", "tar"};
	vector<stream> streams;
	streams.push_back(stream("xfopen", "xfuncs_printf.c:137",
							 new simpleBreakpointHandler({"path", "fp"})));

	streams.push_back(stream("xopen3", "xfuncs_printf.c:148",
							 new simpleBreakpointHandler({"pathname", "ret"})));

	streams.push_back(stream("open3_or_warn", "xfuncs_printf.c:166",
							 new simpleBreakpointHandler({"pathname", "ret"})));

	streams.push_back(stream("dir", "data_extract_all.c:167",
							 new simpleBreakpointHandler({"dst_name", "res"})));

	streams.push_back(stream("dstdir", "make_directory.c:96",
							 new simpleBreakpointHandler({"path"})));

	return new config("busybox_unstripped", args, new simpleProcessFilter(),
					  streams);
}

config* dummy() {
	vector<string> args{"a.out"};
	vector<stream> streams;
	streams.push_back(
		stream("break1", "a.cpp:8", new simpleBreakpointHandler({"x", "y"})));
	return new config("a.out", args, new simpleProcessFilter(), streams);
}

config* simpleConfigInitialize() {
	int id = 2;
	switch (id) {
	case 1:
		return dummy();
	case 2:
		return tarFile();
	default:
		return NULL;
	}
}