#include "gdb-hook.hpp"
#include <common.hpp>
#include <unordered_map>
using std::string;

void run_monitor(const string& pid, const std::vector<stream>& streams) {
	interface* dbg = gdbInterfacebuild(pid);
	std::unordered_map<string, const stream&> breakpoints;
	for (auto& stream : streams) {
		if (dbg->addBreakpoint(stream.getBreakPoint()).empty())
			continue;
		breakpoints.insert({stream.getBreakPoint(), stream});
	}
	while (dbg->continueExec()) {
		string breakpoint = dbg->currentBreakpointHit();
		if (breakpoint == "")
			continue;
		breakpoints.at(breakpoint).handle(breakpoint, *dbg)->output();
	}
	delete dbg;
	return;
}
