#ifndef TRACE_HH
#define TRACE_HH

#include <common.hh>

void runMonitor(const std::vector<stream>&, const int& pid,
                const std::vector<std::string>&, socketClose&,
                const std::string&);
interface* gdbInterfacebuild(const int&, const std::vector<std::string>&);

#endif
