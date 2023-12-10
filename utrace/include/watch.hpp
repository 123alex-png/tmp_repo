#ifndef WATCH_HPP
#define WATCH_HPP

#include <common.hpp>
void run_monitor(const std::vector<stream>& streams, const std::string& pid,
                 const std::vector<std::string>& argv, socketClose& sync);
bool check(std::string filename, config* cfg);

void execEventWatch(config*);
void cmdRun(config*, const std::vector<std::string>&);
void pidAttach(config*, int);
int port_watch(config*, int, int);

#endif
