#ifndef WATCH_HH
#define WATCH_HH

#include <common.hh>
void runMonitor(const std::vector<stream>& streams, const std::string& pid,
                 const std::vector<std::string>& argv, socketClose& sync);
bool check(std::string filename, config* cfg);

void execEventWatch(config*);
void cmdRun(config*, const std::vector<std::string>&);
void pidAttach(config*, int);
int portWatch(config*, const std::string&, const std::string&, int);

#endif
