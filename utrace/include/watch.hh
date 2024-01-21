#ifndef WATCH_HH
#define WATCH_HH

#include <common.hh>
void runMonitor(const std::vector<stream>& streams, const std::string& pid,
                 const std::vector<std::string>& argv, socketClose& sync);
bool check(std::string filename, config* cfg);

int portWatch(config*, const std::string&, const std::string&, int);
void cmdlineGen(const std::string& pid, const std::string& outputFile);
int portPidWatch(config*, const std::string&, const int, int);
int unixDomainSocketWatch(config* , const std::string&, const std::string&, int);
#endif
