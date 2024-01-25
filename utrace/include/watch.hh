#ifndef WATCH_HH
#define WATCH_HH

#include <common.hh>
bool check(int, config*);

int portWatch(config*, const std::string&, const std::string&, int);
int portPidWatch(config*, const std::string&, const int, int);
int unixDomainSocketWatch(config*, const std::string&, const std::string&, int);
#endif
