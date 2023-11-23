#ifndef WATCH_HPP
#define WATCH_HPP

#include <common.hpp>
void execEventWatch(config*);
void cmdRun(config*, const std::vector<std::string>&);
void pidAttach(config*, int);
int port_watch(config*, int, int);

#endif
