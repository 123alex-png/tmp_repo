#ifndef DUMMY_CONFIG_HPP
#define DUUMMY_CONFIG_HPP

#include "common-config.hpp"

config* dummy() {
    vector<string> args{"a.out"};
    vector<stream> streams;
    streams.push_back(
        stream("break1", "a.cpp:10", new simpleBreakpointHandler({"x", "y"})));
    return new config("a.out", args, new simpleProcessFilter(), streams);
}

#endif
