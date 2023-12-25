#ifndef TAR_CONFIG_HPP
#define TAR_CONFIG_HPP

#include "common-config.hpp"

config* tarFile() {
    vector<string> args{"tar"};
    vector<stream> streams;
    streams.push_back(stream("fopen", "xfuncs_printf.c:137",
                             new simpleBreakpointHandler({"path", "fp"})));
    streams.push_back(stream("xopen3", "xfuncs_printf.c:148",
                             new simpleBreakpointHandler({"pathname", "ret"})));
    streams.push_back(stream("open3_or_warn", "xfuncs_printf.c:166",
                             new simpleBreakpointHandler({"pathname", "ret"})));
    return new config("tar", args, new simpleProcessFilter(), streams);
}

#endif
