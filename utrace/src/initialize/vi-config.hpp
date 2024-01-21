#ifndef VI_CONFIG_HPP
#define VI_CONFIG_HPP

#include "common-config.hpp"

class safeWriteBreakpointHandler : public breakpointhandler {
public:
    data* handle(const string& breakpoint, interface& inter) override {
        int count = std::stoi(inter.expEval("count"));
        vector<string> data;
        for (int i = 0; i < count; i++) {
            string exp = "((char *)buf)[" + std::to_string(i) + "]";
            data.push_back(inter.expEval(exp));
        }
        return new simpleData(breakpoint, data);
    }
};

class vifwriteBreakpointHandler : public breakpointhandler {
public:
    data* handle(const string& breakpoint, interface& inter) override {
        int cs = std::stoi(inter.expEval("cs"));
        int ce = std::stoi(inter.expEval("ce"));
        vector<string> data;
        for (int i = cs; i <= ce; i++) {
            string var = "sp[" + std::to_string(i) + "]";
            data.push_back(inter.expEval(var));
        }
        return new simpleData(breakpoint, data);
    }
};

config* viWrite() {
    vector<string> args{"busybox_unstrip", "vi"};
    vector<stream> streams;
    streams.push_back(stream("safe_write", "safe_write.c:17",
                             new safeWriteBreakpointHandler()));
    // streams.push_back(stream("vifwrite", "vi.c:1073", new
    // vifwriteBreakpointHandler()));
    return new config("busybox_unstrip", args, new simpleProcessFilter(),
                      streams);
}

#endif
