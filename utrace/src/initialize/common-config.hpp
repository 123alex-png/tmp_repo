#ifndef COMMON_CONFIG_HPP
#define COMMON_CONFIG_HPP

#include <common.hh>
#include <ctime>
#include <iostream>
#include <output.hh>

using std::cout;
using std::endl;
using std::string;
using std::vector;

class simpleData : public data {
private:
    string breakpoint;
    vector<string> vals;

public:
    simpleData(const string breakpoint, const vector<string>& vals)
        : breakpoint(breakpoint), vals(vals) {}
    void output(const string& outputFile) const override {
        cout << time(0) << ": Breakpoint: " << breakpoint << ": ";
        for (auto& val : vals)
            cout << val << ' ';
        cout << endl;
    }
};

class simpleBreakpointHandler : public breakpointhandler {
private:
    vector<string> vars;

public:
    simpleBreakpointHandler(const vector<string>& vars) : vars(vars) {}
    data* handle(const string& breakpoint, interface& inter) override {
        vector<string> vals;
        for (auto& var : vars)
            vals.push_back(inter.expEval(var));
        return new simpleData(breakpoint, vals);
    }
};

class simpleProcessFilter : public processFilter {
public:
    bool check(const int& pid) override { return true; }
};

#endif
