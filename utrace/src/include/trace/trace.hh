#pragma once

#include <output/output.hh>
#include <string>
#include <vector>

class debugger {
public:
    virtual void start(const pid_t& pid) = 0;
    virtual std::string addBreakpoint(const std::string& breakpoint) = 0;
    virtual bool continueExec() = 0;
    virtual std::string currentBreakpointHit() const = 0;
    virtual std::string expEval(const std::string& exp) = 0;
};

class breakpointhandler {
public:
    virtual simpleData handle(const std::string& breakpoint,
                              debugger& interface) = 0;
};

class simpleBreakpointHandler : public breakpointhandler {
private:
    std::vector<std::string> vars;

public:
    simpleBreakpointHandler(const std::vector<std::string>& vars);
    simpleData handle(const std::string& breakpoint,
                      debugger& interface) override;
};

class stream {
private:
    std::string name, breakpoint;
    breakpointhandler* handler;

public:
    stream(const std::string& name, const std::string& breakpoint,
           breakpointhandler* handler);
    std::string getName() const;
    std::string getBreakPoint() const;
    simpleData handle(debugger& dbg) const;
};

class trace {
private:
    bool cmdlineOutput;
    std::vector<stream> streams;
    output* out;
    debugger* dbg;

public:
    trace(const std::vector<stream>& streams, bool cmdlineOutput, output* out,
          debugger* dbg);
    void work(const pid_t& pid);
};
