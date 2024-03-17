#pragma once

#include <output/output.hh>
#include <string>
#include <vector>
#include <unordered_map>
#include <optional>

class debugger {
public:
    virtual std::string addBreakpoint(const std::string& breakpoint) = 0;
    virtual std::pair<std::string, std::string> continueExec() = 0;
    virtual std::string evaluateExpression(const std::string& exp) = 0;
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

class eventHandler {
public:
    enum action {CONTINUE, EXIT};
    virtual std::pair<action, std::optional<simpleData> > handle() = 0;
};

class simpleEventHandler : public eventHandler {
public:
    std::pair<action, std::optional<simpleData> > handle() override;
};

class trace {
public:
    enum dbgType {GDB, LLDB};

private:
    std::unordered_map<std::string, eventHandler*> eventHandlers;
    bool cmdlineOutput;
    std::vector<stream> streams;
    output* out;
    dbgType debuggerType;

public:
    trace(const std::vector<stream>& streams, bool cmdlineOutput, output* out,
          dbgType debuggerType);
    void work(const pid_t& pid);
    output* getOutput();
};
