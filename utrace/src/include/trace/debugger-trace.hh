#pragma once

#include <trace/trace.hh>
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
    std::shared_ptr<breakpointhandler> handler;

public:
    stream(const std::string& name, const std::string& breakpoint,
           std::shared_ptr<breakpointhandler> handler);
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

class debuggerTrace : public trace {
private:
    std::unordered_map<std::string, std::shared_ptr<eventHandler> > eventHandlers;
    std::vector<stream> streams;

protected:
    virtual std::unique_ptr<debugger> getDebugger(pid_t pid) = 0;

public:
    debuggerTrace(bool cmdlineOutput, std::shared_ptr<output> out, const std::vector<stream>& streams);
    void work(const pid_t& pid, const int sockfd) override;
};
