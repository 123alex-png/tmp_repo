#ifndef COMMON_HH
#define COMMON_HH

#include <condition_variable>
#include <fstream>
#include <mutex>
#include <string>
#include <unistd.h>
#include <vector>

class data {
public:
    virtual void output(const std::string& outputFile) const = 0;
};

class interface {
public:
    virtual std::string addBreakpoint(const std::string&) = 0;
    virtual bool continueExec() = 0;
    virtual std::string currentBreakpointHit() const = 0;
    virtual std::string expEval(const std::string&) = 0;
};

class breakpointhandler {
public:
    virtual data* handle(const std::string&, interface&) = 0;
};

class stream {
private:
    std::string name, breakpoint;
    breakpointhandler* handler;

public:
    stream(const std::string&, const std::string&, breakpointhandler*);
    std::string getName() const;
    std::string getBreakPoint() const;
    data* handle(const std::string&, interface&) const;
};

class processFilter {
public:
    virtual bool check(const int&) = 0;
};

class config {
private:
    std::string name;
    bool cmdline;
    std::vector<std::string> arguments;
    processFilter* filter;

    std::vector<stream> streams;

public:
    config(const std::string&, const std::vector<std::string>&, processFilter*,
           const std::vector<stream>&, bool);
    std::string getName() const;
    std::vector<std::string> getArguments() const;
    bool getCmdline() const;
    bool chk(int pid) const;
    std::vector<stream> getStreams() const;
};

class socketClose {
private:
    std::mutex mutex;
    std::condition_variable cv;
    int clientSocket;
    bool ready = false, use;

public:
    socketClose(int, bool);
    void wait();
    bool inuse() const;
    void notify();
    void closeAll();
};

#endif
