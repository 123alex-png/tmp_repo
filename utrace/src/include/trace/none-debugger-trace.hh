#pragma once

#include <trace/trace.hh>

class nonDebuggerTrace: public trace {
public:
    nonDebuggerTrace(bool cmdlineOutput, std::shared_ptr<output> out);
    void work(const pid_t& pid, const int sockfd) override;
};
