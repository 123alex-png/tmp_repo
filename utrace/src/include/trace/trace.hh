#pragma once

#include <output/output.hh>

class trace {
private:
    bool cmdlineOutput;
    std::shared_ptr<output> out;

protected:
    void outputWrite(const data &data);
    void cmdLine(const pid_t& pid);

public:
    trace(bool cmdlineOutput, std::shared_ptr<output> out);
    virtual void work(const pid_t& pid, const int sockfd) = 0;
};
