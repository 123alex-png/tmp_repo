#include <netinet/in.h>
#include <sstream>
#include <sys/socket.h>
#include <trace/none-debugger-trace.hh>
#include <unistd.h>

// implementation of class nonDebuggerTrace

nonDebuggerTrace::nonDebuggerTrace(bool cmdlineOutput,
                                   std::shared_ptr<output> out)
    : trace(cmdlineOutput, out) {}

void nonDebuggerTrace::work(const pid_t& pid, const int sockfd) {
    cmdLine(pid);

    unsigned succ = 0xdeadbeef;
    if (send(sockfd, &succ, sizeof(succ), 0) != sizeof(succ))
        throw std::runtime_error("send failed");

    auto magichk = [&](unsigned long long MAGIC) {
        unsigned long long magic;
        int len = recv(sockfd, &magic, sizeof(magic), 0);
        if (len == 0)
            return 0;
        if (len < 0)
            throw std::runtime_error("recv failed");
        if (magic != MAGIC)
            throw std::runtime_error("invalid magic number");
        return len;
    };

    while (true) {

        if (magichk(0x12345678) == 0)
            break;

        unsigned long long time;
        if (recv(sockfd, &time, sizeof(time), 0) < 0)
            throw std::runtime_error("recv failed");

        if (magichk(0x87654321) == 0)
            break;

        long long length;
        if (recv(sockfd, &length, sizeof(length), 0) < 0)
            throw std::runtime_error("recv failed");
        std::stringstream ss;
        char buffer[1024];
        while (length > 0) {
            int n = recv(sockfd, buffer,
                         sizeof(buffer) > length ? length : sizeof(buffer), 0);
            if (n < 0)
                throw std::runtime_error("recv failed");
            ss.write(buffer, n);
            length -= n;
        }

        if (magichk(0xdeadbeef) == 0)
            break;

        outputWrite(stringData(ss.str(), time));
    }
    close(sockfd);
}