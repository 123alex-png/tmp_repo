#include <common.hpp>
#include <cstring>
#include <fstream>
#include <iostream>
#include <linux/cn_proc.h>
#include <linux/connector.h>
#include <linux/netlink.h>
#include <netinet/in.h>
#include <sstream>
#include <sys/procfs.h>
#include <sys/socket.h>
#include <unistd.h>
#include <watch.hpp>

using std::string;

extern void run_monitor(const std::vector<stream>& streams, const string& pid,
                        const std::vector<string>& argv, socketClose& sync);

static int nl_connect() {
    int rc;
    int nl_sock;
    struct sockaddr_nl sa_nl;

    nl_sock = socket(PF_NETLINK, SOCK_DGRAM, NETLINK_CONNECTOR);
    if (nl_sock == -1) {
        perror("socket");
        return -1;
    }
    int bufferSize = 4 * 1024 * 1024; // 4MB
    setsockopt(nl_sock, SOL_SOCKET, SO_RCVBUF, &bufferSize, sizeof(bufferSize));

    sa_nl.nl_family = AF_NETLINK;
    sa_nl.nl_groups = CN_IDX_PROC;
    sa_nl.nl_pid = getpid();

    rc = bind(nl_sock, (struct sockaddr*)&sa_nl, sizeof(sa_nl));
    if (rc == -1) {
        perror("bind");
        close(nl_sock);
        return -1;
    }

    return nl_sock;
}

static int set_proc_ev_listen(int nl_sock, bool enable) {
    int rc;
    struct __attribute__((aligned(NLMSG_ALIGNTO))) {
        struct nlmsghdr nl_hdr;
        struct __attribute__((__packed__)) {
            struct cn_msg cn_msg;
            enum proc_cn_mcast_op cn_mcast;
        };
    } nlcn_msg;

    memset(&nlcn_msg, 0, sizeof(nlcn_msg));
    nlcn_msg.nl_hdr.nlmsg_len = sizeof(nlcn_msg);
    nlcn_msg.nl_hdr.nlmsg_pid = getpid();
    nlcn_msg.nl_hdr.nlmsg_type = NLMSG_DONE;

    nlcn_msg.cn_msg.id.idx = CN_IDX_PROC;
    nlcn_msg.cn_msg.id.val = CN_VAL_PROC;
    nlcn_msg.cn_msg.len = sizeof(enum proc_cn_mcast_op);

    nlcn_msg.cn_mcast = enable ? PROC_CN_MCAST_LISTEN : PROC_CN_MCAST_IGNORE;

    rc = send(nl_sock, &nlcn_msg, sizeof(nlcn_msg), 0);
    if (rc == -1) {
        perror("netlink send");
        return -1;
    }

    return 0;
}

static bool check(string filename, config* cfg) {
    static char buf[1024];

    FILE* fp = fopen(("/proc/" + filename + "/comm").c_str(), "r");
    if (fp == NULL)
        return false;

    fgets(buf, 1024, fp);
    fclose(fp);
    string comm = buf;
    comm.erase(comm.find_last_not_of(" \n\r\t") + 1);
    if (comm != cfg->getName())
        return false;

    std::vector<std::string> arguments;
    std::ifstream file("/proc/" + filename + "/cmdline");
    if (file.is_open()) {
        std::stringstream buffer;
        buffer << file.rdbuf();

        std::string cmdline = buffer.str();
        std::istringstream iss(cmdline);
        std::string argument;

        while (getline(iss, argument, '\0')) {
            arguments.push_back(argument);
        }
    }
    for (const auto& arg : cfg->getArguments()) {
        bool flag = false;
        for (const auto& argument : arguments) {
            if (argument.find(arg) != std::string::npos) {
                flag = true;
                break;
            }
        }
        if (!flag)
            return false;
    }

    if (!cfg->chk())
        return false;

    return true;
}

static int handle_proc_ev(int nl_sock, config* cfg) {
    int rc;
    struct __attribute__((aligned(NLMSG_ALIGNTO))) {
        struct nlmsghdr nl_hdr;
        struct __attribute__((__packed__)) {
            struct cn_msg cn_msg;
            struct proc_event proc_ev;
        };
    } nlcn_msg;

    while (true) {
        rc = recv(nl_sock, &nlcn_msg, sizeof(nlcn_msg), 0);
        if (rc == 0) {
            /* shutdown? */
            return 0;
        } else if (rc == -1) {
            perror("netlink recv");
            return -1;
        }
        if (nlcn_msg.proc_ev.what == proc_event::PROC_EVENT_EXEC) {
            string filename =
                std::to_string(nlcn_msg.proc_ev.event_data.exec.process_pid);
            if (check(filename, cfg)) {
                std::cout << "pid: " << filename << std::endl;
                socketClose sync(-1, false);
                run_monitor(cfg->getStreams(), filename, cfg->getArguments(),
                            sync);
            }
        }
    }
}

void execEventWatch(config* cfg) {
    int nl_sock = nl_connect();
    if (nl_sock == -1)
        return;

    if (set_proc_ev_listen(nl_sock, true) < 0) {
        close(nl_sock);
        return;
    }

    if (handle_proc_ev(nl_sock, cfg) < 0) {
        close(nl_sock);
        return;
    }
    set_proc_ev_listen(nl_sock, false);
    close(nl_sock);
    return;
}

void cmdRun(config* cfg, const std::vector<string>& argv) {
    socketClose sync(-1, false);
    run_monitor(cfg->getStreams(), "", argv, sync);
    return;
}

void pidAttach(config* cfg, int pid) {
    socketClose sync(-1, false);
    run_monitor(cfg->getStreams(), std::to_string(pid), std::vector<string>(),
                sync);
    return;
}

int port_watch(config* cfg, int port, int max_client) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("socket");
        return -1;
    }

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return -1;
    }

    if (listen(sock, max_client) < 0) {
        perror("listen");
        return -1;
    }
    std::cout << "Listening on port " << port << std::endl;

    while (true) {
        struct sockaddr_in clientAddress;
        socklen_t clientAddressLength = sizeof(clientAddress);
        int client = accept(sock, (struct sockaddr*)&clientAddress,
                            &clientAddressLength);
        if (client < 0) {
            perror("accept");
            return -1;
        }

        struct {
            int pid;
            unsigned magicnumber;
        } data;
        if (recv(client, &data, sizeof(data), 0) < 0) {
            perror("recv");
            return -1;
        }
        if (data.magicnumber != 0xdeadbeef ||
            !check(std::to_string(data.pid), cfg))
            continue;
        socketClose sync(client, true);

        run_monitor(cfg->getStreams(), std::to_string(data.pid),
                    std::vector<string>(), sync);

        // std::cout << "Attaching to pid " << data.pid << std::endl;

        unsigned succ = 0xdeadbeef;
        if (send(client, &succ, sizeof(succ), 0) < 0) {
            perror("send");
            return -1;
        }
        // std::cout << "send " << succ << std::endl;
        close(client);
        sync.notify();
    }
    close(sock);
    return 0;
}