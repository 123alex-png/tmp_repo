#include <common.hh>
#include <cstring>
#include <iostream>
#include <linux/cn_proc.h>
#include <linux/connector.h>
#include <linux/netlink.h>
#include <netinet/in.h>
#include <sys/procfs.h>
#include <sys/socket.h>
#include <unistd.h>
#include <watch.hh>

using std::string;

static int nlConnect() {
    int rc;
    int nlSock;
    struct sockaddr_nl sa_nl;

    nlSock = socket(PF_NETLINK, SOCK_DGRAM, NETLINK_CONNECTOR);
    if (nlSock == -1) {
        perror("socket");
        return -1;
    }
    int bufferSize = 4 * 1024 * 1024; // 4MB
    setsockopt(nlSock, SOL_SOCKET, SO_RCVBUF, &bufferSize, sizeof(bufferSize));

    sa_nl.nl_family = AF_NETLINK;
    sa_nl.nl_groups = CN_IDX_PROC;
    sa_nl.nl_pid = getpid();

    rc = bind(nlSock, (struct sockaddr*)&sa_nl, sizeof(sa_nl));
    if (rc == -1) {
        perror("bind");
        close(nlSock);
        return -1;
    }

    return nlSock;
}

static int setProcEvListen(int nlSock, bool enable) {
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

    rc = send(nlSock, &nlcn_msg, sizeof(nlcn_msg), 0);
    if (rc == -1) {
        perror("netlink send");
        return -1;
    }

    return 0;
}

static int handleProcEv(int nlSock, config* cfg) {
    int rc;
    struct __attribute__((aligned(NLMSG_ALIGNTO))) {
        struct nlmsghdr nl_hdr;
        struct __attribute__((__packed__)) {
            struct cn_msg cn_msg;
            struct proc_event proc_ev;
        };
    } nlcn_msg;

    while (true) {
        rc = recv(nlSock, &nlcn_msg, sizeof(nlcn_msg), 0);
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
                runMonitor(cfg->getStreams(), filename, cfg->getArguments(),
                           sync);
            }
        }
    }
}

void execEventWatch(config* cfg) {
    int nl_sock = nlConnect();
    if (nl_sock == -1)
        return;

    if (setProcEvListen(nl_sock, true) < 0) {
        close(nl_sock);
        return;
    }

    if (handleProcEv(nl_sock, cfg) < 0) {
        close(nl_sock);
        return;
    }
    setProcEvListen(nl_sock, false);
    close(nl_sock);
    return;
}
