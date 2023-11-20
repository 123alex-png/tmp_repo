#include <common.hpp>
#include <cstring>
#include <iostream>
#include <linux/cn_proc.h>
#include <linux/connector.h>
#include <linux/netlink.h>
#include <sys/procfs.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

using std::string;

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

bool check(string filename, config* cfg) {
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

	fp = fopen(("/proc/" + filename + "/cmdline").c_str(), "r");
	if (fp == NULL)
		return false;
	fgets(buf, 1024, fp);
	fclose(fp);
	string cmdline = buf;
	for (const auto& arg : cfg->getArguments())
		if (cmdline.find(arg) == string::npos)
			return false;

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
			extern interface* gdbInterfacebuild(const string& pid);
			string filename =
				std::to_string(nlcn_msg.proc_ev.event_data.exec.process_pid);
			if (check(filename, cfg)) {
				std::cout << "pid: " << filename << std::endl;
				interface* dbg = gdbInterfacebuild(filename);
				extern void run_monitor(const string& pid,
										const std::vector<stream>& streams,
										interface* dbg);
				std::thread t(run_monitor, filename, cfg->getStreams(), dbg);
				t.detach();
			}
		}
	}
}

void watch(config* cfg) {
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
/*
	while (true) {
		DIR* dir = opendir("/proc");
		if (dir == NULL) {
			perror("opendir");
			exit(1);
		}
		struct dirent* entry;
		while ((entry = readdir(dir)) != NULL) {
			if (entry->d_type == DT_DIR && check(entry->d_name, cfg)) {
				std::cout << "pid: " << entry->d_name << std::endl;
				extern void run_monitor(const string& pid,
										const std::vector<stream>& streams);
				std::thread t(run_monitor, entry->d_name,
   cfg->getStreams()); t.join(); return;
			}
		}
		closedir(dir);
	}
*/