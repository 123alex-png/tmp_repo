#include <common.hpp>
#include <cstring>
#include <dirent.h>
#include <iostream>
#include <sys/types.h>
#include <thread>
#include <unistd.h>
#include <unordered_set>

using std::string;
bool check(const string& filename, config* cfg) {
	static char buf[1024];

	if (filename.find_first_not_of("0123456789") != string::npos)
		return false;
	FILE* fp = fopen(("/proc/" + filename + "/comm").c_str(), "r");
	if (fp == NULL) {
		perror("fopen");
		exit(1);
	}

	fgets(buf, 1024, fp);
	fclose(fp);
	string comm = buf;
	comm.erase(comm.find_last_not_of(" \n\r\t") + 1);
	if (comm != cfg->getName())
		return false;

	fp = fopen(("/proc/" + filename + "/cmdline").c_str(), "r");
	if (fp == NULL) {
		perror("fopen");
		exit(1);
	}
	fgets(buf, 1024, fp);
	fclose(fp);
	string cmdline = buf;
	for (const auto& arg : cfg->getArguments())
		if (cmdline.find(arg) == string::npos)
			return false;

	if (!cfg->chk())
		return false;

	static std::unordered_set<string> pids;
	if (pids.find(filename) == pids.end()) {
		pids.insert(filename);
		return true;
	} else
		return false;
}

void watch(config* cfg) {
	while (true) {
		sleep(1);
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
				std::thread t(run_monitor, entry->d_name, cfg->getStreams());
				t.detach();
			}
		}
	}
}