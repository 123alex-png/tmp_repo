#include <assert.h>
#include <common.hpp>
#include <cstring>
#include <iostream>
#include <map>
#include <stdarg.h>
#include <thread>
#include <unistd.h>
#include <vector>

using std::cout;
using std::endl;

#define DEBUG

#ifdef DEBUG
#define LOG(fmt, ...)                                                          \
	printf("[%s  %d]:" fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__)
#else
#define LOG(fmt, ...) ((void)0)
#endif
using std::string;

class xpipe {
	private:
	int fa2ch[2], ch2fa[2];

	public:
	xpipe() { assert(pipe(fa2ch) == 0 && pipe(ch2fa) == 0); }
	int ch2fa_read() { return ch2fa[0]; }
	int ch2fa_write() { return ch2fa[1]; }
	int fa2ch_read() { return fa2ch[0]; }
	int fa2ch_write() { return fa2ch[1]; }

	void ch_init() {
		close(ch2fa_read());
		close(fa2ch_write());
		return;
	}

	void fa_init() {
		close(ch2fa_write());
		close(fa2ch_read());
		return;
	}

	void fa_write(const char* fmt, ...) {
		va_list v;
		va_start(v, fmt);
		static char buffer[256];
		int len = vsnprintf(buffer, 256, fmt, v);
		assert(len < 256);
		// LOG("WRITE: %s",buffer);
		write(fa2ch_write(), buffer, len);
	}

	std::vector<string> fa_getline() {
		static char buffer[256];
		buffer[0] = 0;
		int i;
		for (i = 0;; i++) {
			read(ch2fa_read(), buffer + i, 1);
			if (buffer[i] == '\n')
				break;
		}
		buffer[i] = 0;
		// LOG("READ: %s",buffer);
		std::vector<string> ret;
		for (char* p = strtok(buffer, " ,"); p; p = strtok(NULL, " ,")) {
			string s = string(p);
			if (s == "(gdb)")
				continue;
			if (s == "exited")
				exit(0);
			ret.push_back(s);
		}
		// for(string &s:ret) LOG("Divided: %s",s.c_str());
		return ret;
	}
};

static void gdb_run(const string& pid, const std::vector<stream>& streams) {
	xpipe communicate;
	pid_t p = fork();
	if (!p) { // run gdb
		LOG("Try to run gdb!");
		communicate.ch_init();
		dup2(communicate.fa2ch_read(), STDIN_FILENO);
		dup2(communicate.ch2fa_write(), STDOUT_FILENO);
		execlp("gdb", "gdb", "-p", pid.c_str(), NULL);
		assert(0);
	} else { // communicate with gdb
		LOG("Try to communicate!");
		communicate.fa_init();
		std::map<string, int> breakpoint_id;
		std::vector<string> line;
		for (int i = 0; i < streams.size(); i++) {
			communicate.fa_write("break %s\n",
								 streams[i].getBreakPoint().c_str());
			while (1) {
				line = communicate.fa_getline();
				if (line.size() > 3 && line[0] == "Breakpoint")
					break;
			}
			breakpoint_id[line[1]] = i;
		}
		communicate.fa_write("continue\n");
		while (1) {
			while (1) {
				line = communicate.fa_getline();
				if (line.size() > 0 && line[0] == "Breakpoint")
					break;
			}
			int id = breakpoint_id[line[1]];
			auto readVar = [&](const string& var) -> string {
				communicate.fa_write("print %s\n", var.c_str());
				while (1) {
					line = communicate.fa_getline();
					if (line.size() > 0 && line[1] == "=")
						break;
				}
				return line[2];
			};
			streams[id].handle(readVar)->output();
			communicate.fa_write("continue\n");
		}

		exit(0);
	}
}

void run_monitor(const string& pid, const std::vector<stream>& streams) {
	gdb_run(pid, streams);
}
