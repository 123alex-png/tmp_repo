#ifndef CORE_UTIL_CONFIG_HPP
#define CORE_UTIL_CONFIG_HPP

#include "common-config.hpp"
#include <fstream>
#include <sstream>
#include <string>

static const std::vector<std::string> coreutilList = {
    "[",         "[[",      "arch",    "ash",      "base32",    "base64",
    "basename",  "cat",     "chgrp",   "chmod",    "chown",     "chroot",
    "cksum",     "comm",    "cp",      "crc32",    "cttyhack",  "cut",
    "date",      "dd",      "df",      "dirname",  "dos2unix",  "du",
    "echo",      "env",     "expand",  "expr",     "factor",    "false",
    "fold",      "fsync",   "groups",  "head",     "hostid",    "hush",
    "id",        "install", "link",    "ln",       "logname",   "ls",
    "md5sum",    "mkdir",   "mkfifo",  "mknod",    "mktemp",    "mv",
    "nice",      "nl",      "nohup",   "nproc",    "od",        "paste",
    "printenv",  "printf",  "pwd",     "readlink", "realpath",  "rm",
    "rmdir",     "seq",     "sh",      "sha1sum",  "sha256sum", "sha3sum",
    "sha512sum", "shred",   "shuf",    "sleep",    "sort",      "split",
    "stat",      "stty",    "sum",     "sync",     "tac",       "tail",
    "tee",       "test",    "timeout", "touch",    "tr",        "true",
    "truncate",  "tsort",   "tty",     "uname",    "unexpand",  "uniq",
    "unix2dos",  "unlink",  "users",   "usleep",   "uudecode",  "uuencode",
    "w",         "wall",    "wc",      "who",      "whoami",    "yes"};

class coreUtilFilter : public processFilter {
public:
    bool check(const int& pid) override {
        std::ifstream file("/proc/" + std::to_string(pid) + "/comm");
        if (!file.is_open())
            return false;
        std::stringstream buffer;
        buffer << file.rdbuf();
        std::string comm = buffer.str();
        comm.erase(comm.find_last_not_of(" \n\r\t") + 1);
        for (const auto& coreutil : coreutilList) {
            if (comm.find(coreutil) != std::string::npos)
                return true;
        }
        return false;
    }
};

config* coreutilConfig() {
    return new config("coreutil", {}, new coreUtilFilter(), {}, true);
}

#endif
