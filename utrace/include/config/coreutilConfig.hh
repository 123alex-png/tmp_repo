#pragma once

#include <config/config.hh>

class coreUtilFilter : public processFilter {
private:
    const std::vector<std::string> coreutilList = {
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

public:
    bool check(const pid_t& pid) override;
};

class coreutilConfig : public config {
public:
    coreutilConfig(output* out, const std::string& portFile);
};
