#include <config/coreutilConfig.hh>
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

coreutilConfig::coreutilConfig(output* out, const std::string& portFile)
    : config(out) {
    trace* trc = new trace({}, true, getOutput(), trace::dbgType::GDB);
    setTrace(trc);
    connection* conn = nullptr;
    try {
        int port = std::stoi(portFile);
        conn = new connection("coreUtil", coreutilList, {},
                              new simpleProcessFilter(), trc, 1, port);
    } catch (const std::invalid_argument& e) {
        conn = new connection("coreUtil", coreutilList, {},
                              new simpleProcessFilter(), trc, 1, portFile);
    }
    setConnect(conn);
}