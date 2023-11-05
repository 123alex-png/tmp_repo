#ifndef COMMON_HPP
#define COMMON_HPP

#include <string>
#include <vector>
using std::string;
using std::vector;

struct streamHandler{
    string handler;
};

struct stream{
    string name;
    string breakpoint;
    streamHandler handler;
};
extern vector<stream> streams;
extern string name;
extern vector<string> arguments;
extern string Processfilter;

struct data{
    vector<string> vals;
};

#endif
