#pragma once

#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>
#include <vector>

class data {
private:
    time_t time;

public:
    data();
    time_t getTime() const;
    virtual std::string toString() const = 0;
};

class cmdlineData : public data {
private:
    std::vector<std::string> args;

public:
    cmdlineData(const pid_t pid);
    std::string toString() const override;
};

class simpleData : public data {
private:
    std::string breakpoint;
    std::vector<std::string> vals;

public:
    simpleData(const std::string breakpoint,
               const std::vector<std::string>& vals);
    std::string toString() const override;
};

class output {
private:
    std::ofstream file;
    std::streambuf* outputBuffer;

public:
    output(const std::string& outputFile);
    output();
    void write(const data& d);
    void close();
};
