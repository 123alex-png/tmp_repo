#pragma once

#include <fstream>
#include <iostream>
#include <streambuf>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <memory>

class data {
private:
    time_t time;

protected:
    void setTime(const time_t time);

public:
    data();
    time_t getTime() const;
    virtual nlohmann::json toJson() const = 0;
};

class cmdlineData : public data {
private:
    std::vector<std::string> args;

public:
    cmdlineData(const pid_t pid);
    nlohmann::json toJson() const override;
};

class simpleData : public data {
private:
    std::string breakpoint;
    std::vector<std::string> vals;

public:
    simpleData(const std::string breakpoint,
               const std::vector<std::string>& vals);
    nlohmann::json toJson() const override;
};

class stringData : public data {
private:
    std::string s;

public:
    stringData(const std::string &s, const time_t time);
    nlohmann::json toJson() const override;
};

class jsonData : public data {
private:
    nlohmann::json j;

public:
    jsonData(const nlohmann::json &j, const time_t time);
    nlohmann::json toJson() const override;
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
