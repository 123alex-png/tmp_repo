#pragma once

#include <config/config.hh>

class whisperCppCheck : public processFilter {
public:
    bool check(const pid_t& pid) override;
};

class whisperCppConfig : public config {
public:
    whisperCppConfig(output* out, const std::string& portFile);
};
