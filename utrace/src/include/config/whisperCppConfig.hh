#pragma once

#include <config/config.hh>

class whisperCppConfig : public config {
public:
    whisperCppConfig(output* out, const std::string& portFile);
};
