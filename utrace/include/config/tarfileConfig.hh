#pragma once
#include <config/config.hh>

class tarfileConfig : public config {
public:
    tarfileConfig(output* out, const std::string& portFile);
};
