#pragma once

#include <config/config.hh>

class coreutilConfig : public config {
public:
    coreutilConfig(output* out, const std::string& portFile);
};
