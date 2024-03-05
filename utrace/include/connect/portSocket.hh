#pragma once

#include <connect/connect.hh>

class portSocket : public connection {
public:
    portSocket(const std::string& name, const std::vector<std::string>& args,
               processFilter* filter, trace* trc, int maxClinet, int port);
};