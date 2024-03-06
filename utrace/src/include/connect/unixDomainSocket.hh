#pragma once

#include <connect/connect.hh>

class unixDomainSocket : public connection {
public:
    unixDomainSocket(const std::string& name,
                     const std::vector<std::string>& args,
                     processFilter* filter, trace* trc, int maxClient,
                     const std::string& path);
};