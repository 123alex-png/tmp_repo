#pragma once

#include <connection/connection.hh>

class receive : public connection {
public:
    receive(trace * trc, int maxClient, int port);
    receive(trace * trc, int maxClient, const std::string& path);
    void watch();
};