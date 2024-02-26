#include "core-util-config.hpp"
#include "dummy-config.hpp"
#include "tar-config.hpp"
#include "vi-config.hpp"
#include "whisper-cpp-config.hpp"

config* simpleConfigInitialize(string configFile) {
    int id = std::stoi(configFile);
    switch (id) {
    case 1:
        return dummy();
    case 2:
        return tarFile();
    case 3:
        return viWrite();
    case 4:
        return coreutilConfig();
    case 5:
        return whisperCppConfig();
    default:
        return NULL;
    }
}