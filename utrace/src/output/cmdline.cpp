#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <output.hh>
#include <sstream>
#include <string>
#include <time.h>
#include <unistd.h>
#include <vector>

using std::string;
void cmdline(const int& pid, const string& outputFile) {

    //    std::cout << std::endl<< "Generating cmdline for " << pid << " at "<<
    //    std::ctime(new time_t);
    std::ifstream file("/proc/" + std::to_string(pid) + "/cmdline");
    std::vector<string> arguments;
    if (file.is_open()) {
        std::stringstream buffer;
        buffer << file.rdbuf();
        // output all the command line
        std::string cmdline = buffer.str();
        std::istringstream iss(cmdline);
        std::string argument;
        while (getline(iss, argument, '\0')) {
            arguments.push_back(argument);
        }
    }
    using json = nlohmann::json;
    json j;
    j["cmdline"] = arguments;
    j["time"] = std::time(nullptr);
    if (!outputFile.empty()) {
        std::ofstream o(outputFile, std::ios::app);
        o << j.dump() << std::endl;
    } else {
        std::cout << j.dump() << std::endl;
    }
}
