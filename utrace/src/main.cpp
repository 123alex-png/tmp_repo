#include <boost/program_options.hpp>
#include <cassert>
#include <common.hh>
#include <iostream>
#include <watch.hh>

config* simpleConfigInitialize(std::string config_file);
int main(int argc, const char* argv[]) {
    std::cout << "Hello, world!" << std::endl;
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");

    int pid = 0;
    std::string portFile = "";
    std::string configFile = "";
    std::string outputFile = "";
    std::vector<std::string> exec;

    desc.add_options()("help", "produce help message")(
        "config,c", po::value<std::string>(&configFile),
        "set config file")("exec,e", po::value<std::vector<std::string>>(&exec),
                           "set run command")("pid,p", po::value<int>(&pid),
                                              "set pid to be watched")(
        "port,l", po::value<std::string>(&portFile), "set port to be listened")(
        "output,o", po::value<std::string>(&outputFile), "set output file");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }
    if (vm.count("config")) {
        std::cout << "Config file: " << configFile << std::endl;
    }
    if (vm.count("exec")) {
        std::cout << "Exec: ";
        for (const auto& exe : exec) {
            std::cout << exe << " ";
        }
        std::cout << std::endl;
    }
    if (vm.count("pid")) {
        std::cout << "Pid: " << pid << std::endl;
    }
    if (vm.count("port")) {
        std::cout << "Port: " << portFile << std::endl;
    }
    if (vm.count("output")) {
        std::cout << "Output: " << outputFile << std::endl;
    }
    config* cfg = nullptr;
    if (!configFile.empty())
        cfg = simpleConfigInitialize(configFile);
    if (pid != 0) {
        pidAttach(cfg, pid);
    } else if (!portFile.empty()) {
        portWatch(cfg, outputFile, portFile, 1);
    } else if (!exec.empty()) {
        cmdRun(cfg, exec);
    } else {
        execEventWatch(cfg);
    }
    return 0;
}