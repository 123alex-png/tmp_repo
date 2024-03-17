#include <boost/program_options.hpp>
#include <cassert>
#include <config/config.hh>
#include <iostream>
#include <output/output.hh>

int main(int argc, const char* argv[]) {
    std::cout << "Hello, world!" << std::endl;
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");

    int pid = 0;
    std::string portFile = "";
    std::string configFile = "";
    std::string outputFile = "";
    std::vector<std::string> exec;

    // clang-format off
    desc.add_options()
        ("help", "produce help message")
        ("config,c", po::value<std::string>(&configFile),"set config file")
        ("port,l", po::value<std::string>(&portFile), "set port to be listened")
        ("output,o", po::value<std::string>(&outputFile), "set output file");
    // clang-format on

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }
    if (vm.count("config")) {
        std::cout << "Config file: " << configFile << std::endl;
    } else
        std::cout << "No config file was set." << std::endl;

    if (vm.count("port")) {
        std::cout << "Port: " << portFile << std::endl;
    } else
        throw std::invalid_argument("Port file is required");

    output* out = nullptr;
    if (vm.count("output")) {
        std::cout << "Output: " << outputFile << std::endl;
        out = new output(outputFile);
    } else
        out = new output();

    configInit* simpleConfigInitialize =
        new configInit(configFile, portFile, out);
    config* cfg = simpleConfigInitialize->getConfig();
    connection* conn = cfg->getConnect();
    conn->watch();
    return 0;
}