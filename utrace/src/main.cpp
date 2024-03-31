#include <boost/program_options.hpp>
#include <cassert>
#include <connection/connection.hh>
#include <iostream>

int main(int argc, const char* argv[]) {
    std::cout << "Hello, world!" << std::endl;
    namespace po = boost::program_options;
    po::options_description desc("Allowed options");

    // clang-format off
    desc.add_options()
        ("help", "produce help message");
    // clang-format on

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }

    connection conn(1, "/tmp/utrace.sock");
    conn.watch();
    return 0;
}