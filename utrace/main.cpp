#include <boost/program_options.hpp>
#include <cassert>
#include <common.hpp>
#include <iostream>
#include <watch.hpp>

config* simpleConfigInitialize(std::string config_file);
int main(int argc, const char* argv[]) {
	std::cout << "Hello, world!" << std::endl;
	namespace po = boost::program_options;
	po::options_description desc("Allowed options");

	int pid = 0, port = 0;
	std::string config_file = "";
	std::vector<std::string> exec;

	desc.add_options()("help", "produce help message")(
		"config", po::value<std::string>(&config_file), "set config file")(
		"exec", po::value<std::vector<std::string>>(&exec), "set run command")(
		"pid", po::value<int>(&pid), "set pid to be watched")(
		"port,p", po::value<int>(&port), "set port to be listened");

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if (vm.count("help")) {
		std::cout << desc << std::endl;
		return 1;
	}
	if (vm.count("config")) {
		std::cout << "Config file: " << config_file << std::endl;
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
		std::cout << "Port: " << port << std::endl;
	}
	config* cfg = simpleConfigInitialize(config_file);
	if (pid != 0) {
		pidAttach(cfg, pid);
	} else if (port != 0) {
		port_watch(cfg, port, 1);
	} else if (!exec.empty()) {
		cmdRun(cfg, exec);
	} else {
		execEventWatch(cfg);
	}
	return 0;
}