#include <cassert>
#include <common.hpp>
#include <iostream>

config* simpleConfigInitialize();
void watch(config* cfg);
int main(int argc, char* argv[]) {
	std::cout << "Hello, world!" << std::endl;
	config* cfg = simpleConfigInitialize();
	watch(cfg);
	return 0;
}