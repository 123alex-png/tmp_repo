#include "dummy-config.hpp"
#include "tar-config.hpp"
#include "vi-config.hpp"

config* simpleConfigInitialize(string config_file) {
	int id = std::stoi(config_file);
	switch (id) {
	case 1:
		return dummy();
	case 2:
		return tarFile();
	case 3:
		return vi_write();
	default:
		return NULL;
	}
}