#include <iostream>
#include <assert.h>
#include <common.hpp>

extern void parse_file(const char * filename);
extern void watch(const string name);
int main(int argc, char *argv[]) {
    std::cout << "Hello, world!" << std::endl;
    assert(argc==2);
    parse_file(argv[1]);
    watch(name);
    return 0;
}