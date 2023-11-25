#include "start-client.hpp"
#include <iostream>
#include <string>
int main(int argc, char* argv[]) {
    start_client(argc > 2 ? std::stoi(std::string(argv[1])) : 1145);
    int x = 0, y = 0;
    for (int i = 1; i < 10; i++) {
        x = x + 1;
        y = y + 2;
        std::cout << x << ' ' << y << std::endl;
    }
}
