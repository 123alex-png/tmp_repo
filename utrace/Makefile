SRC=$(shell find -name "*.cpp")
include=$(shell find -name "*.hpp")
INC=-I./include
TARGET=build/test

$(TARGET):$(SRC) $(include)
	clang-format -i $(SRC) $(filter-out ./include/toml.hpp, $(include))
	mkdir -p build
	g++ -std=c++17 $(INC) -g $(SRC) -o $@ 

run: $(TARGET)
	./$< example-input/config.toml

gdb: $(TARGET)
	gdb ./$< example-input/config.toml

.phony:run gdb