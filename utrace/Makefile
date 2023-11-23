SRC=$(filter-out ./example-input/a.cpp, $(shell find -name "*.cpp"))
include=$(filter-out ./example-input/start-client.hpp, $(shell find -name "*.hpp"))
INC=-I./include
LDFLAGS=-lboost_program_options
CFLAGS=-std=c++17 -g $(INC) $(LDFLAGS)
TARGET=build/test
TARARGS=/home/caoyunfan/busybox-1.36.1/busybox_unstripped tar -xvf  /tmp/archive.tar
ARGS=--config 2 -p 1145

$(TARGET):$(SRC) $(include)
	clang-format -i $(SRC) $(filter-out ./include/toml.hpp, $(include))
	mkdir -p build
	g++ $(SRC) -o $@ $(CFLAGS) 

run: $(TARGET)
	sudo ./$< $(ARGS)

gdb: $(TARGET)
	sudo gdb ./$< --args $< $(ARGS)

.phony:run gdb