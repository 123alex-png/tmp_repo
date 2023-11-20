SRC=$(shell find -name "*.cpp")
include=$(shell find -name "*.hpp")
INC=-I./include
TARGET=build/test
ARGS=/home/caoyunfan/busybox-1.36.1/busybox_unstripped tar -cvf /tmp/archive.tar /tmp/dummy

$(TARGET):$(SRC) $(include)
	clang-format -i $(SRC) $(filter-out ./include/toml.hpp, $(include))
	mkdir -p build
	g++ -std=c++17 $(INC) -g $(SRC) -o $@ 

run: $(TARGET)
	./$< $(ARGS)

gdb: $(TARGET)
	gdb ./$< --args $< $(ARGS)

.phony:run gdb