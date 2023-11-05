SRC=$(shell find -name "*.cpp")
INC=-I./include
TARGET=build/test

$(TARGET):$(SRC) $(shell find -name "*.hpp")
	mkdir -p build
	g++ -std=c++17 $(INC) -g $(SRC) -o $@ 

run: $(TARGET)
	./$< example-input/config.toml

.phony:run